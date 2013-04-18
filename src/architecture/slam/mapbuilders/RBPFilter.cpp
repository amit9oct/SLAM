/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class RBPFilter
*
* Implements an slam algorithm consisting on a rao-blackwellized particle filter
*
*/

#include "RBPFilter.h"
#include "gridMapInterface.h"
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

#ifndef WIN32 
#include <sys/time.h>
#else
#include <process.h>
#endif

using namespace std;

// Para registrar la clase en la factoria
namespace{
	slamInterface* CreateRBPFilter(int bots, ConfigFile& file){
		return new RBPFilter(bots, file);
	}
	const int RBPF = 0;
	const bool registered = slamFactory::Instance().Register(RBPF, slamCreator(CreateRBPFilter));
}

// Constructor
RBPFilter::RBPFilter():
	vslamFilter(),
	M(0),
	p(0),
	newp(0),
	index(0),
	dispRobot(0),
	odoData(0),
	lastOdoData(0),
	rsData(0),
	lmData(0),
	endSlam(false),
	onlyonegridmap(true),
	omap(0),
	ppmap(0),
	ipmap(0),
	Identity3(matrix::identity(3)),
	mahTh(0.0f)
{
//	printf("RBPF default constructor\n");
}

RBPFilter::RBPFilter(int nrobots, ConfigFile& configFile):
	vslamFilter		(nrobots,configFile),
	M			(nrobots*configFile.read<int>("PARTICLESPERROBOT")),
	p			(new particle[M]()),
	newp			(new particle[M]()),
	index			(0),
	dispRobot		(new float[nBots]),
	odoData			(new pose*[nBots]),
	lastOdoData		(new pose*[nBots]),
	rsData			(new rangeSensorData*[nBots]),
	lmData			(new landmarksData*[nBots]),
	endSlam			(true),
	onlyonegridmap		(configFile.keyExists("ONLYONEGRIDMAP")? configFile.read<bool>("ONLYONEGRIDMAP") : true ),
	omap			(0),
	ppmap			(0),
	ipmap			(0),
	Identity3		(matrix::identity(3)),
	mahTh			(configFile.read<float>("MAHALANOBISTH"))
{
	if (onlyonegridmap){
		omap = gridMapFactory::Instance().CreateObject(gmtype,width, height, resolution, xorigin, yorigin);
		ppmap = new binMap(gmwidth, gmheight, resolution, xorigin, yorigin);
		ipmap = new binMap(gmwidth, gmheight, resolution, xorigin, yorigin);
	}
	int gt = (onlyonegridmap)? -1: gmtype;
	for (int m = 0; m< M; m++)
		p[m].initialize(nrobots, width, height, resolution, xorigin, yorigin, gt, vmtype, nmarks, configFile);
	for (int r = 0; r< nBots; r++){
		dispRobot[r] = 0.0f;
	}
//	printf("RBPF created\n");
}

RBPFilter::~RBPFilter(){
//	printf("RBPF destroyer...");
	stop();
	if (logstr){
		char myfilestr[100];
		if (onlyonegridmap){
			sprintf(myfilestr,"%somap.jpg",logstr);
			omap->saveMapAsImage(myfilestr);
			sprintf(myfilestr,"%sppmap.jpg",logstr);
			ppmap->saveMapAsImage(myfilestr);
		}
		else{
			sprintf(myfilestr,"%somap.jpg",logstr);
			p[index].getOMap().saveMapAsImage(myfilestr);
			sprintf(myfilestr,"%sppmap.jpg",logstr);
			p[index].getPrecisePoseMap().saveMapAsImage(myfilestr);
		}
		sprintf(myfilestr,"%svmap",logstr);
		p[index].getVMap().saveMap(myfilestr);
	}
	if (p)				delete[] p;
	if (newp)			delete[] newp;
	if (dispRobot)		delete[] dispRobot;
	if (odoData)		delete[] odoData;
	if (lastOdoData)	delete[] lastOdoData;
	if (rsData)			delete[] rsData;
	if (lmData)			delete[] lmData;
	if (omap)			delete omap;
	if (ppmap)			delete ppmap;
	if (ipmap)			delete ipmap;

//	printf("RBPF destroyed\n");
}

// Initializer
void RBPFilter::initialize(int nrobots, ConfigFile& configFile){
	vslamFilter::initialize(nrobots, configFile);
	
	M			= configFile.read<int>("PARTICLESPERROBOT")*nBots;
	if (p)			delete[] p;
	p			= new particle[M]();
	if (newp)		delete[] newp;
	newp			= new particle[M]();
	index			= 0;
	if (dispRobot)	delete[] dispRobot;
	dispRobot		= new float[nBots];
	if (odoData)	delete[] odoData;
	odoData			= new pose*[nBots];
	if (lastOdoData) delete[] lastOdoData;
	lastOdoData		= new pose*[nBots];
	if (rsData)		delete[] rsData;
	rsData			= new rangeSensorData*[nBots];
	if (lmData)		delete[] lmData;
	lmData			= new landmarksData*[nBots];
	endSlam			= false;
	onlyonegridmap		= configFile.keyExists("ONLYONEGRIDMAP")? configFile.read<bool>("ONLYONEGRIDMAP") : true ;
	mahTh			= configFile.read<float>("MAHALANOBISTH");

	if (onlyonegridmap){
		if (omap)  delete omap;
		if (ppmap) delete ppmap;
		if (ipmap) delete ipmap;
		omap = gridMapFactory::Instance().CreateObject(gmtype, width, height, resolution, xorigin, yorigin);
		ppmap = new binMap(gmwidth, gmheight, resolution, xorigin, yorigin);
		ipmap = new binMap(gmwidth, gmheight, resolution, xorigin, yorigin);
	}
	
	int gt = (onlyonegridmap)? -1: gmtype;
	for (int m = 0; m< M; m++)
		p[m].initialize(nrobots, width, height, resolution, xorigin, yorigin, gt, vmtype, nmarks, configFile);
	for (int r = 0; r< nBots; r++){
		dispRobot[r] = 0.0f;
	}
}

int RBPFilter::setup(){
	prio = 50;
	return 0;
}

void RBPFilter::onStop(){
	if (!endSlam){
		endSlam = true;
		closing.lock();
		closing.unlock();
	}
	printf("[FastSLAM] SLAM stopped\n");
}

void RBPFilter::execute(){
	printf("[FastSLAM] \t\t\t\t\t\t Running SLAM\n");
	closing.lock();
	int r;

	// open the log file
	FILE* slamlog=0;
	if (logstr){
		char myfilestr[100];
		sprintf(myfilestr,"%sslam.m",logstr);	
		slamlog = fopen(myfilestr,"w");
		fprintf(slamlog,"poses_est=[");
	}

	if (displayomap)  cvNamedWindow("OMAP",  CV_WINDOW_AUTOSIZE);
	if (displayppmap) cvNamedWindow("PPMAP", CV_WINDOW_AUTOSIZE);
	if (displayipmap) cvNamedWindow("IPMAP", CV_WINDOW_AUTOSIZE);
	
	// inicializamos SLAM
	
//	printf("[FastSLAM 1] initializing slam...\n");
	for (r=0; r<nBots; r++){
		if (rbase[r]) rbase[r]->beginConsumition();	
		lastOdoData[r] = rbase[r]->getOdometry();
		//printf("initial pose r%d: x:%f, y%f, th=%f\n",r,lastOdoData[r]->x,lastOdoData[r]->y,lastOdoData[r]->th);
		for (int i = 0; i < M; i++){
			p[i].setPos(*(lastOdoData[r]),r);
		}
		if (rbase[r]) rbase[r]->endConsumition();
	}
//	printf("[FastSLAM 1] slam initialized\n");
	
	float cw = 1.0f/M;
	for (int i = 0; i < M; i++){
		p[i].setWeight(cw); 	// Resetamos el peso de las particulas
	}

	endSlam=false;
	while (!endSlam){
		//printID("SLAM RBPF");
		//Sleep(10);
		// Leemos sensores
		
//		printf("[FastSLAM 1] slam step\n");
		for (r=0; r<nBots; r++){
			if (robotsEnabled[r]){
				rbase[r]->beginConsumition();
				if (robotsEnabled[r]){
					odoData[r] = rbase[r]->getOdometry();
					rsData[r]  = rbase[r]->getRangeSensorData();
					lmData[r]  = rbase[r]->getLandmarksData();
				}
				rbase[r]->endConsumition();
			}
		}
//		printf("[FastSLAM 1] consumed\n");
		
//		printf("[FastSLAM 1] fastslam found: %d, in the map: %d\n",lmData[0]->getNLandmarks(), p[index].getVMap().getNLandmarks());

		// Fastslam
		for (r=0; r<nBots; r++){
			if (robotsEnabled[r]){
				pose deltaOdo = *(odoData[r]) - *(lastOdoData[r]);
				fastSlam(r, *(lmData[r]), *(lastOdoData[r]), deltaOdo);
			}
		}
//		printf("[FastSLAM 1] resample\n");
		double neff = resample();								//	resampling
//		printf("[FastSLAM 1] neff=%f\n",neff);
		//printf("estimated pose x: %f, y: %f, th: %f\n",p[index].getPos(0).x,p[index].getPos(0).y,p[index].getPos(0).th);

//		printf("[FastSLAM 1] update\n");
		for (r=0; r<nBots; r++)	updateAll(*(rsData[r]),r);		// updating occupation map
		step();
//		printf("[FastSLAM 1] clean data\n");
		// delete sensor data
		for (r=0; r<nBots; r++){
			if (robotsEnabled[r]){
				if (lastOdoData[r]) delete lastOdoData[r];
				lastOdoData[r] = odoData[r];
				if (rsData[r]) delete rsData[r];
				if (lmData[r]) delete lmData[r];
			}
		}

		emit slamUpdated();

/*		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  show maps  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		printf("\t\t\tshow\n");
		IplImage* im, *im2, *im3;
		if (displayomap){
			im = (onlyonegridmap)? omap->getMapAsImage(): p[index].getOMap().getMapAsImage();
			CvScalar redcolor = CV_RGB(255,0,0);
			int hinv = gmheight-1;
			if (displayposes)						// display all particles
				for(int m=0; m<M; m++)
					for (r=0; r<nBots; r++)
						cvCircle( im, cvPoint(p[m].getCell(r).x, hinv - p[m].getCell(r).y), 0, redcolor, 1 );
			if (displayfeatures) p[index].getVMap().drawMarks(*im,xorigin,yorigin,resolution);
			if (windowName) cvShowImage(windowName,im);
			else cvShowImage("OMAP",im);		}
		if (displayppmap){
			im2 = (onlyonegridmap)? ppmap->getMapAsImage(): p[index].getPrecisePoseMap().getMapAsImage();
			cvShowImage("PPMAP",im2);
		}
		if (displayipmap){
			im3 = (onlyonegridmap)? ipmap->getMapAsImage(): p[index].getImprecisePoseMap().getMapAsImage();
			cvShowImage("IPMAP",im3);
		}
		
		if (displayomap || displayppmap || displayipmap) cvWaitKey(10);
		if (displayomap)  cvReleaseImage(&im);
		if (displayppmap) cvReleaseImage(&im2);
		if (displayipmap) cvReleaseImage(&im3);
		
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  end show maps  
*/		
		// write logs
		if (logstr){
			fprintf(slamlog,"%d",getStep());
			for (r=0; r<nBots;r++){
				fprintf(slamlog,", %f, %f, %f, %f",p[index].getPos(r).x, p[index].getPos(r).y, p[index].getPos(r).th, getDisp(r));
			}
			fprintf(slamlog,", %f\n", neff);
		}
		
//		printf("[FastSLAM 1] end loop\n");
	}// while (!endSlam)
	
	if (logstr){
		fprintf(slamlog,"];");	
		fclose(slamlog);
	}
	
	if (displayomap)  cvDestroyWindow("OMAP");
	if (displayppmap) cvDestroyWindow("PPMAP");
	if (displayipmap) cvDestroyWindow("IPMAP");
	
//	printf("[FastSLAM 1] Stopping SLAM\n");
	closing.unlock();
}


void RBPFilter::updateParticle(int robot, const landmarksData& lmData, const pose& lastOdo, const pose& deltaOdo, int i){
	
	// Hacemos avanzar la pose del robot en la particula segun la odometria y el ruido
	p[i].setPos(odometryModel(lastOdo, deltaOdo, p[i].getPos(robot) ),robot);
	
	matrix G = jacobian(p[i].getPos(robot));							// jacobiano
	matrix Ginv = G.inverse();
	matrix Gtrans = G.transpose();
	matrix Ginvtrans = Ginv.transpose();
	
	//printf("marcas %d\n",lmData.getNLandmarks());
	for (int j = 0; j < lmData.getNLandmarks(); j++){ // Se aplica de manera repetida para todas las medidas realizadas. Se multiplican los pesos
		
		pos3d lpos  = base2global(p[i].getVMap().cam2base(lmData.getLandmark(j).pos), p[i].getPos(robot));	// medida en coordenadas globales	
		matrix ZT	= lmData.getLandmark(j).pos;											// Medida (coordenadas de camara)
		matrix Rt	= lmData.getLandmark(j).covariance;										// covarianza de la medida
		
		//printf("mark medi %d, (x: %f, y: %f, z: %f)\n", j, lmData.getLandmark(j).pos.getX(), lmData.getLandmark(j).pos.getY(), lmData.getLandmark(j).pos.getZ());
		//printf("mark real %d, (x: %f, y: %f, z: %f)\n", j, lpos.getX(), lpos.getY(), lpos.getZ());
		
		// associacion de datos
		landmark* mark = (perfectMatching)?
			p[i].getVMap().getLandmarkByDesc(lmData.getLandmark(j).descriptor) :
			p[i].getVMap().dataAssociation(ZT, Rt, lpos, p[i].getPos(robot), G, Gtrans, lmData.getLandmark(j).descriptor) ;
			
		
        // Si es una nueva landmark, la inicializamos
		if(!mark) {
			mark = new landmark(lpos,													// global pos
								Ginv*(Rt*(Ginvtrans)),									// covariance
								lmData.getLandmark(j).descriptor,						// descriptor
								lmData.getLandmark(j).desclength,						// descriptor length
								0);														// repeated
			p[i].getVMap().addLandmark(*mark);											// la insertamos en el mapa de la particula
			p[i].setWeight(p[i].getWeight()*mahTh);										// Dividimos el peso de la particula
		}
		// Si es una landmark anterior, le calculamos su EKF
        else {
			matrix zgorro	= p[i].getVMap().base2cam(global2base(mark->pos, p[i].getPos(robot)));		// medida estimada
            matrix In		= (ZT - zgorro);											// innovacion
            matrix Z		= ((G*(mark->covariance*(Gtrans))) + Rt);					// covarianza de la innovacion
            matrix Zinv		= Z.inverse();
			matrix K		= (mark->covariance*((Gtrans)*(Zinv)));						// ganancia
            mark->pos		= ((K*In)+mark->pos).toPos3d();								// actualizamos pose
            mark->covariance= (Identity3-K*G)*mark->covariance;							// actualizamos covarianza
			
			for (int k=0; k< mark->desclength; k++)
				mark->descriptor[k] = (mark->descriptor[k] + lmData.getLandmark(j).descriptor[k])/2;

            p[i].getVMap().changeLastReturnedLandmark(*mark);
			
			// Calculamos el nuevo peso
            p[i].setWeight(p[i].getWeight()*exp(-0.5*visualMap::mahalanobis(In.toPos3d(), Z)) /(sqrt((Z*PIx2).det())));
		}
		delete mark;
	}
}

// fastslam algorithm
void RBPFilter::fastSlam(int robot, const landmarksData& lmData, const pose& lastOdo, const pose& deltaOdo){
	int i;

	//printf("\t\t\tfastslam...\n");
	// update particles
	for (i = 0; i < M; i++){		
		updateParticle(robot,lmData,lastOdo,deltaOdo,i);
	}
	//printf("\t\t\tok\n");
	
	// Normalize weight
	double suma = 0;
	//double sumalog = 0;
	for (i = 0; i < M; i++)	suma += p[i].getWeight();
	if (suma > 0){
		for (i = 0; i < M; i++){
			p[i].setWeight(p[i].getWeight()/suma);
			p[i].setSumLogWeight(p[i].getSumLogWeight() + log(p[i].getWeight()));
		}
	}
	else{
//		printf("[FastSLAM 1] Null sum of weigths!!!! - Resetting to ones\n");
		double invM = 1.0f/M;
		for (int i = 0; i < M; i++){
			p[i].setWeight(invM);
			p[i].setSumLogWeight(0.0);
		}
	}
}

// update occupancy, precise poses and imprecise poses maps
void RBPFilter::updateAll(const rangeSensorData& rsData, int r ){
	evalDisp(r);
	if (onlyonegridmap){
		omap->update(rsData, p[index].getPos(r), getDisp(r));
		updatePPMap(p[index].getCell(r), *ppmap, getDisp(r), badlocalized[r]);
		updateIPMap(p[index].getCell(r), *ipmap, getDisp(r), badlocalized[r]);
	}
	else{
		for (int m = 0; m < M ; m++){
			p[m].getOMap().update(rsData, p[m].getPos(r), getDisp(r));
			updatePPMap(p[m].getCell(r), p[m].getPrecisePoseMap(), getDisp(r), badlocalized[r]);
			updateIPMap(p[m].getCell(r), p[m].getImprecisePoseMap(), getDisp(r), badlocalized[r]);
		}
	}
}


// importance resampling
double RBPFilter::resample(){

	int i,m;
	double suma = 0;
	int newindex = 0;
	double maxweight = p[0].getWeight();

	for (i = 0; i < M; i++){
		suma += p[i].getWeight()*p[i].getWeight();
		if(p[i].getSumLogWeight() > maxweight){
			newindex = i;
			maxweight = p[i].getSumLogWeight();
		}
	}
	double Neff = 1/suma;

	if (Neff > M + 1 || Neff < 1){ 
//		printf("[FastSLAM] error\n");
		exit(0);
	}

	if (Neff < M){
		
		// muestreo con resposicion de M muestras equiespaciadas aleatorio
		int* resamp = new int[M];
		
		double r = ((double(rand())+1)/(double(RAND_MAX)+2))/M;			// me ha fallado alguna vez
		//double r = 0.5/M;												// tambien se puede poner fijo a la mitad
		double u = r;
		int j = 0;
		double c = p[0].getWeight();
		
		double inc = 1.0/M;
		for (m = 0; m < M; m++){
			while(u > c){
				j++;
				if (j>=M) {
//					printf("[FastSLAM] ERROR!!!\n"); //Sleep(100000);
				}
				c += p[j].getWeight();
			}
			resamp[m]=j;
			u += inc;
		}

		double w0 = 1.0/M;
		for (i = 0; i < M; i++){
			p[i].setWeight(w0);
		}

		// RESAMPLING
		sampling.lock();
		for (m = 0; m < M; m++){
			newp[m] = p[resamp[m]];
		}
		index = resamp[newindex];
		particle* aux = p;
		p = newp;
		newp = aux;
		sampling.unlock();

		for (m = 0; m < M; m++) newp[m].releaseVMap();
		
		delete[] resamp;
	}
	return Neff;
}

// Evaluates th uncertainty in the position for each robot
void RBPFilter::evalDisp(int r){
	int j;
	float sigma;
	float meanx, meany, meanth;
	double sum=M;

	meanx=0; meany=0; meanth=0;
	for (j =0; j < M ; j++){
		meanx  += p[j].getPos(r).x ;
		meany  += p[j].getPos(r).y ;
		meanth += p[j].getPos(r).th;
		//sum += p[j].getSumLogWeight();
	}

	meanx/= sum; meany/= sum; meanth/= sum;
	sigma = 0;
	for (j =0; j < M ; j++){
		sigma += (pow(p[j].getPos(r).x  - meanx,2) + pow(p[j].getPos(r).y  - meany,2));
	}	
	sigma = sqrt(sigma/sum);

	dispRobot[r] = sigma;

	//printf("[FastSLAM 1] sigma = %f\n",sigma);
	// LOCALIZATION HYSTERESIS LOOP 
	if (badlocalized[r] == false && sigma>th_high)	badlocalized[r] = true;
	if (badlocalized[r] == true && sigma<th_low)	badlocalized[r] = false;
}

matrix RBPFilter::jacobian(const pose& pos){

	matrix res(3,3);

	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);

	res.set(0,0,costh);
	res.set(0,1,sinth);
	res.set(1,0,-sinth);
	res.set(1,1,costh);
	res.set(2,2,1.0f);

	return res;
}

//TODO eval a covariance from the particles
matrix RBPFilter::getCovariance(int robot) const{
	matrix mat(3,3);
	return mat;
};



gridMapInterface* RBPFilter::getOMap() {
	sampling.lock();
	gridMapInterface* map = (onlyonegridmap)? omap->clone() : p[index].getOMap().clone();
	sampling.unlock();
	return map;
}

visualMap* RBPFilter::getVMap() {
	sampling.lock();
	visualMap* map = p[index].getVMap().clone();
	sampling.unlock();
	return map;
}

binMap* RBPFilter::getPreMap() {
	sampling.lock();
	binMap* map = new binMap( (onlyonegridmap)? *ppmap : p[index].getPrecisePoseMap() );	
	sampling.unlock();
	return map;
}

binMap* RBPFilter::getImpMap() {
	sampling.lock();
	binMap* map = new binMap( (onlyonegridmap)? *ipmap : p[index].getImprecisePoseMap() );
	sampling.unlock();
	return map;
}

pose RBPFilter::getPos(int robot) {
	sampling.lock();
	pose pos =  p[index].getPos(robot);
	sampling.unlock();
	return pos;
}

point RBPFilter::getCell(int robot) {
	sampling.lock();
	point c = p[index].getCell(robot);
	sampling.unlock();
	return c;
}

Ematrix RBPFilter::getGlobalCovariance() const{

	return Ematrix();

}

QPixmap* RBPFilter::getPixmap(){
//	IplImage* img = omap->getMapAsImage();
//	cvSaveImage("imgoctmp.jpg",img);
//	cvReleaseImage(&img);
//	QPixmap* pix = new QPixmap("imgoctmp.jpg");
//	return pix; 

	gridMapInterface* map = getOMap();
 	visualMap* vismap = getVMap();

	IplImage* img = map->getMapAsImage();
	for (int r=0; r<nBots; r++)
		drawPoses (*img, xorigin,  yorigin,  resolution);
	vismap->drawMarks(*img,xorigin,yorigin,resolution);
	
	delete map;
	delete vismap; 

	QImage* qim = new QImage((const uchar *)img->imageData, img->width, img->height, img->widthStep, QImage::Format_RGB888);
	QPixmap* mapfig = new QPixmap(QPixmap::fromImage(*qim));
	cvReleaseImage(&img);
	delete qim;

	return mapfig;

}


void RBPFilter::drawPoses (IplImage& im, float xorigin, float yorigin, float resolution) {

	CvScalar redcolor = CV_RGB(255,0,0);
	sampling.lock();
	for (int r=0; r<nBots; r++){
		for (int m = 0; m < M ; m++){
			point cell = p[m].getCell(r);
			cvEllipse(&im, cvPoint(cell.x,im.height-cell.y), cvSize(0,0), 0, 0, 360, redcolor, CV_FILLED );
		}
	}
	sampling.unlock();
}

