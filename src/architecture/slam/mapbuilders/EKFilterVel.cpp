#include "EKFilterVel.h"
#include <opencv2/opencv.hpp>
#include <list>

#ifndef WIN32
#include "sys/time.h"
#endif

using namespace std;

// Para registrar la clase en la factoria
namespace{
	slamInterface* CreateEKFilterVel(int bots, ConfigFile& file){
		return new EKFilterVel(bots, file);
	}
	const int EKF = 2;
	const bool registered = slamFactory::Instance().Register(EKF, slamCreator(CreateEKFilterVel));
}

/// Constructor
EKFilterVel::EKFilterVel():
	vslamFilter(),
	robotcells(0),
	dispRobot(0),
	odoData(0),
	lastOdoData(0),
	rsData(0),
	lmData(0),
	endSlam(false),
	omap(0),
	ppmap(0),
	ipmap(0),
	vmap(0),
	Identity3(matrix::identity(3)),
	maxCorrect(0.0f),
	minIncorrect(999999999.9f)
{
	//printf("[EKF] EKF default constructor\n");
}

/// Constructor
EKFilterVel::EKFilterVel(int nrobots, ConfigFile& configFile):
	vslamFilter		(nrobots,configFile),
	state			(6*nrobots,1,6*nrobots+3*nmarks,1),
	covariance		(6*nrobots,6*nrobots,6*nrobots+3*nmarks,6*nrobots+3*nmarks),
	robotcells		(new point[nBots]),
	dispRobot		(new float[nBots]),
	odoData			(new pose*[nBots]),
	lastOdoData		(new pose*[nBots]),
	rsData			(new rangeSensorData*[nBots]),
	lmData			(new landmarksData*[nBots]),
	endSlam			(false),
	omap			(gridMapFactory::Instance().CreateObject(gmtype,width, height, resolution, xorigin, yorigin)),
	ppmap			(new binMap(gmwidth, gmheight, resolution, xorigin, yorigin)),
	ipmap			(new binMap(gmwidth, gmheight, resolution, xorigin, yorigin)),
	vmap			(visualMapFactory::Instance().CreateObject(vmtype,nmarks,configFile)),
	Identity3		(matrix::identity(3)),
//	range			(configFile.read<float>("SEARCHRANGE")),
	mahTh			(configFile.read<float>("MAHALANOBISTH")),
	descTh			(configFile.read<float>("DESCRIPTORTH")),
	nearestNeighbourOrderBy (configFile.read<int>("NEARESTNEIGHBOURBY")),
	maxCorrect		(0.0f),
	minIncorrect		(999999999.9f)
{
	for (int r = 0; r< nBots; r++){
		dispRobot[r] = 0.0f;
		rbase[r] = 0;
		odoData[r] = 0;
		lastOdoData[r] = 0;
		rsData[r] = 0;
		lmData[r] = 0;
		robotsEnabled[r]=true;
	}
	printf("[EKF2] EKF created\n");
}

/// Destructor
EKFilterVel::~EKFilterVel(){
	//printf("[EKF] EKF destroyer...");
	stop();
	if (logstr){
		char myfilestr[100];
		sprintf(myfilestr,"%somap.jpg",logstr);
		omap->saveMapAsImage(myfilestr);
		sprintf(myfilestr,"%sppmap.jpg",logstr);
		ppmap->saveMapAsImage(myfilestr);
		sprintf(myfilestr,"%svmap",logstr);
		vmap->saveMap(myfilestr);
	}
	if (robotcells)			delete[] robotcells;
	if (dispRobot)			delete[] dispRobot;
	if (odoData){			for (int i=0; i< nBots; i++) if (odoData[i]) delete odoData[i];
					delete[] odoData;
	}
	if (lastOdoData){		for (int i=0; i< nBots; i++) if (lastOdoData[i]) delete lastOdoData[i];
					delete[] lastOdoData;
	}
	if (rsData){			for (int i=0; i< nBots; i++) if (rsData[i]) delete rsData[i];
					delete[] rsData;}
	if (lmData){			for (int i=0; i< nBots; i++) if (lmData[i]) delete lmData[i];
					delete[] lmData;}
	if (omap)			delete omap;
	if (ppmap)			delete ppmap;
	if (ipmap)			delete ipmap;
	if (vmap)			delete vmap;
	//printf("EKF destroyed\n");
}

///initializer	
void EKFilterVel::initialize(int nrobots, ConfigFile& configfile){
	vslamFilter::initialize(nrobots, configfile);
	
	state.initialize(6*nrobots,1,6*nrobots+3*nmarks,1);
	covariance.initialize(6*nrobots,6*nrobots,6*nrobots+3*nmarks,6*nrobots+3*nmarks);
	
	if (robotcells)		delete[] robotcells;
	robotcells		= new point[nBots];
	if (dispRobot)		delete[] dispRobot;
	dispRobot		= new float[nBots];
	if (odoData)		delete[] odoData;
	odoData			= new pose*[nBots];
	if (lastOdoData) 	delete[] lastOdoData;
	lastOdoData		= new pose*[nBots];
	if (rsData)		delete[] rsData;
	rsData			= new rangeSensorData*[nBots];
	if (lmData)		delete[] lmData;
	lmData			= new landmarksData*[nBots];
	endSlam			= false;
	if (omap)		delete omap;
	omap			= gridMapFactory::Instance().CreateObject(gmtype, width, height, resolution, xorigin, yorigin);
	if (ppmap)		delete ppmap;
	ppmap			= new binMap(gmwidth, gmheight, resolution, xorigin, yorigin);
	if (ipmap)		delete ipmap;
	ipmap			= new binMap(gmwidth, gmheight, resolution, xorigin, yorigin);
	if (vmap)		delete vmap;
	vmap			= visualMapFactory::Instance().CreateObject(vmtype,nmarks,configfile);
	
	for (int r = 0; r< nBots; r++){
		dispRobot[r] = 0.0f;
		rbase[r] = 0;
		odoData[r] = 0;
		lastOdoData[r] = 0;
		rsData[r] = 0;
		lmData[r] = 0;
		robotsEnabled[r]=true;
	}
}

/// set up for the execution thread
int EKFilterVel::setup(){
	prio = 50;
	return 0;
}

/// called before stopping
void EKFilterVel::onStop(){
	if (!endSlam){		
		endSlam = true;
		//scene->beginProduction();
		//scene->endProduction();
		closing.lock();
		closing.unlock();
	}
	printf("[EKF2] SLAM stopped\n");
}

/// primary execution loop
void EKFilterVel::execute(){
	closing.lock();
	printf("[EKF2] \t\t\t\t\t\t\t Running SLAM\n");
	endSlam=false;
	int r;

	// open the log file
	FILE* slamlog=0;
	//CvVideoWriter* avifile=0;

	if (logstr){
		char myfilestr[100];
		sprintf(myfilestr,"%sslam.m",logstr);	
		slamlog = fopen(myfilestr,"w");
		fprintf(slamlog,"poses_est=[");

	}

	// initialize SLAM
	for (r=0; r<nBots; r++){
		if (rbase[r]) rbase[r]->beginConsumition();
		odoData[r] = rbase[r]->getOdometry();
		state.set(r*6,0,odoData[r]->x);			// state [ x      ]
		state.set(r*6+1,0,odoData[r]->y);		//       [ y      ]
		state.set(r*6+2,0,odoData[r]->th);		//       [ theta  ]
		state.set(r*6+3,0,0);				//       [ vx     ]
		state.set(r*6+4,0,0);				//       [ vy     ]
		state.set(r*6+5,0,0);				//       [ vtheta ]
   
		pose pos = getPos(r);
		robotcells[r].x = (int) floor((pos.x - xorigin)/resolution);
		robotcells[r].y = (int) floor((pos.y - yorigin)/resolution);
		if (rbase[r]) rbase[r]->endConsumition();
	}
	covariance.clear();
	
	int nBotsx3 = 6*nBots;
	matrix Ft = matrix::identity(nBotsx3);	
	Ematrix Qt(nBotsx3,nBotsx3);
	errorcount = 0;

	// main loop
	while (!endSlam){
		
		// read sensors
		int nummarks=0;
		for (r=0; r<nBots; r++){
			if (robotsEnabled[r]){
				rbase[r]->beginConsumition();
				if (robotsEnabled[r]){
					if (lastOdoData[r]) delete lastOdoData[r];		// I am supossed, not to use this
					lastOdoData[r] = odoData[r];
					odoData[r] = rbase[r]->getOdometry();
					if (rsData[r]) delete rsData[r];
					rsData[r]  = rbase[r]->getRangeSensorData();
					if (lmData[r]) delete lmData[r];
					lmData[r]  = rbase[r]->getLandmarksData();
					nummarks += lmData[r]->getNLandmarks();
				}
				rbase[r]->endConsumition();
			}
		}
		//printf("Marks observed %d \n", nummarks);
		int nmarksprev = vmap->getNLandmarks();
		int nmarksprev3 = 3*nmarksprev;
		
		Ematrix G(nBotsx3+nmarksprev3,2);
		Ematrix& xpkmas1 = state;
		Ematrix& Ppkmas1 = covariance;


		// predict
		for (r=0; r<nBots; r++){
			if (robotsEnabled[r]){
				int r3 = 6*r;
				// pose
				pose xrk = getPos(r);									// cogemos la pose del estado		
				pose vxrk = getVel(r);
				xrk.print("pos=");
				vxrk.print("speed=");

				// odo inc
//				pose deltaOdo = *(odoData[r]) - *(lastOdoData[r]);					// incremento de odo
//				if (deltaOdo.th>PI)		deltaOdo.th -= PIx2;
//				else if (deltaOdo.th <= -PI)	deltaOdo.th += PIx2;
				
				// prediction
				xpkmas1.set(r3, 0, motionModel (xrk, vxrk));						// modelo de movimiento
				Ft.set(r3,r3,jacobianF(xrk, vxrk));							// ampliamos el jacobiano F						
				//Ft.print("Ft=");
				Ematrix G = jacobianG(xrk, vxrk);							// jacobiano G
				//G.print("G=");
				Ematrix Q=noiseQ();
				//Q.print("Q=");
				Qt.set(r3,r3,G*(Q*G.transpose()));						// noise
				//Qt.print("Qt=");
			}
		}
//		Ppkmas1.set(0,0, 0);

		matrix Fttrans = Ft.transpose();
		matrix Pvv = Ppkmas1.subMat(0,nBotsx3-1,0,nBotsx3-1);
		matrix Pvl = Ppkmas1.subMat(0,nBotsx3-1,nBotsx3,nBotsx3+nmarksprev3-1);

		Ppkmas1.set(0,0,Qt + ( Ft* (Pvv*Fttrans) ));
		Ppkmas1.set(0,nBotsx3,Ft*Pvl);
		Ppkmas1.set(nBotsx3,0,(Ppkmas1.subMat(0,nBotsx3-1,nBotsx3,nBotsx3+nmarksprev3-1)).transpose());
		
		//Pvv.print("Pvv=");

		// update		
		
		int nummarks3 = 3*nummarks;
		
		Ematrix H(0,nBotsx3+nmarksprev3,3*nummarks,nBotsx3+nmarksprev3+nummarks3);
		Ematrix ZT(0,1,nummarks3,1);
		Ematrix zgorro(0,1,nummarks3,1);
		Ematrix Rt(0,0,nummarks3,nummarks3);
		
		int markid=0;

		for (r=0; r<nBots; r++){
			if (robotsEnabled[r]){
				int r3 = 6*r;
				//int r3mas2 = r3+2; 
				pose xrpkmas1	= xpkmas1.subMat(r3,r3+2,0,0).toPose();	// pose	
				//pose vrpkmas1	= xpkmas1.subMat(r3+3,r3+5,0,0).toPose();	// speed	
				matrix H2	= jacobianH2(xrpkmas1);										// jacobiano H
				matrix H2trans	= H2.transpose();
				
				//if (lmData[r]->getNLandmarks()>2){
					for (int j = 0; j < lmData[r]->getNLandmarks(); j++){							// for each detected mark
						
						pos3d lpos 	= base2global(vmap->cam2base(lmData[r]->getLandmark(j).pos), xrpkmas1);		// medida en coordenadas globales	
						matrix ZTi	= lmData[r]->getLandmark(j).pos;						// Medida (coordenadas de camara)
						matrix Rti	= lmData[r]->getLandmark(j).covariance;						// covarianza de la medida
						
						// data association 
						landmark* mark = (perfectMatching)?
							vmap->getLandmarkByDesc(lmData[r]->getLandmark(j).descriptor) :
							dataAssociation(ZTi, Rti, lpos,  xrpkmas1, H2, H2trans, lmData[r]->getLandmark(j).descriptor,Ppkmas1,r) ;
						
						if (mark){	// if matching							
							//printf("matched mark \n");
							int markid3 = 3*markid;
							ZT.extend(3,0);
							ZT.set(markid3,0,ZTi);
							Rt.extend(3,3);
							Rt.set(markid3,markid3,Rti);
							matrix zgorroi	= vmap->base2cam(global2base(mark->pos, xrpkmas1));	// medida estimada
							zgorro.extend(3,0);
							zgorro.set(markid3,0,zgorroi);
							
							matrix H1= jacobianH1(xrpkmas1,mark->pos);
							int lid = vmap->returnLastReturnedId();
							H.extend(3,0);
							H.set(markid3,nBotsx3+3*lid,H2);
							H.set(markid3,r3,H1);
							
							printf("mark [id %f] in map index: %d ", lmData[r]->getLandmark(j).descriptor[0], lid);
							mark->pos.print("mark in map=");
							markid++;
							
							for (int k=0; k< mark->desclength; k++)
								mark->descriptor[k] = (mark->descriptor[k] + lmData[r]->getLandmark(j).descriptor[k])/2;
							vmap->changeLastReturnedLandmark(*mark);
						}
						else{ // new mark 
							//printf("new mark \n");
							//lpos.print("mark=");
							printf("mark [id %f] is new ", lmData[r]->getLandmark(j).descriptor[0]);
							lpos.print("mark in map=");							
							matrix Pl2 = H2trans*(Rti*(H2));
							
							mark = new landmark(	lpos,						// global pos
										Pl2,						// covariance
										lmData[r]->getLandmark(j).descriptor,		// descriptor
										lmData[r]->getLandmark(j).desclength,		// descriptor length
										0);						// repeated
							vmap->addLandmark(*mark);
							
							int numm = 3*(vmap->getNLandmarks()-1);
							
							// add the mark to the state map
							xpkmas1.extend(3,0);
							xpkmas1.set(nBotsx3+numm,0,matrix(lpos));
							//xpkmas1.subMat(nBotsx3+numm,nBotsx3+numm+2,0,0).print("init mark=");

							Ematrix J1 = jacobianJ1(xrpkmas1,lpos);							
							Ematrix R  = J1*Ppkmas1.subMat(r3,r3+5, 0, nBotsx3+numm-1);
							Ematrix Pr = Ppkmas1.subMat(r3,r3+5,r3,r3+5);
							//Ematrix Pl = J1*Pr*J1.transpose() + Pl2;
							Ematrix Pl = J1*Pr*J1.transpose() + Pl2;
							
							Ppkmas1.extend(3,3);
							Ppkmas1.set(0,nBotsx3+numm,R.transpose());
							Ppkmas1.set(nBotsx3+numm,0,R);
							Ppkmas1.set(nBotsx3+numm,nBotsx3+numm,Pl);
							
							H.extend(0,3);
						}
						if (mark) {delete mark; mark=0;}
					} // for each mark
				//}
			}
		}// for each robot

		if(markid){
			Ematrix Htrans		= H.transpose();
			Ematrix In		= (ZT - zgorro);				// innovacion
			for (int jjj = 0; jjj < markid; jjj++){ 
				int iii = 3*jjj+2;
				if (In.get(iii,0) > M_PI)  In.set(iii, 0, In.get(iii,0) - PIx2 );
				if (In.get(iii,0) < -M_PI) In.set(iii, 0, In.get(iii,0) + PIx2 );
			}
			Ematrix S		= ((H*(Ppkmas1.mul2(Htrans))) + Rt);		// covarianza de la innovacion
			Ematrix K		= (Ppkmas1.mul2((Htrans)*(S.inverse())));	// ganancia
			ZT.transpose().print("Zt=");
			zgorro.transpose().print("Zgorro=");
			In.transpose().print("In=");
			//printf("K %d x %d\n", K.getNumRows(), K.getNumCols());
			//printf("P %d x %d\n", Ppkmas1.getNumRows(), Ppkmas1.getNumCols());
			//printf("In %d x %d\n", In.getNumRows(), In.getNumCols());
			xpkmas1			= (K*In)+xpkmas1;				// actualizamos el estado
			Ppkmas1			= Ppkmas1-K.mul2(H*Ppkmas1);			// actualizamos covarianza
		}
		//int numm = 3*(vmap->getNLandmarks()-1);
		//xpkmas1.subMat(nBotsx3+numm,nBotsx3+numm+2,0,0).print("updated mark=");

		//state.print("state=");

		//float uncertainty = evalUncertainty();

		// pasar los valores de las matrices al mapa
		for (int l = 0; l < vmap->getNLandmarks(); l++){
			landmark* mark 		= vmap->getLandmarkById(l);
			if (mark){
				mark->pos		= state.subMat(nBotsx3+l*3,nBotsx3+l*3+2,0,0).toPos3d();
				mark->covariance 	= covariance.subMat(nBotsx3+l*3,nBotsx3+l*3+2,nBotsx3+l*3,nBotsx3+l*3+2);					
				vmap->changeLastReturnedLandmark(*mark);
				delete mark;
				mark = 0;
			}
		}
		//printf("[EKF] marcas en el mapa= %d\n",vmap->getNLandmarks());

		for (r=0; r<nBots; r++){
			if (robotsEnabled[r]){
				int r3 = 6*r;
				int r3mas2 = r3+2; 
				if (xpkmas1.get(r3mas2,r3mas2)>PI)			xpkmas1.set(r3mas2,r3mas2,xpkmas1.get(r3mas2,r3mas2)- PIx2);
				else if (xpkmas1.get(r3mas2,r3mas2) <= -PI)		xpkmas1.set(r3mas2,r3mas2,xpkmas1.get(r3mas2,r3mas2)+ PIx2);
				evalDisp(r);
				pose pos = getPos(r);
				robotcells[r].x = (int) floor((pos.x - xorigin)/resolution);
				robotcells[r].y = (int) floor((pos.y - yorigin)/resolution);
			
				omap->update(*(rsData[r]), pos, getDisp(r));					// updating occupation map
				updatePPMap(robotcells[r], *ppmap, getDisp(r), badlocalized[r]);
				updateIPMap(robotcells[r], *ipmap, getDisp(r), badlocalized[r]);
			}
		}
		step();

		emit slamUpdated();

		// write logs
		if (logstr){
			fprintf(slamlog,"%d",getStep());
			for (r=0; r<nBots;r++){
				pose p = state.subMat(r*6,r*6+2,0,0).toPose();
				pose vp = state.subMat(r*6+3,r*6+5,0,0).toPose();
				//fprintf(slamlog,", %f, %f, %f, %f, %f, %f, %f", p.x, p.y, p.th, getDisp(r),uncertainty,mediana,determinante);
				fprintf(slamlog,", %f, %f, %f, %f, %f, %f, %f", p.x, p.y, p.th, vp.x, vp.y, vp.th, getDisp(r));
			}
			fprintf(slamlog,"\n");
		}
		
	}// while (!endSlam)
	
	if (logstr){
		fprintf(slamlog,"];");	
		fclose(slamlog);
	}
	
	//printf("[EKF] Stopping SLAM\n");
	closing.unlock();
}

QImage* EKFilterVel::getQImage(){
//	IplImage* img = omap->getMapAsImage();
//	QImage* qim = new QImage((const uchar *)img->imageData, img->width, img->height, img->widthStep, QImage::Format_RGB888);
//	QImage* qim = slam->getQImage();
//	QPixmap mapfig = QPixmap::fromImage(*qim);
//	cvReleaseImage(&img);
//	return qim;
	return 0;
}

QPixmap* EKFilterVel::getPixmap(){
//	IplImage* img = omap->getMapAsImage();
//	cvSaveImage("imgoctmp.jpg",img);
//	cvReleaseImage(&img);
//	QPixmap* pix = new QPixmap("imgoctmp.jpg");
//	return pix; 

	IplImage* img = omap->getMapAsImage();
	for (int r=0; r<nBots; r++)
		drawPose (*img, getPos(r), getCovariance(r),  xorigin,  yorigin,  resolution);
	vmap->drawMarks(*img,xorigin,yorigin,resolution);
	
	QImage* qim = new QImage((const uchar *)img->imageData, img->width, img->height, img->widthStep, QImage::Format_RGB888);
	QPixmap* mapfig = new QPixmap(QPixmap::fromImage(*qim));
	cvReleaseImage(&img);
	delete qim;
	return mapfig;

}


// odometry noise matrix Q
Ematrix EKFilterVel::noiseQ(){
	Ematrix Q(3,3);
	Q.set(0,0,0.02);
	Q.set(1,1,0.00);
	Q.set(2,2,0.01);
	return Q;
}

// Motion model
matrix EKFilterVel::motionModel (const pose& pos, const pose& velo){
	float aux = (2*pos.th+velo.th*0.5)/2;
	if 	(aux>PI)		aux-= PIx2;
	else if (aux <= -PI)		aux+= PIx2;
	matrix X(6,1);
	X.set(0,0,velo.x*0.5*cos(aux)+pos.x);
	X.set(1,0,velo.x*0.5*sin(aux)+pos.y);
	X.set(2,0,pos.th+velo.th*0.5);
	if (X.get(2,0)>PI)			X.set(2,0,X.get(2,0)- PIx2);
	else if (X.get(2,0) <= -PI)		X.set(2,0,X.get(2,0)+ PIx2);
	X.set(3,0,velo.x);
	X.set(4,0,velo.y);
	X.set(5,0,velo.th);
	return X;
}

// jacobian F
matrix EKFilterVel::jacobianF(const pose& pos, const pose& velo){
	float aux = pos.th+(velo.th*0.5)/2;
	if 	(aux>PI)		aux-= PIx2;
	else if (aux <= -PI)		aux+= PIx2;
	matrix F(6,6);
	F.set(0,0, 1);
	F.set(0,2, -velo.x*0.5*sin(aux));
	F.set(0,3, 0.5*cos(aux));
	F.set(0,5, -velo.x*0.5*sin(aux)*0.25);

	F.set(1,1, 1);
	F.set(1,2, velo.x*0.5*cos(aux));
	F.set(1,3, 0.5*sin(aux));
	F.set(1,5, velo.x*0.5*cos(aux)*0.25);

	F.set(2,2,1);
	F.set(2,5,0.5);

	F.set(3,3,1);
	F.set(4,4,1);
	F.set(5,5,1);
	return F;
}

// jacobian G
Ematrix EKFilterVel::jacobianG(const pose& pos, const pose& velo){
	float aux = (2*pos.th+velo.th*0.5)/2;
	if 	(aux>PI)		aux-= PIx2;
	else if (aux <= -PI)		aux+= PIx2;
	Ematrix G(6,3);
	G.set(0,0, -0.5*velo.x*sin(aux));
	G.set(0,2, -velo.x*0.5*sin(aux)*0.25);
	G.set(1,0,  0.5*velo.x*cos(aux));
	G.set(1,2, velo.x*0.5*cos(aux)*0.25);
	G.set(2,0, 1);
	G.set(2,2, 0.5);
	G.set(3,0, 1);
	G.set(4,1, 1);
	G.set(5,2, 1);
	return G;
}

// jacobian H
matrix EKFilterVel::jacobianH1(const pose& pos, const pos3d& mark){
	matrix H(3,3);
	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);
	float auxx = mark.getX() - pos.x;
	float auxy = mark.getY() - pos.y;
	H.set(0,0,-costh);
	H.set(0,1,-sinth);
	H.set(0,2,-(auxx)*sinth + (auxy)*costh );
	H.set(1,0, sinth);
	H.set(1,1,-costh);
	H.set(1,2,-(auxx)*costh - (auxy)*sinth );
	return H;
}

matrix EKFilterVel::jacobianH2(const pose& pos){
	matrix H(3,3);
	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);
	H.set(0,0,costh);
	H.set(0,1,sinth);
	H.set(1,0,-sinth);
	H.set(1,1,costh);
	H.set(2,2,1.0f);
	return H;
}

// jacobian J1
matrix EKFilterVel::jacobianJ1(const pose& pos, const pos3d& mark){
	matrix J1(3,6);
	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);
	pos3d p = vmap->cam2base(mark);
	J1.set(0,0,1);
	J1.set(0,2,-p.getX()*sinth - p.getY()*costh);
	J1.set(1,1,1);
	J1.set(1,2, p.getX()*costh - p.getY()*sinth);
	J1.set(2,2,1);
	return J1;
}



// asociacion de datos
landmark* EKFilterVel::dataAssociation(const matrix& ZT, const matrix& Rt, const pos3d& globalpos, const pose& robotPos, const matrix& H2, const matrix& H2trans, const float* desc, const Ematrix& Pcomp, int r) {
	float minDist=descTh, dist;
	float maxDist=mahTh;
	//landmark* res = 0;
	landmark* tmpmark = 0;
	int markid=-1;
//	printf("Comienzo asociacion de datos>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	for (int i = 0; i< vmap->getNLandmarks(); i++){
		tmpmark = vmap->getLandmarkById(i);
	//	if (sqrt( pow(tmpmark->pos.getX()-globalpos.getX(),2)+ pow(tmpmark->pos.getY()-globalpos.getY(),2))<range){ 
			
			matrix H(3,6);
			H.set(0,0,jacobianH1(robotPos,tmpmark->pos));
			H.set(0,3,H2);
			matrix Htrans = H.transpose();
			matrix P(6,6);
			int r3=r*6;
			int nBotsx3 = 6*nBots;
			P.set(0,0,Pcomp.subMat(r3,r3+2,r3,r3+2));
			P.set(0,3,Pcomp.subMat(r3,r3+2,nBotsx3+i*3,nBotsx3+i*3+2));
			P.set(3,0,Pcomp.subMat(nBotsx3+i*3,nBotsx3+i*3+2,r3,r3+2));
			P.set(3,3,Pcomp.subMat(nBotsx3+i*3,nBotsx3+i*3+2,nBotsx3+i*3,nBotsx3+i*3+2));

			matrix zgorro	= vmap->base2cam(global2base(tmpmark->pos, robotPos));
			matrix In	= ZT - zgorro;
			matrix Z	= (H*(P*(Htrans))) + Rt; /////////////////////////////////////////////////////////////

			//float pmatch =  exp(-0.5*vmap->mahalanobis(In.toPos3d(), Z)) /(sqrt((Z*PIx2).det()));			
			float pmatch =  vmap->mahalanobis(In.toPos3d(), Z);			
			if (tmpmark->descriptor[0]==desc[0]){
				if (pmatch > maxCorrect){
					maxCorrect = pmatch;
/*					printf("Desc %f\n",desc[0]);
					printf("match = %f\n",pmatch);
					globalpos.print("new estimated pos=");
					tmpmark->pos.print("map position=");
					robotPos.print("robot pose=");
					ZT.print("ZT=");
					zgorro.print("zgorro=");
					P.print("P=");
					Rt.print("Rt=");
					H.print("H=");
					In.print("IN=");
					Z.print("Z=");
*/				}
			}
			else{
				if (pmatch < minIncorrect){
					minIncorrect = pmatch;
/*					printf("Desc %f\n",desc[0]);
					globalpos.print("new estimated pos=");
					tmpmark->pos.print("map position=");
					robotPos.print("robot pose=");
					ZT.print("ZT=");
					zgorro.print("zgorro=");
					P.print("P=");
					Rt.print("Rt=");
					H.print("H=");
					In.print("IN=");
					Z.print("Z=");
*/				}
			}
			//printf("maxCorrect=%f, minIncorrect = %f\n",maxCorrect,minIncorrect);
			if ( pmatch < mahTh){							// check mahalanobis
				dist = visualMap::descDist(tmpmark->descriptor, desc, tmpmark->desclength);
				if (dist < descTh){		// get the minor descriptor distance but bigger than descTh
					if (nearestNeighbourOrderBy == 0){ // nearest desc
						if(dist < minDist){ 
							markid = i;
							minDist = dist;
						}	
					}
					else{								// nearest mahalanobis
						if(pmatch < maxDist){ 
							markid = i;
							maxDist = pmatch;
						}
					}
				}
			}
		//}
		delete tmpmark; tmpmark=0;
	}
	//printf("fin de la asociacion de datos\n");
	landmark* match = 0;
	if (markid>=0){
		match = vmap->getLandmarkById(markid);
		//if (desc[0]!= match->descriptor[0]) printf("DATA ASSOCIATION ERRORRR!!!!!!!!!!!\n");
	}
	//printf("Fin asociacion de datos>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");

	return (markid>=0)? match:0;
}


// evaluates the uncertainty on the position of each robot
void EKFilterVel::evalDisp(int r){
	matrix covariance = getCovariance(r);

	CvMat* cov = cvCreateMat(2,2,CV_32FC1);
	CvMat* evects = cvCreateMat(2,2,CV_32FC1);
	CvMat* evals = cvCreateMat(2,1,CV_32FC1);
	cvmSet(cov,0,0, covariance(0,0));
	cvmSet(cov,0,1, covariance(0,1));
	cvmSet(cov,1,0, covariance(1,0));
	cvmSet(cov,1,1, covariance(1,1));

	cvEigenVV( cov, evects, evals, DBL_EPSILON);

	float a = 3*sqrt(cvmGet(evals,0,0));
	float b = 3*sqrt(cvmGet(evals,1,0));
	if (a != a) a = 0.0f;
	if (b != b) b = 0.0f;

	cvReleaseMat(&cov);
	cvReleaseMat(&evects);
	cvReleaseMat(&evals);

	dispRobot[r] = PI*a*b;

	if (badlocalized[r] == false && dispRobot[r]>th_high)	badlocalized[r] = true;
	if (badlocalized[r] == true && dispRobot[r]<th_low)	badlocalized[r] = false;
}

float EKFilterVel::evalUncertainty(){
/*
	CvMat* cov = cvCreateMat(covariance.getNumRows(),covariance.getNumCols(),CV_32FC1);
	CvMat* evects = cvCreateMat(covariance.getNumRows(),covariance.getNumCols(),CV_32FC1);
	CvMat* evals = cvCreateMat(1,covariance.getNumCols(),CV_32FC1);
	for (int i = 0 ; i < covariance.getNumRows(); i++)
		for (int j = 0 ; j < covariance.getNumCols(); j++)
			cvmSet(cov,i,j, covariance(i,j));

	cvEigenVV( cov, evects, evals, DBL_EPSILON);
*/
	float trace = 0;
	//list<float> eigenvalues;
	//determinante = 0.0;
	for (int i = 0; i < covariance.getNumRows(); i++){
		trace += covariance(i,i);
		//determinante += log(cvmGet(evals,0,i));
		//eigenvalues.push_back(cvmGet(evals,0,i));
	}
	trace /= covariance.getNumRows();
	//determinante -= log((float)covariance.getNumRows());

	//eigenvalues.sort();
	//mediana = 0.0f;
	//for (unsigned int v = 0; v < eigenvalues.size()/2; v++) eigenvalues.pop_back();
	//mediana = eigenvalues.back();

	//cvReleaseMat(&cov);
	//cvReleaseMat(&evects);
	//cvReleaseMat(&evals);

	return trace;
}



// maps
gridMapInterface* EKFilterVel::getOMap() {
	return omap->clone();
}

visualMap* EKFilterVel::getVMap() {
	return vmap->clone();
}

binMap* EKFilterVel::getPreMap() {
	return new binMap(*ppmap);	
}

binMap* EKFilterVel::getImpMap() {
	return new binMap(*ipmap);
}

/// returns the matrix that represents the covariance of robots and marks
Ematrix EKFilterVel::getGlobalCovariance() const{
	return(covariance);
};

/// returns the current pose of the robot
pose EKFilterVel::getPos(int r){
	return state.subMat(r*6,r*6+2,0,0).toPose();
}
/// returns the current velocity of the robot
pose EKFilterVel::getVel(int r){
	return state.subMat(r*6+3,r*6+5,0,0).toPose();
}
/// returns the current cell of the robot
point EKFilterVel::getCell(int robot){
	return robotcells[robot];
}

/// returns the matrix that represents the covariance of the position of the robot
matrix EKFilterVel::getCovariance(int r) const{
	return covariance.subMat(r*6,r*6+2,r*6,r*6+2);
}

/// returns a measure of the dispersion of the robot
float EKFilterVel::getDisp(int r) const{
	return dispRobot[r];
}

void EKFilterVel::drawPose (IplImage& im, const pose& pos, const matrix& covariance, float xorigin, float yorigin, float resolution) {
	
	//pos.print("pos=");
	//covariance.print("Cov=");
	//printf("%e %e %e\n",covariance.get(0,0),covariance.get(1,1),covariance.get(2,2));	
	
	CvScalar redcolor = CV_RGB(255,0,0);
	CvPoint p = cvPoint (   (int) floor ( (pos.x - xorigin)/resolution),				// posicion
				(int) (im.height-1-floor((pos.y - yorigin)/resolution)));
	CvPoint p2 = cvPoint (  (int) floor ( (pos.x+0.5*cos(pos.th) - xorigin)/resolution),				// posicion
				(int) (im.height-1-floor((pos.y+0.5*sin(pos.th) - yorigin)/resolution)));

	CvMat* cov = cvCreateMat(2,2,CV_32FC1);
	CvMat* evects = cvCreateMat(2,2,CV_32FC1);
	CvMat* evals = cvCreateMat(2,1,CV_32FC1);
	cvmSet(cov,0,0, covariance(0,0));
	cvmSet(cov,0,1, covariance(0,1));
	cvmSet(cov,1,0, covariance(1,0));
	cvmSet(cov,1,1, covariance(1,1));

	cvEigenVV( cov, evects, evals, DBL_EPSILON);

	float a = 3*sqrt(cvmGet(evals,0,0));
	float b = 3*sqrt(cvmGet(evals,1,0));
	float alfa;

	if (a != a) a = 0.0f;
	if (b != b) b = 0.0f;
	if (a<b){
		float tmp = a;
		a = b;
		b = tmp;
		alfa = 180.0f/PI*atan2(cvmGet(evects,0,0),cvmGet(evects,0,1));
	}
	else	alfa = 180.0f/PI*atan2(cvmGet(evects,0,1),cvmGet(evects,0,0));

	//printf("a= %f, b=%f\n",a,b);

	int ia = (int)floor(a/resolution+0.5);
	int ib = (int)floor(b/resolution+0.5);
	
	if (ia >= 0 && ia < im.width && ib >= 0 && ib < im.width){
		cvEllipse(&im, p, cvSize(ia,ib), alfa, 0, 360, redcolor, CV_FILLED );
	}

//	cvLine(&im, p, p2,  cvScalar(255,0,0), 1);


	cvReleaseMat(&cov);
	cvReleaseMat(&evects);
	cvReleaseMat(&evals);

}
