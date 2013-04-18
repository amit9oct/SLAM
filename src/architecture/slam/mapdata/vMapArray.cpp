/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class visualMap
*
* Implements a visual landmark 3d map
*
*/

#include "vMapArray.h"
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "matFuns.h"

namespace{
	visualMap* CreateVMapArray(int n, const ConfigFile& configFile){
		return new vMapArray(n,configFile);
	};
	const int VMAPARRAY = 0;
	const bool registered = visualMapFactory::Instance().Register(VMAPARRAY, visualMapCreator(CreateVMapArray));
}

// Constructor
vMapArray::vMapArray():
	visualMap(),
	nMarksReserved(0),
	lastRetId(-1),
	landmarks(0)
{
	
}

// Constructor
vMapArray::vMapArray(int n, const ConfigFile& configFile):
	visualMap(n,configFile),
	nMarksReserved(n),
	lastRetId(-1),
	landmarks(new landmark[n]())
{
	
}

// copy constructor
vMapArray::vMapArray(const vMapArray &map):
	visualMap(map),
	nMarksReserved(map.nMarksReserved),
	lastRetId(map.lastRetId),
	landmarks(new landmark[map.nMarksReserved]())
{
	for (int i = 0; i < nlandmarks; i++)
		landmarks[i] = map.landmarks[i];
}

// destructor
vMapArray::~vMapArray(){
	if (landmarks)	delete[] landmarks;
}

// initializer
void vMapArray::initialize(int n, const ConfigFile& configFile){
	visualMap::initialize(n,configFile);
	if (landmarks)	delete[] landmarks;
	landmarks = new landmark[n]();
	nMarksReserved = n;
	lastRetId = -1;
}

// assignment operator
vMapArray& vMapArray::operator=(const visualMap& vmap){
    assert(typeid(vmap) == typeid(*this));
    const vMapArray& map = dynamic_cast<const vMapArray&>(vmap);
    visualMap::operator=(vmap);

	if (nMarksReserved != map.nMarksReserved){
		nMarksReserved = map.nMarksReserved;
		if (landmarks) delete[] landmarks;
		landmarks = new landmark[nMarksReserved]();
	}
	for (int i = 0; i < nlandmarks; i++)
		landmarks[i] = map.landmarks[i];

	return *this;
}

// Data functions
void vMapArray::addLandmark(const landmark& mark){
	if (nlandmarks < nMarksReserved){
		landmarks[nlandmarks] = mark;
		nlandmarks++;
	}
}

void vMapArray::changeLastReturnedLandmark(const landmark& mark){
	if (lastRetId>=0) landmarks[lastRetId] = mark;

}

void vMapArray::changeLandmark(int id, const landmark& mark){
	if (id>=0) landmarks[id] = mark;
}

// get landmark by index
landmark* vMapArray::getLandmarkById(int n) {
	if (n<nlandmarks){
		lastRetId = n;
		return new landmark (landmarks[n]);
	}
	else{
		lastRetId = -1;
		return 0;
	}
}

// get landmark by descriptor 
landmark* vMapArray::getLandmarkByDesc(const float* desc) {
	const landmark* landres=0; 
	lastRetId = -1;
	for (int i = 0; i< nlandmarks; i++){
		if (descDist(landmarks[i].descriptor,desc,landmarks[i].desclength)<descTh){
			landres = &landmarks[i];
			lastRetId = i;
		}
	}
	return (landres)? new landmark(*landres) : 0;
}

// asociacion de datos
landmark* vMapArray::dataAssociation(const matrix& ZT, const matrix& Rt, const pos3d& globalpos, const pose& robotPos, const matrix& H, const matrix& Htrans, const float* desc) {
	float minDist=1000.0f, dist;
	lastRetId = -1;
	landmark* res = 0;
	for (int i = 0; i< nlandmarks; i++){
//		if (sqrt( pow(landmarks[i].pos.getX()-globalpos.getX(),2)+ pow(landmarks[i].pos.getY()-globalpos.getY(),2))<range){ 

			matrix zgorro	= base2cam(global2base(landmarks[i].pos, robotPos));
			matrix In		= ZT - zgorro;
			matrix Z		= (H*(landmarks[i].covariance*(Htrans))) + Rt;
			
			float pmatch =  exp(-0.5*mahalanobis(In.toPos3d(), Z));// /(sqrt((Z*PIx2).det()));
			
			if ( pmatch > mahTh){							// check mahalanobis
				dist = visualMap::descDist(landmarks[i].descriptor, desc, landmarks[i].desclength);
				if (dist < minDist && dist < descTh){		// get the minor descriptor distance but bigger than descTh
					lastRetId = i;
					res = &landmarks[i];
					minDist = dist;
				}
			}
//		}
	}
	return (res)? new landmark(*res):0;
}

void vMapArray::clear(){
	nlandmarks=0;
}

void vMapArray::saveMap(const char* str)  {
	char mystr[50];
	sprintf(mystr,"%s.m",str);
	FILE * outfile = fopen( mystr, "w" );
	fprintf(outfile,"landmarks_map=[",str);
	for(int i = 0; i < nlandmarks; i++)
		fprintf(outfile, "%e,%e,%e,%e,%e,%e,%e,%e,%e,%e,%e,%e,%f\n",landmarks[i].pos.getX(), landmarks[i].pos.getY(), landmarks[i].pos.getZ(),
		landmarks[i].covariance(0,0), landmarks[i].covariance(0,1), landmarks[i].covariance(0,2),
		landmarks[i].covariance(1,0), landmarks[i].covariance(1,1), landmarks[i].covariance(1,2),
		landmarks[i].covariance(2,0), landmarks[i].covariance(2,1), landmarks[i].covariance(2,2),
		landmarks[i].descriptor[0]);
	fprintf(outfile,"];");
	fclose(outfile);
}

void vMapArray::drawMarks (IplImage& im, float xorigin, float yorigin, float resolution) {

	//printf("nland: %d\n",getNLandmarks());
	CvScalar greencolor = CV_RGB(0,255,0);
	for (int l=0; l<nlandmarks; l++){
		CvPoint p = cvPoint ((int) floor ( (landmarks[l].pos.getX() - xorigin)/resolution),				// posicion
							 (int) (im.height-1-floor((landmarks[l].pos.getY() - yorigin)/resolution)));
/*
		CvMat* cov = cvCreateMat(3,3,CV_32FC1);
		CvMat* evects = cvCreateMat(3,3,CV_32FC1);
		CvMat* evals = cvCreateMat(1,3,CV_32FC1);
		cvmSet(cov,0,0, landmarks[l].covariance(0,0));
		cvmSet(cov,0,1, landmarks[l].covariance(0,1));
		cvmSet(cov,0,2, landmarks[l].covariance(0,2));
		cvmSet(cov,1,0, landmarks[l].covariance(1,0));
		cvmSet(cov,1,1, landmarks[l].covariance(1,1));
		cvmSet(cov,1,2, landmarks[l].covariance(1,2));
		cvmSet(cov,2,0, landmarks[l].covariance(2,0));
		cvmSet(cov,2,1, landmarks[l].covariance(2,1));
		cvmSet(cov,2,2, landmarks[l].covariance(2,2));
		cvEigenVV( cov, evects, evals, DBL_EPSILON);
*/
		CvMat* cov = cvCreateMat(2,2,CV_32FC1);
		CvMat* evects = cvCreateMat(2,2,CV_32FC1);
		CvMat* evals = cvCreateMat(2,1,CV_32FC1);
		cvmSet(cov,0,0, landmarks[l].covariance(0,0));
		cvmSet(cov,0,1, landmarks[l].covariance(0,1));
		cvmSet(cov,1,0, landmarks[l].covariance(1,0));
		cvmSet(cov,1,1, landmarks[l].covariance(1,1));
		cvEigenVV( cov, evects, evals, DBL_EPSILON);

		float a = 3*sqrt(cvmGet(evals,0,0));
		float b = 3*sqrt(cvmGet(evals,1,0));
		float alfa;
		if (a != a) a = 0.0f;
		if (b != b) b = 0.0f;

		if (a>b){
			float tmp = a;
			a = b;
			b = tmp;
			alfa = 180.0f/PI*atan2(cvmGet(evects,0,0),cvmGet(evects,0,1));
		}
		else	
			alfa = 180.0f/PI*atan2(cvmGet(evects,0,1),cvmGet(evects,0,0));

		cvReleaseMat(&cov);
		cvReleaseMat(&evects);
		cvReleaseMat(&evals);

		int ia = (int)(a/resolution);
		int ib = (int)(b/resolution);
		
		if (ia >= 0 && ia < im.width && ib >= 0 && ib < im.width){
			cvEllipse(&im, p, cvSize(ia,ib), alfa, 0, 360, greencolor, CV_FILLED );
		}
	}
}
