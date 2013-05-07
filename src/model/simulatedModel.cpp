#include "simulatedModel.h"
#include "ConfigFile.h"
#include "StringTokenizer.h"
#include <iostream>
#include "matFuns.h"
#ifndef WIN32
#include <sys/time.h>
#include <pthread.h>
#endif
#include <QDir>

#ifdef WIN32
#include <direct.h>	// getcwd() gets current working dir
#endif

#define MAXACQUIREDLANDMARKS 60

//TODO: Hacer el numero de robots dinámico a medida que llegan getids()

simulatedModel::simulatedModel():
	nBots(0),
	maxNumRobots(0),
	step(0),
	rsData(0),
	lmData(0),
	odo(0),
	gt(0),
	lastodo(0),
	lastgt(0),
	rbase(0),
	robotsEnabled(0),
	velo(0),
	logstr(0),
	alfa1(0.0f),					
	alfa2(0.0f),					
	alfa3(0.0f),
	alfa4(0.0f),	
	drifttrans(1.0f),
	imgwidth(0),
	imgheight(0),
	f(0.0f), 
	I(0.0f), 
	sigma2r(0.0f),
	sigma2c(0.0f),
	sigma2cp(0.0f),
	sigmar(0.0f),
	sigmac(0.0f),
	sigmacp(0.0f),
	betaMAX(0.0f),
	gammaMAX(0.0f),
	distMAX(0.0f),
	distMIN(0.0f),
	camx(0.0),
	camy(0.0),
	camz(0.0),
	rx(0.0),
	ry(0.0),
	rz(0.0),
	sample_time(0.0f),
	numSensors(0),		
	devError(0.0f),
	realError(0.0f),
	gammamax(0.0f),
	aperture(0.0f),
	maxDist(0.0f),
	minDist(0.0f),
	//robots_lines(0),
	endth(true),
	idCounter(0),
	Scam(9,9),
	width(0),
	height(0),
	showsimulation(false),
//	groupInitialPoses	(false),
//	maxInitialPoseDist	(0),
	numfeat			(0),
	mapLoaded(false),
	omap(0),
	omapfilename(new char[50])
{
	printf("[SIM] Default simulated model created\n");
}

simulatedModel::simulatedModel(int b, ConfigFile& config, float sampletime):
	nBots			(b),	
	maxNumRobots		(config.read<int>("MAXNUMROBOTS")),
	step			(0),
	rsData			(new rangeSensorData[maxNumRobots]),
	lmData			(new landmarksData[maxNumRobots]),
	odo			(new pose[maxNumRobots]),
	gt			(new pose[maxNumRobots]),
	lastodo			(new pose[maxNumRobots]),
	lastgt			(new pose[maxNumRobots]),
	rbase			(new robotBase*[maxNumRobots]),
	robotsEnabled		(new bool[maxNumRobots]),
	velo			(new speed[maxNumRobots]),
	logstr			(0),
	alfa1			(config.read<float>("alfa1")),
	alfa2			(config.read<float>("alfa2")),
	alfa3			(config.read<float>("alfa3")),
	alfa4			(config.read<float>("alfa4")),
	drifttrans		(1.0),

	imgwidth		(config.read<int>("WIDTH")),
	imgheight		(config.read<int>("HEIGHT")),

	sigma2f			(0.0), 
	sigmaf			(0.0),
	f			(config.read<float>("f")), 
	Realf			(normrnd(f,sigmaf)), 

	sigma2I			(0.0), 
	sigmaI			(0.0),
	I			(config.read<float>("I")), 
	RealI			(normrnd(I,sigmaI)), 
	
	sigma2Cx		(0.0), 
	sigmaCx			(0.0),
	Cx			(imgwidth/2.0), 
	RealCx			(normrnd(Cx,sigmaCx)), 
	
	sigma2Cy		(0.0), 
	sigmaCy			(0.0),
	Cy			(imgheight/2.0), 
	RealCy			(normrnd(Cy,sigmaCy)), 
	
	sigma2Cxp		(0.0), 
	sigmaCxp		(0.0),
	Cxp			(imgwidth/2.0), 
	RealCxp			(normrnd(Cxp,sigmaCxp)), 

	sigmar			(config.read<float>("sigmar")),
	sigmac			(config.read<float>("sigmac")),	
	sigmacp			(0.0),
	sigmad			(config.read<float>("sigmad")), 
	sigma2r			(pow(sigmar,2)),
	sigma2c			(pow(sigmac,2)),
	sigma2cp		(0.0),
	sigma2d			(pow(sigmad,2)),
	
	betaMAX			(atan2(Cy,f)),
	gammaMAX		(atan2(Cx,f)),

	distMAX			(config.read<float>("distMAX")),
	distMIN			(config.read<float>("distMIN")),
	camx			(config.read<float>("camx")),
	camy			(config.read<float>("camy")),
	camz			(config.read<float>("camz")),
	rx			(0.0),
	ry			(0.0),
	rz			(0.0),

	numSensors		(config.read<int>("NUMPOINTS")),		
	devError		(config.read<float>("LASERSIGMA")),
	realError		(devError),
	gammamax		(config.read<float>("LASERMAXANGLE")*PI/180.0),
	aperture		(0.0),
	maxDist			(config.read<float>("LASERMAXDIST")),
	minDist			(config.read<float>("LASERMINDIST")),
	sample_time		(sampletime),
	//robots_lines		(new std::vector<line>[nBots]),
	endth			(true),
	idCounter		(0),
	Scam			(9,9),
//	width			(config.read<int>("SIMULATORWIDTH")),
//	height			(config.read<int>("SIMULATORHEIGHT")),
	width			(800),
	height			(600),
//	showsimulation		(config.read<bool>("SHOWSIMULATION")),
	showsimulation		(false),
	//groupInitialPoses	(false),
	//maxInitialPoseDist	(0),
	numfeat			(0),
	mapLoaded		(false),
	omap			(0),
	omapfilename		(new char[50])
{
	robots_lines.reserve(maxNumRobots);

	string robotpos = config.read<string>("FOOTPRINT");
	StringTokenizer st(&robotpos[0]);
	robot_footprint.clear();
	for (int t = 0; t < st.countTokens(); t++){
		float pt = atof(st.nextToken());
		robot_footprint.push_back(pt);
	}

	// Matriz de covarianza del sensor
	Scam.set(0,0,sigma2c);	// S = [sigma2c       0        0		   0		   0			0	     0			  0 ]
	Scam.set(1,1,sigma2r);	//     [      0 sigma2r        0		   0		   0			0	     0			  0 ]
	Scam.set(2,2,sigma2cp);	//     [      0       0 sigma2cp		   0		   0			0	     0			  0 ]
	Scam.set(3,3,sigma2Cx);	//     [      0       0		   0	sigma2Cx 		   0			0	     0			  0 ]
	Scam.set(4,4,sigma2Cy);	//     [      0       0		   0	       0	sigma2Cy			0	     0			  0 ]
	Scam.set(5,5,sigma2Cxp);//     [      0       0		   0           0	       0	sigma2Cxp	     0			  0 ]
	Scam.set(6,6,sigma2f);	//     [      0       0        0	       0	       0	        0	sigma2f			  0 ]
	Scam.set(7,7,sigma2I);	//     [      0       0        0	       0	       0	        0	      0		sigma2I ]
	Scam.set(8,8,sigma2d);

	robots_lines.clear();

	for (int r=0; r<maxNumRobots; r++){
		std::vector<line> v(robot_footprint.size());
		robots_lines.push_back(v);
	}

	reset();

	loadMapFile(config);

	printf("[SIM] simulation model created\n");
}

void simulatedModel::reset(){

	idCounter=0;
	
	for (int r=0; r<maxNumRobots; r++){
		rsData[r].initialize(numSensors, devError, gammamax, aperture, maxDist, minDist);
		rbase[r] = 0;
		lmData[r].reserve(MAXACQUIREDLANDMARKS);
		robotsEnabled[r]=true;
	}
}

void simulatedModel::setNumRobots(int n){
	nBots = n;
	reset();
}

simulatedModel::~simulatedModel(){
	stop();
	if (rsData) delete[] rsData;
	if (lmData) delete[] lmData;
	if (odo) delete[] odo;
	if (gt) delete[] gt;
	if (lastodo) delete[] lastodo;
	if (lastgt) delete[] lastgt;
	if (velo) delete[] velo;
	if (logstr) delete[] logstr;
	if (rbase) delete[] rbase;
	if (robotsEnabled) delete[] robotsEnabled;
	if (omap) delete omap;	
	if (omapfilename) delete[] omapfilename;

	printf("[SIM] Simulation ended\n");
}

// initializer
void simulatedModel::initialize(int b, ConfigFile& config, float sampletime){

	printf("initializing simulation model...\n");
	nBots 			= b;
	maxNumRobots		= config.read<int>("MAXNUMROBOTS");
	step 			= 0;
	if (rsData) 		delete[] rsData;
	rsData 			= new rangeSensorData[maxNumRobots];
	if (lmData) 		delete[] lmData;
	lmData 			= new landmarksData[maxNumRobots];
	if (odo)		delete[] odo;
	odo 			= new pose[maxNumRobots];
	if (gt) 		delete[] gt;
	gt 			= new pose[maxNumRobots];
	if (lastodo) 		delete[] lastodo;
	lastodo 		= new pose[maxNumRobots];
	if (lastgt) 		delete[] lastgt;
	lastgt 			= new pose[maxNumRobots];
	if (velo) 		delete[] velo;
	velo 			= new speed[maxNumRobots];
	if (logstr) 		delete[] logstr;
	logstr 			= 0;
	if (rbase) 		delete[] rbase;
	rbase 			= new robotBase*[maxNumRobots];
	if (robotsEnabled) 	delete[] robotsEnabled;
	robotsEnabled 		= new bool[maxNumRobots];
	if (omap) 		delete omap;	
	omap 			= 0;
	if (omapfilename)	delete[] omapfilename;
	omapfilename 		= new char[50];

	alfa1 			= config.read<float>( "alfa1" );
	alfa2 			= config.read<float>( "alfa2" );
	alfa3 			= config.read<float>( "alfa3" );
	alfa4 			= config.read<float>( "alfa4" );
	drifttrans 		= 1.0;

	imgwidth		= config.read<int>("WIDTH");
	imgheight		= config.read<int>("HEIGHT");

	sigma2f			= 0.0;
	sigmaf			= 0.0;
	f			= config.read<float>("f");
	Realf			= normrnd(f,sigmaf);

	sigma2I			= 0.0;
	sigmaI			= 0.0;
	I			= config.read<float>("I");
	RealI			= normrnd(I,sigmaI);
	
	sigma2Cx		= 0.0;
	sigmaCx			= 0.0;
	Cx			= imgwidth/2.0;
	RealCx			= normrnd(Cx,sigmaCx);
	
	sigma2Cy		= 0.0;
	sigmaCy			= 0.0;
	Cy			= imgheight/2.0;
	RealCy			= normrnd(Cy,sigmaCy);
	
	sigma2Cxp		= 0.0;
	sigmaCxp		= 0.0;
	Cxp			= imgwidth/2.0;
	RealCxp			= normrnd(Cxp,sigmaCxp);

	sigmar			= config.read<float>("sigmar");
	sigmac			= config.read<float>("sigmac");	
	sigmacp			= 0.0;
	sigmad			= config.read<float>("sigmad");
	sigma2r			= pow(sigma2r,2);
	sigma2c			= pow(sigma2c,2);
	sigma2cp		= pow(sigma2cp,2);
	sigma2d			= pow(sigma2d,2);

	betaMAX			= atan2(Cy,f);
	gammaMAX		= atan2(Cx,f);
	distMAX 		= config.read<float>( "distMAX" );
	distMIN 		= config.read<float>( "distMIN" );

	camx 			= config.read<float>( "camx" );
	camy 			= config.read<float>( "camy" );
	camz 			= config.read<float>( "camz" );
	rx	 		= 0.0;
	ry	 		= 0.0;
	rz	 		= 0.0;

	// sonar
	numSensors		= config.read<int>("NUMPOINTS");	
	devError		= config.read<float>("LASERSIGMA");
	realError		= devError;
	gammamax		= config.read<float>("LASERMAXANGLE")*PI/180.0;
	aperture		= 0.0;
	maxDist			= config.read<float>("LASERMAXDIST");
	minDist			= config.read<float>("LASERMINDIST");

	sample_time		= sampletime;

	//robots_lines = new std::vector<line>[nBots];
	robots_lines.clear();
	robots_lines.reserve(maxNumRobots);

	endth =true;
	idCounter=0;

//	width=config.read<int>("SIMULATORWIDTH");
//	height=config.read<int>("SIMULATORHEIGHT");
//	showsimulation		= config.read<bool>("SHOWSIMULATION");

	width=800;
	height=600;
	showsimulation = false;	

	//groupInitialPoses=false;
	//maxInitialPoseDist=0;
	numfeat=0;
	mapLoaded=false;

	//---------

	string robotpos = config.read<string>("FOOTPRINT");
	StringTokenizer st(&robotpos[0]);
	robot_footprint.clear();
	for (int t = 0; t < st.countTokens(); t++){
		float pt = atof(st.nextToken());
		robot_footprint.push_back(pt);
	}

	for (int r=0; r<maxNumRobots; r++){
		std::vector<line> v(robot_footprint.size());
		robots_lines.push_back(v);
	}

	reset();

	// Matriz de covarianza del sensor
	Scam.clear();
	Scam.set(0,0,sigma2c);	// S = [sigma2c       0        0		   0		   0			0	     0			  0 ]
	Scam.set(1,1,sigma2r);	//     [      0 sigma2r        0		   0		   0			0	     0			  0 ]
	Scam.set(2,2,sigma2cp);	//     [      0       0 sigma2cp		   0		   0			0	     0			  0 ]
	Scam.set(3,3,sigma2Cx);	//     [      0       0		   0	sigma2Cx 		   0			0	     0			  0 ]
	Scam.set(4,4,sigma2Cy);	//     [      0       0		   0	       0	sigma2Cy			0	     0			  0 ]
	Scam.set(5,5,sigma2Cxp);//     [      0       0		   0           0	       0	sigma2Cxp	     0			  0 ]
	Scam.set(6,6,sigma2f);	//     [      0       0        0	       0	       0	        0	sigma2f			  0 ]
	Scam.set(7,7,sigma2I);	//     [      0       0        0	       0	       0	        0	      0		sigma2I ]
	Scam.set(8,8,sigma2d);

	loadMapFile(config);

	printf("simulated model initialized\n");

}

void simulatedModel::setLogName(const char* str){
	if (logstr)	delete[] logstr;
	logstr = new char[strlen(str)+1];
	strcpy(logstr,str);
}

void simulatedModel::update_footprint(int r){
	robots_lines[r].clear();
	int total_lines2 = robot_footprint.size();	
	for (int l = 0; l < robot_footprint.size(); l+=2){
		float x1 = robot_footprint[l];
		float y1 = robot_footprint[l+1];
		float x2 = robot_footprint[(l+2)%total_lines2];
		float y2 = robot_footprint[(l+3)%total_lines2];
		line lin(	gt[r].x+x1*cos(gt[r].th)-y1*sin(gt[r].th),
				gt[r].y+x1*sin(gt[r].th)+y1*cos(gt[r].th),
				gt[r].x+x2*cos(gt[r].th)-y2*sin(gt[r].th),
				gt[r].y+x2*sin(gt[r].th)+y2*cos(gt[r].th));
		robots_lines[r].push_back(lin);
	}
}

int simulatedModel::setup(){
	prio = 50;
	return 0;
}

void simulatedModel::onStop(){
	//printf("[SIM] Simulator on stop...\n");
	if(!endth){
		endth = true;
		closing.lock();
		closing.unlock();
	}
}

void simulatedModel::execute(){
	printf("[SIM] \t\t\t\t\t\t\t Simulation Running\n");
	closing.lock();
	endth = false;
	
	speed speedconst;
	speedconst.v=0.5f;
	speedconst.w=0.0f;

	for (int r = 0; r< nBots; r++){
		rbase[r]->beginProduction();
		odo[r] = gt[r];
//		printf("[SIM] generating initial state for robot %d...\n",r);
		simRangeAdquisition(gt[r], rsData[r],r);						// leemos sonar
		lmData[r].clear();
		simCamAdquisition(gt[r], lmData[r],r);							// leemos landmarks
		lastgt[r] = gt[r];
		lastodo[r] = odo[r];
//		printf("[SIM] produced initial state for robot %d\n",r);
		rbase[r]->endProduction();
	}

//	printf("Parameters>>>>>>\n");
//	printf("f = %f (%f)\n",Realf, f);
//	printf("I = %f (%f)\n",RealI, I);
//	printf("Cx = %f (%f)\n",RealCx, Cx);
//	printf("Cy = %f (%f)\n",RealCy, Cy);
//	printf("Cxp = %f (%f)\n",RealCxp, Cxp);
//	printf(">>>>>>>>>>>>>>>>>>\n");

	// open the log file
	FILE* vworldlog=0;
	if (logstr){
		char myfilestr[100];
		sprintf(myfilestr,"%s.m",logstr);	
		vworldlog = fopen(myfilestr,"w");
		fprintf(vworldlog,"ground_truth = [",logstr);
	}

	char* wname= "Simulator";
	IplImage* background = 0;
	if (showsimulation){
		background = getMapImage();
		cvNamedWindow(wname,0);
	}
	int capNum=0;
	char filestr[100];

	elapsedTime = 0.0f;
	int step=1;
	do{
		//printf("[SIM] sleeping\n");		
		sleepms(100);
		for (int r = 0; r< nBots; r++){	
			if (robotsEnabled[r]){
				//printf("[SIM] waiting to produce step %d for robot %d...\n", step,r);
				rbase[r]->beginProduction();
				//printf("[SIM] generating step %d for robot %d...\n", step,r);
				if (robotsEnabled[r]){
					if(step>2) robotMotion(gt[r], velo[r], sample_time);						// move robots
					update_footprint(r);
					//gt[r].print("\t\t\t\t\t\t\t\t\tground truth=");
					//robotMotion(gt[r], speedconst, sample_time);						// move robots
					motionSample(lastgt[r], gt[r], lastodo[r], odo[r]);					// leemos odometria
					//odo[r].print("\t\t\t\t\t\t\t\t\todo=");
					simRangeAdquisition(gt[r], rsData[r],r);						// leemos sonar
					lmData[r].clear();
					simCamAdquisition(gt[r], lmData[r],r);							// leemos landmarks
					lastgt[r] = gt[r];
					lastodo[r] = odo[r];
				}
				//printf("[SIM] produced data for step %d and robot %d\n",step,r);
			}
			rbase[r]->endProduction();
		}
		step++;
		// write logs
		if (logstr){
			fprintf(vworldlog,"%f",elapsedTime);
			for (int r=0; r<nBots;r++){
				fprintf(vworldlog,", %f, %f, %f",gt[r].x, gt[r].y, gt[r].th);
			}
			fprintf(vworldlog,"\n");
		}
		elapsedTime+=sample_time;
		redrawing();
//		if (showsimulation){
//			IplImage* img = cvCloneImage(background);
//			drawRobots(img);
//			cvShowImage(wname,img);
//			int key = cvWaitKey(2);
//			if (key=='c'){
//				sprintf(filestr,"simcap%d.png",capNum);
//				cvSaveImage(filestr,img);				
//				capNum++;
//			}
//			cvReleaseImage(&img);
//		}
	} while (!endth);
	
	if (showsimulation){
		cvDestroyWindow(wname);
		cvReleaseImage(&background);
	}

	if (logstr){
		fprintf(vworldlog,"];");
		fclose(vworldlog);
		// save img
	}

	//printf("[SIM] closing unlock\n");
	closing.unlock();
}

// TODO: Include error checking, it is likely to crash with an erroneous map file
void simulatedModel::loadMapFile(const ConfigFile& config){
	printf("[SIM] loading map...\n");
	//ConfigFile config(mapfile);

	// initial poses
	char entrada[20];
	
	// walls / obstacles
	int numwalls = config.read<int>("NUMWALLS");
	mapWall.clear();
	mapWall.reserve(numwalls);
	for (int i = 0; i< numwalls; i++){
		sprintf(entrada,"WALL%i",i+1);
		string wallstr = config.read<string>(entrada);
		StringTokenizer st(&(wallstr[0]));
		float x1 = atof(st.nextToken());
		float y1 = atof(st.nextToken());
		float x2 = atof(st.nextToken());
		float y2 = atof(st.nextToken());
		line wall( x1, y1, x2, y2 );
		mapWall.push_back(wall);
	}
	
	// features
	numfeat = config.read<int>("NUMFEATURES");
	desclength = config.read<int>("DESCRIPTORLENGTH");
	float* desc = new float[desclength];
	landmarks.clear();
	landmarks.reserve(numfeat);
	for (int i = 0; i< numfeat; i++){
		sprintf(entrada,"FEAT%i",i+1);
		string featstr = config.read<string>(entrada);
		StringTokenizer st(&(featstr[0]));
		float x = atof(st.nextToken());
		float y = atof(st.nextToken());
		float z = atof(st.nextToken());
		for (int j=0; j < desclength; j++){
//			desc[j] = atof(st.nextToken());
			desc[j] = i;
		}
		landmarks.push_back(feature(pos3d(x,y,z),desc,desclength));
	}
	delete[] desc;
	
	//initial poses
	for (int r = 0; r< maxNumRobots; r++){
		sprintf(entrada,"R%i",r+1);
		string robotpos = config.read<string>(entrada);
		StringTokenizer st(&robotpos[0]);

		float x = atof(st.nextToken());
		float y = atof(st.nextToken());
		float th = atof(st.nextToken());
		pose pos(x,y,th);
		odo[r]	   = pos;
		lastodo[r] = pos;
		gt[r]      = pos;
		lastgt[r]  = pos;
		update_footprint(r);
	}

//	if (config.read<bool>("RANDOM")){
		if (omap) delete omap;
		omap = gridMapFactory::Instance().CreateObject(config.read<int>("GRIDMAPTYPE"),0.0,0.0,0.0,0.0,0.0);
		strcpy(omapfilename, ((QDir::homePath()+="/.mrxt/maps/")+=config.read<string>("GRIDMAP").c_str()).toStdString().c_str());
		omap->loadMapFromFile(omapfilename);	
//	}

//	cvNamedWindow("OMAP");
//	omap->showMap("OMAP");
//	cvWaitKey(5000);

//	groupInitialPoses = true; //config.read<bool>("GROUPED");
//	maxInitialPoseDist = 6.0/omap->getResolution(); // config.read<float>("MAXGROUPDIST")/omap->getResolution();
//	minInitialPoseDist = 0.85; //config.read<float>("MINGROUPDIST");

	printf("[SIM] map loaded\n");

	mapLoaded = true;
}

void simulatedModel::randomPoses(bool groupInitialPoses, double max, double min){
	binMap visibleArea;

	double maxInitialPoseDist = max/omap->getResolution(); // config.read<float>("MAXGROUPDIST")/omap->getResolution();
	double minInitialPoseDist = min; //config.read<float>("MINGROUPDIST");

//	if (config.read<bool>("RANDOM")){
	if(mapLoaded){
	//	printf("aqui\n");		
		binMap accessible;
	//	printf("aqui\n");		
		point p = omap->toCell(odo[0].x, odo[0].y);
	//	printf("aqui\n");		
		omap->countAccessible(&p, 1, accessible);
	//	printf("pose robot 1: (%f, %f), in cells (%d, %d)\n", odo[0].x, odo[0].y, p.x, p.y);
		point basePoint;
		std::vector<point> positives;
		bool vis;
		int maxIter = 10000;
		int countIter=0;
		bool error=false;

		do{
		for (int r = 0; r< maxNumRobots; r++){
			int x,y;
			bool testRange=true;
			
			countIter=0;
			error = false;
			pointf pos;
			do{
				countIter++;
				if (countIter>maxIter){
					//printf("Solution not found, trying again\n");
					error=true;
					break;
				}						
				vis=false;
				if (!groupInitialPoses){
					x = rand() % omap->getWidth();
					y = rand() % omap->getHeight();
					vis = true;
				}
				else if (r==0){
					x = rand() % omap->getWidth();
					y = rand() % omap->getHeight();
					basePoint.x = x;
					basePoint.y = y;
					visibleArea.clear();
					positives.clear();
					omap->esz(x,y,visibleArea, 0,maxInitialPoseDist);
					if (visibleArea.getPositives(positives)<30) vis = false;
					else vis = true;
				}
				else{
					int idx = rand() % positives.size();
					x = positives[idx].x;
					y = positives[idx].y;
					vis = visibleArea.get(x,y);
				}
				
				pos = omap->toCoords(x,y);
				testRange=true;	
				for(int rr = 0; rr < r; rr++){
					if( sqrt( pow(pos.x-gt[rr].x ,2) + pow(pos.y-gt[rr].y,2) ) < minInitialPoseDist ) testRange = false;
				}
			} while (!accessible.get(x,y) || !omap->isfree(x,y,4) || !vis || !testRange);		

			if (error) break;
			landmarksData lma;
			lma.reserve(numfeat);
			pose newpos(pos.x,pos.y,0);
			do{
				lma.clear();				
				newpos.th = (rand() % 628)/100.0f-3.14f;
				simCamAdquisition(newpos, lma, r);
			//	printf("th =%f, marks = %d\n",newpos.th, lma.getNLandmarks());
			} while(lma.getNLandmarks() < 4);
			
			//printf("[SIM] new robot position for robot %d: cells (%d,%d), coords (%f,%f)\n", r, x, y, pos.x, pos.y);
			odo[r] = newpos;
			lastodo[r] = newpos;
			gt[r] = newpos;
			lastgt[r] = newpos;
			update_footprint(r);
		}

		} while(error);
	}
}


void simulatedModel::redrawing(){
	//QPixmap* pixmap = getPixmap();
	//emit redraw(*pixmap);
	//delete pixmap;

	//emit ()

	//printf("emit poses\n");
	rposes poses;
	for(int r = 0; r < nBots; r++){
		poses.push_back(gt[r]);
	}
	emit changedPositions(poses);
}

QPixmap* simulatedModel::getPixmap(){
	IplImage* img = getMapImage();
	drawRobots(img);
	cvSaveImage("imgtmp.png",img);
	cvReleaseImage(&img);
	QPixmap* pix = new QPixmap("imgtmp.png");
	return pix;
}

rposes simulatedModel::getPoses() const{
	rposes poses;
	for(int r = 0; r < nBots; r++){
		poses.push_back(gt[r]);
	}
	return poses;	
}

void simulatedModel::drawRobots(IplImage*& img){
	CvScalar redcolor = CV_RGB(255,0,0);
	CvScalar bluecolor = CV_RGB(0,0,255);
	CvScalar blackcolor = CV_RGB(0,0,0);
	CvFont font; 
	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
	char numstr[3];

	std::vector<line>::iterator lineit;
	for (int r = 0; r < nBots; r++){
		for ( lineit = robots_lines[r].begin(); lineit < robots_lines[r].end(); lineit++ ){
			CvPoint pt1 = cvPoint((int)((lineit->x1-minx)*scalex), height-(int)((lineit->y1-miny)*scaley) );
			CvPoint pt2 = cvPoint((int)((lineit->x2-minx)*scalex), height-(int)((lineit->y2-miny)*scaley) );
			//printf("(%d, %d) - (%d, %d)\n", pt1.x, pt1.y, pt2.x, pt2.y);
			//if (rbase[r]){
			//	if (rbase[r]->isCaptured())
			//		cvLine(img, pt1, pt2, bluecolor);
			//	else
					cvLine(img, pt1, pt2, redcolor);
			//}
		}
		sprintf(numstr,"%d",r);
		CvPoint ptA = cvPoint((int)((gt[r].x-minx)*scalex), height-(int)((gt[r].y-miny)*scaley) );
		CvPoint ptB = cvPoint(ptA.x+10*cos(gt[r].th),ptA.y-10*sin(gt[r].th));
		CvPoint ptT = cvPoint(ptA.x+10,ptA.y+10);
		cvPutText(img, numstr, ptT, &font, blackcolor);
		//if (rbase[r]){
			//if (rbase[r]->isCaptured())
			//	cvLine(img,ptA,ptB,bluecolor);
			//else
				cvLine(img,ptA,ptB,redcolor);
		//}
	}
}

IplImage* simulatedModel::getMapImage() {

	CvScalar blackcolor = CV_RGB(0,0,0);
	CvScalar whitecolor = CV_RGB(255,255,255);

	IplImage* img = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3); 
	cvSet(img,whitecolor);

	std::vector<line>::iterator lineit;

	// buscamos maximo y minimo
//	printf("\t\t\t\tnum walls = %d\n",mapWall.size());
	maxx=FLT_MIN; maxy=FLT_MIN; minx=FLT_MAX; miny=FLT_MAX;
	for ( lineit = mapWall.begin(); lineit < mapWall.end(); lineit++ ){		
		if (maxx < lineit->x1) maxx= lineit->x1;
		if (maxx < lineit->x2) maxx= lineit->x2;
		if (maxy < lineit->y1) maxy= lineit->y1;
		if (maxy < lineit->y2) maxy= lineit->y2;
		if (minx > lineit->x1) minx= lineit->x1;
		if (minx > lineit->x2) minx= lineit->x2;
		if (miny > lineit->y1) miny= lineit->y1;
		if (miny > lineit->y2) miny= lineit->y2;
	}
	scalex = width/(maxx-minx);
	scaley = height/(maxy-miny);

//	printf("xrange: [%f-%f], yrange: [%f-%f]\n", minx, maxx, miny, maxy);
//	printf("scalex: %f, scaley: %f", scalex, scaley);

	for ( lineit = mapWall.begin(); lineit < mapWall.end(); lineit++ ){		
		CvPoint pt1 = cvPoint((int)((lineit->x1-minx)*scalex), height-(int)((lineit->y1-miny)*scaley) );
		CvPoint pt2 = cvPoint((int)((lineit->x2-minx)*scalex), height-(int)((lineit->y2-miny)*scaley) );
	//	printf("[SIM] (%d, %d) - (%d, %d)\n", pt1.x, pt1.y, pt2.x, pt2.y);
		cvLine(img, pt1, pt2, blackcolor);
	}

	return img;

}

// Hace avanzar al robot durante un tiempo sample_time a velocidad vel
void simulatedModel::robotMotion(pose &pos, const speed& vel, float st){
	float vt = st*vel.v;
	float wt = st*vel.w;
	float meanang = pos.th + wt/2.0f;
    	pos.x += vt*cos(meanang);
    	pos.y += vt*sin(meanang);
	pos.th+= wt;
	if (pos.th>M_PI)			pos.th -= PIx2;
        else if (pos.th <= -M_PI)		pos.th += PIx2;
}

// Modelo de odometria
void simulatedModel::motionSample(const pose& lastPose, const pose& newPose, const pose& lastodo, pose& newodo){
	
	float diffy = newPose.y-lastPose.y;
	float diffx = newPose.x-lastPose.x;
	float diffth = newPose.th-lastPose.th;
	if (diffth>M_PI)			diffth -= PIx2;
	else if (diffth <= -M_PI)		diffth += PIx2;
	if (!(diffx == 0 && diffy == 0 && diffth ==0)){ 
		float deltarot1;
		//if (!(fabs(diffx) < 0.000001 && fabs(diffy) < 0.000001))
			deltarot1  = atan2(diffy, diffx) - lastPose.th;				// cuanto ha rotado 
		//else
		//	deltarot1 = 0;			
		float deltatrans = drifttrans*sqrt((diffx*diffx + diffy*diffy));		// distancia avanzada
		float deltarot2  = diffth - deltarot1;						// incremento de orientacion aparte de la rotacion
	
	//	float dtheta = fabs(atan2(sin(diffth),cos(diffth)));

	//	float aux = (alfa1*dtheta + alfa2*deltatrans);
	//	float deltarot1g  = (float)(normrnd(deltarot1,  aux));
	//	float deltatransg = (float)(normrnd(deltatrans, alfa3*deltatrans + alfa4*dtheta));
	//	float deltarot2g  = (float)(normrnd(deltarot2,  aux));   

		if (deltarot1>M_PI)			deltarot1 -= PIx2;
		else if (deltarot1 <= -M_PI)		deltarot1 += PIx2;
		if (deltarot2>M_PI)			deltarot2 -= PIx2;
		else if (deltarot2 <= -M_PI)		deltarot2 += PIx2;
	
		float rot1n = alfa1*fabs(deltarot1) + alfa2*deltatrans;
		float trann = alfa3*deltatrans + alfa4*(fabs(deltarot1+deltarot2));
		float rot2n = alfa1*fabs(deltarot2) + alfa2*deltatrans;
		float deltarot1g  = (float)(normrnd(deltarot1,  rot1n));
		float deltatransg = (float)(normrnd(deltatrans, trann));
		float deltarot2g  = (float)(normrnd(deltarot2,  rot2n));
	
		float aux2 = lastodo.th+deltarot1g;
	
		newodo.x  = lastodo.x  + deltatransg * cos(aux2);
		newodo.y  = lastodo.y  + deltatransg * sin(aux2);
		newodo.th = lastodo.th + deltarot1g + deltarot2g;

		if (newodo.th>M_PI)			newodo.th -= PIx2;
		else if (newodo.th <= -M_PI)		newodo.th += PIx2;
	}
	else{
		newodo.x  = lastodo.x;
		newodo.y  = lastodo.y;
		newodo.th = lastodo.th;
	}

}

// TODO: incluir una probabilidad de que una landmark no se detecte aun estando en el rango de vision
void simulatedModel::simCamAdquisition(const pose& robotpos, landmarksData& lmData, int nr){

	std::vector<feature>::iterator markit;
	pose3d campos = camPose(robotpos);
	//printf("robot pose x=%f, y=%f, th=%f\n", robotpos.x, robotpos.y, robotpos.th);
	for ( markit = landmarks.begin(); markit != landmarks.end(); markit++ ){ //comprobar la distancia a todas las landmarks del mapa
		pos3d v = base2cam(global2base(markit->pos, robotpos));	// posicion relativa de la marca a la camara
		float r = sqrt(v.getX()*v.getX()+v.getY()*v.getY());	// distancia de la camara a la marca

		double gamma = atan2(v.getY(), v.getX());
		double beta = atan2(v.getZ(), v.getX());
		if (gamma >= PI) gamma-=PIx2; 
		if (gamma < -PI) gamma+=PIx2; 
		if (beta >= PI) beta-=PIx2; 
		if (beta < -PI) beta+=PIx2; 

		//Si esta dentro del FOV del robot comprobamos su visibilidad dentro del mapa
		if ((r < distMAX) && (r > distMIN) && (fabs(gamma) < gammaMAX) && (fabs(beta) < betaMAX)){
			
			bool visibility = testVisibility(campos.pos, markit->pos, nr);
			
			//printf("vis= %d || r = %f [%f - %f], gamma = %f (%f), beta = %f (%f)\n", visibility, r,distMAX,distMIN, gamma, gammaMAX, beta, betaMAX);
			
			// si cae dentro del radio de accion del robot y es visible
			// Si cae dentro del FOV entonces recogemos la medida, anyadimos en simulacion un error a la medida segun 
			// una ley de propagacion del error... ver lineas de investigacion
			if (visibility){
				
				// proyectamos el punto real en la camara
				float aux1 = Realf*RealI;
				double c = -Realf*v.getY() / v.getX() + RealCx;		
				double r = Realf*v.getZ() / v.getX() + RealCy;		
				//double cp = -Realf*(RealI+v.getY())/ v.getX() + RealCxp;	
				double d = aux1/v.getX() ;//(c - RealCx) - (cp - RealCxp);
				double d2 = d*d;
				
				if(c<0 || r<0 || c>2*Cx || r>2*Cy ) continue;
				//printf("sigmas: %f, %f, %f\n",sigmac, sigmar, sigmad);

				// Añadimos ruido a (Xc, Yc, Zc) (Ruido gaussiano en r,c,d y propagamos)
				double cprev = c;
				do{
					c = normrnd(cprev,sigmac);
				}while(fabs(c-cprev)>3*sigmac);

				double rprev = r;
				do{
					r = normrnd(rprev,sigmar);
				}while(fabs(r-rprev)>3*sigmar);

//				cp = normrnd(cp,sigmacp);
				
				//d = (c - Cx) - (cp - Cxp);
				double dprev = d;
				do{
					d = normrnd(dprev,sigmad);
				}while(fabs(d-dprev)>3*sigmad);
				d2 = d*d;
				double d2prev = dprev*dprev; 
				
//				v.print("VW MARK = ");

				// Obtenemos la pose a partir de las medidas simuladas
				aux1 = f*I;
				float aux3 = I/d;
				v.getX() = aux3*f;
				v.getY() = aux3*(Cx-c);
				v.getZ() = aux3*(r-Cy);				

				/*
				// Jacobiano
				matrix J(3,8);
				J.set(0,0, -aux1/d2);
				J.set(0,2, aux1/d2);
				J.set(0,3, aux1/d2);
				J.set(0,5, -aux1/d2);
				J.set(0,6, I/d);
				J.set(0,7, f/d);

				J.set(1,0, -I/d - I*(Cx-c)/d2);
				J.set(1,2, I*(Cx-c)/d2);
				J.set(1,3, I/d + I*(Cx-c)/d2);
				J.set(1,5, -I*(Cx-c)/d2);
				J.set(1,7, (Cx-c)/d);

				J.set(2,0, -I*(r-Cy)/d2);
				J.set(2,1, I/d);
				J.set(2,2, I*(r-Cy)/d2);
				J.set(2,3, I*(r-Cy)/d2);
				J.set(2,4, -I/d);
				J.set(2,5, -I*(r-Cy)/d2);
				J.set(2,7, (r-Cy)/d);
*/
//				printf("Cx=%f, Cy=%f, Cxp=%f, f=%f, I=%f\n",Cx,Cy,Cxp,f,I);
//				printf("c=%f, r=%f, cp=%f, d=%f d2=%f\n",c,r,cp,d,d2);

				matrix J(3,9);
				J.set(0,8, -aux1/d2);
				J.set(1,0, -I/d);
				J.set(1,8, I*(c-Cx)/d2);			
				J.set(2,1, I/d);
				J.set(2,8, -I*(r-Cy)/d2);

				// Covarianza de la medida
//				v.print("V=");
//				printf("c= %f, r= %f, d=%f\n",c,r,d);
//				printf("sigmac = %f, sigmar = %f, sigmad = %f\n", sigmac, sigmar, sigmad);
//				printf("sigma2c = %e, sigma2r = %e, sigma2d = %e\n", sigma2c, sigma2r, sigma2d);
//				J.print("J=");
//				Scam.print("Scam=");
				matrix cov = J*(Scam*J.transpose());
//				cov.print("cov=");
//				cov*=1.5;

				//printf("c=%f, r=%f, cp=%f, d=%f\n",c,r,cp,d);
				//v.print("VW MARK CON RUIDO = ");
//				cov.print("COV=");

				// añadimos una marca a lmdata
				landmark lmark(v,cov,markit->desc,desclength,0);
				//printf("voy a anyadir\n");
				lmData.addLandmark(lmark);
				//printf("anyadida\n");
				//if( lmData.getNLandmarks() > 10) break;
			} // visible
		} // in the FOV
	}// for each landmark
	//printf("[VIRTUALWORLD] observed features: %d\n", lmData.getNLandmarks());
}

// TODO: use the sensor pose information
void simulatedModel::simRangeAdquisition(const pose& realPose, rangeSensorData& sonarData, int r){
	pose dest;
	for (int i = 0; i < sonarData.getNumSensors(); i++ ) {
		float ang = realPose.th+ sonarData.getSensorPose(i).th + aperture/2;
		dest.x = realPose.x + sonarData.getMaxDist()*cos(ang);
		dest.y = realPose.y + sonarData.getMaxDist()*sin(ang);
       		float dist1 = visibilityDistance(realPose, dest,r);

		ang = realPose.th+ sonarData.getSensorPose(i).th - aperture/2;
		dest.x = realPose.x + sonarData.getMaxDist()*cos(ang);
		dest.y = realPose.y + sonarData.getMaxDist()*sin(ang);
        	float dist2 = visibilityDistance(realPose, dest,r);

		ang = realPose.th+ sonarData.getSensorPose(i).th;
		dest.x = realPose.x + sonarData.getMaxDist()*cos(ang);
		dest.y = realPose.y + sonarData.getMaxDist()*sin(ang);
        	float dist3 = visibilityDistance(realPose, dest,r);

		float dist = (dist1<dist2)? ((dist1<dist3)? dist1: dist3) : ((dist2<dist3)? dist2: dist3) ;
		sonarData.setSensorValue(i,normrnd(dist, pow(realError*dist/3.0f,2)));
	}
}

bool simulatedModel::testVisibility(const pos3d& pos, const pos3d& mark, int nr){

	int suma = 0;
	pointf crosspoint;
	line ray(pos.getX(),pos.getY(),mark.getX(),mark.getY());

	//Comprobamos las veces que corta con alguna de las rectas del mapa
	std::vector<line>::iterator lineit;
	for ( lineit = mapWall.begin(); lineit < mapWall.end(); lineit++ ){
		int inter = intersect(ray, *lineit, crosspoint);
		if (!(fabs(crosspoint.x-mark.getX())<0.05 && fabs(crosspoint.y -mark.getY())<0.05 )) suma+=inter;
	}
//	for (int r = 0; r < nBots; r++){
//		if (r == nr) continue;
//		for ( lineit = robots_lines[r].begin(); lineit < robots_lines[r].end(); lineit++ ){
//			int inter = intersect(ray, *lineit, crosspoint);
//			if (!(fabs(crosspoint.x-mark.getX())<0.05 && fabs(crosspoint.y -mark.getY())<0.05 )) suma+=inter;
//		}
//	}
	// si unicamente hemos detectado un corte, entonces es visible. Si corta 2 o
	// mas veces, entonces el punto atraviesa alguna pared
	// se supone q las marcas estan sobre las paredes por lo tanto hay un corte
	if (suma < 1 )
	    return true;
	else
	    return false;
}

float simulatedModel::visibilityDistance(const pose& ori, const pose& dest, int nr){
	pointf crosspoint;



	line ray(ori.x,ori.y,dest.x,dest.y);
	float diffx = ori.x-dest.x;
	float diffy = ori.y-dest.y;
	float dist = diffx*diffx+diffy*diffy;

	//Comprobamos las veces que corta con alguna de las rectas del mapa
	std::vector<line>::iterator lineit;

	for ( lineit = mapWall.begin(); lineit < mapWall.end(); lineit++ ){
		if (intersect(ray, *lineit, crosspoint)){
			diffx = ori.x-crosspoint.x;
			diffy = ori.y-crosspoint.y;
			float aux = diffx*diffx+diffy*diffy;
			if (aux < dist)	dist = aux;
		}
	}
//	for (int r = 0; r < nBots; r++){
//		if (r == nr) continue;		
//		for ( lineit = robots_lines[r].begin(); lineit < robots_lines[r].end(); lineit++ ){
//			if (intersect(ray, *lineit, crosspoint)){
//				diffx = ori.x-crosspoint.x;
//				diffy = ori.y-crosspoint.y;
//				float aux = diffx*diffx+diffy*diffy;
//				if (aux < dist)	dist = aux;
//			}
//		}
//	}
	return (sqrt(dist));
}

// TODO: consider rotation angles rx, ry, rz
pose3d simulatedModel::camPose(const pose& robotpose){
	float cth = cos(robotpose.th);
	float sth = sin(robotpose.th);
	return pose3d( robotpose.x + camx*cth+camy*sth, robotpose.y + camx*sth+camy*cth, camz, robotpose.th);
}

// TODO: use calibration rx, ry, rz
// translates a position in the camera frame of reference to the robot's frame of reference
pos3d simulatedModel::cam2base(const pos3d& p){
	return pos3d(p.getX() + camx,
				 p.getY() + camy,
				 p.getZ() + camz);
}

// translates a position in the robot's frame of reference to the camera frame of reference
pos3d simulatedModel::base2cam(const pos3d& p){
	return pos3d(p.getX() - camx,
				 p.getY() - camy,
				 p.getZ() - camz);
}

robotBase* simulatedModel::createNewPlatform(){
	robotBase* res;
	newIdsMutex.lock();
	if (idCounter<nBots){
		rbase[idCounter] = new robotBase(idCounter);
		rbase[idCounter]->setWorldModel(this);
		res = rbase[idCounter];
		idCounter++;
	}
	else	res= 0;
	newIdsMutex.unlock();
	//printf("the rbase created is at pointer %d",res);
	return res;
}

void simulatedModel::disablePlatform(const robotBase& r){
	int id = r.getNumber();
	//printf("[SIM] Disabling robot...%d\n",id);
	if (robotsEnabled[id]){
		rbase[id]->beginConsumition();
		robotsEnabled[id]=false;
		rbase[id]->endConsumition();
	}
	//printf("[SIM] robot %d disabled\n",id);
}

int simulatedModel::fire(int robot){
	int res = 0;
	pointf crosspoint;
	for (int r = 0; r < nBots; r++){
		if (r == robot) continue; 
		if (!rbase[r]) continue;

		if (rbase[r]->isCaptured()) continue;

		line ray(gt[robot].x,gt[robot].y,gt[r].x,gt[r].y);
		std::vector<line>::iterator lineit;
		bool visibility = true;
		for ( lineit = mapWall.begin(); lineit < mapWall.end(); lineit++ ){
			if (intersect(ray, *lineit, crosspoint)){
				visibility = false;
				break;
			}
		}
		
		if (visibility){
			float distx = gt[robot].x-gt[r].x;
			float disty = gt[robot].y-gt[r].y;
			float dist = sqrt(distx*distx+disty*disty);
			if (dist < maxDist){
				res++;
				rbase[r]->setCaptured();
			}
		}
	}
	return res;
}

double simulatedModel::evaluateVMap(visualMap& vm) const{
	double error =0;
	double suma=0;
	int N=vm.getNLandmarks();
	for (int l = 0; l<N; l++){
		landmark* m = vm.getLandmarkById(l);
		suma += sqrt( pow( landmarks[m->descriptor[0]].pos.getX() - m->pos.getX() ,2) + 
			      pow( landmarks[m->descriptor[0]].pos.getY() - m->pos.getY() ,2) +
			      pow( landmarks[m->descriptor[0]].pos.getZ() - m->pos.getZ() ,2) );
		delete m;
	}
	error = sqrt(suma/N);

	return error;

}


