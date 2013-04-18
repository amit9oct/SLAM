#include "vslamFilter.h"


// Constructor
vslamFilter::vslamFilter():
	nBots(0),
	th_high(0),
	th_low(0),
	width(0),
	height(0),
	resolution(0), 
	gmwidth(0),
	gmheight(0),
   	xorigin(0),
	yorigin(0),
	rbase(0),
	robotsEnabled(0),
	scene(0),
	badlocalized(0),
	logstr(0),
	displayomap(false),
	displayppmap(false),
	displayipmap(false),
	displayposes(false),
	displayfeatures(false),
	saveAviFile(false),
	alfa1(0.0f),
	alfa2(0.0f),
	alfa3(0.0f),
	alfa4(0.0f),
	drifttrans(0.0f),
	gmtype(0),
	vmtype(0),
	perfectMatching(false),
	nmarks(0),
	windowName		(0)
{
	printf("Slam Filter default constructor\n");
}

vslamFilter::vslamFilter(int nrobots, ConfigFile& configFile):
	nBots			(nrobots),
	th_high			(configFile.read<float>("THRESHOLD_HIGH")),
	th_low			(configFile.read<float>("THRESHOLD_LOW")),
	width			(configFile.read<float>("SCENE_WIDTH")),
	height			(configFile.read<float>("SCENE_HEIGHT")),
	resolution		(configFile.read<float>("RESOLUTION")), 
	gmwidth			((int)floor(width/resolution+0.5)),
	gmheight		((int)floor(height/resolution+0.5)),
   	xorigin			(configFile.read<float>("XORIGIN")),
	yorigin			(configFile.read<float>("YORIGIN")),
	rbase			(new robotBase*[nBots]),
	robotsEnabled		(new bool[nBots]),
	scene			(0),
	badlocalized		(new bool[nBots]),
	logstr			(0),
	displayomap		((configFile.keyExists("DISPLAYOMAP"))? configFile.read<bool>("DISPLAYOMAP"): false),
	displayppmap		((configFile.keyExists("DISPLAYPPMAP"))? configFile.read<bool>("DISPLAYPPMAP"): false),
	displayipmap		((configFile.keyExists("DISPLAYIPMAP"))? configFile.read<bool>("DISPLAYIPMAP"): false),
	displayposes		((configFile.keyExists("DISPLAYPOSES"))? configFile.read<bool>("DISPLAYPOSES"): false),
	displayfeatures		((configFile.keyExists("DISPLAYFEATURES"))? configFile.read<bool>("DISPLAYFEATURES"): false),
	saveAviFile		((configFile.keyExists("SAVEAVIFILE"))? configFile.read<bool>("SAVEAVIFILE"): false),
	alfa1			(configFile.read<float>("alfa1")),
	alfa2			(configFile.read<float>("alfa2")),
	alfa3			(configFile.read<float>("alfa3")),
	alfa4			(configFile.read<float>("alfa4")),
	drifttrans		(configFile.read<float>("drifttrans")),
	gmtype			((configFile.keyExists("GRIDMAP"))? configFile.read<int>("GRIDMAP"): 3),
	vmtype			((configFile.keyExists("VISUALMAP"))? configFile.read<int>("VISUALMAP"): 0),
	perfectMatching		((configFile.keyExists("PERFECTMATCHING"))? configFile.read<bool>("PERFECTMATCHING"): false),
	nmarks			((configFile.keyExists("NLANDMARKS"))? configFile.read<int>("NLANDMARKS"): 1000),
	windowName		(0)
{
	for (int r = 0; r< nBots; r++){
		rbase[r] = 0;
		badlocalized[r]=false;
	}
	printf("[VSLAM] Multi-Robot SLAM created\n");
}

vslamFilter::~vslamFilter(){
	//printf("slam destroyer...");
	if (rbase)		delete[] rbase;
	if (robotsEnabled)	delete[] robotsEnabled;
	if (badlocalized)	delete[] badlocalized;
	if (logstr)		delete[] logstr;

	//printf("Slam Filter destroyed\n");
}

// Initializer
void vslamFilter::initialize(int nrobots, ConfigFile& configFile){
	nBots			= nrobots;
	th_high			= configFile.read<float>("THRESHOLD_HIGH");
	th_low			= configFile.read<float>("THRESHOLD_LOW");
	width			= configFile.read<float>("SCENE_WIDTH");
	height			= configFile.read<float>("SCENE_HEIGHT");
	resolution		= configFile.read<float>("RESOLUTION"); 
	gmwidth			= (int)floor(width/resolution+0.5);
	gmheight		= (int)floor(height/resolution+0.5);
   	xorigin			= configFile.read<float>("XORIGIN");
	yorigin			= configFile.read<float>("YORIGIN");
	if (rbase)		delete[] rbase;
	rbase			= new robotBase*[nBots];
	if (robotsEnabled)	delete[] robotsEnabled;
	robotsEnabled		=new bool[nBots];
	scene			= 0;
	if (badlocalized) 	delete[] badlocalized;
	badlocalized		= new bool[nBots];
	if (logstr)		delete[] logstr;
	logstr			= 0;
	displayomap		= (configFile.keyExists("DISPLAYOMAP"))? configFile.read<bool>("DISPLAYOMAP") : false;
	displayppmap		= (configFile.keyExists("DISPLAYOMAP"))? configFile.read<bool>("DISPLAYPPMAP") : false;
	displayipmap   		= (configFile.keyExists("DISPLAYOMAP"))? configFile.read<bool>("DISPLAYIPMAP") : false;
	displayposes		= (configFile.keyExists("DISPLAYOMAP"))? configFile.read<bool>("DISPLAYPOSES") : false;
	displayfeatures 	= (configFile.keyExists("DISPLAYOMAP"))? configFile.read<bool>("DISPLAYFEATURES") : false;
	saveAviFile		= (configFile.keyExists("DISPLAYOMAP"))? configFile.read<bool>("SAVEAVIFILE") : false;
	alfa1			= configFile.read<float>("alfa1");
	alfa2			= configFile.read<float>("alfa2");
	alfa3			= configFile.read<float>("alfa3");
	alfa4			= configFile.read<float>("alfa4");
	drifttrans		= configFile.read<float>("drifttrans");

	gmtype			= (configFile.keyExists("GRIDMAP"))? configFile.read<int>("GRIDMAP") : 3;
	vmtype			= (configFile.keyExists("VISUALMAP"))? configFile.read<int>("VISUALMAP") : 0;
	perfectMatching		= (configFile.keyExists("PERFECTMATCHING"))? configFile.read<bool>("PERFECTMATCHING") : false;
	nmarks			= (configFile.keyExists("NLANDMARKS"))? configFile.read<int>("NLANDMARKS") : 1000;
	windowName		= 0;
	resetCounter();

	for (int r = 0; r< nBots; r++){
		rbase[r] = 0;
		badlocalized[r]=false;
	}
	//printf("[VSLAM] Multi-Robot SLAM initialized\n");
}

QPixmap* vslamFilter::getPixmap(){
	return 0;
}


// odometry model
pose vslamFilter::odometryModel(const pose& lastOdo, const pose& deltaOdo, const pose& lastPose){
	
	// modelo de 3 movimientos: rotacion rot1, avance trans y rotacion rot2.
	
	float deltarot1  = atan2(deltaOdo.y, deltaOdo.x) - lastOdo.th; // cuanto ha rotado 
	float deltatrans = drifttrans*sqrt((deltaOdo.x*deltaOdo.x+deltaOdo.y*deltaOdo.y)); // distancia avanzada
	float deltarot2  = deltaOdo.th - deltarot1;	// incremento de orientacion aparte de la rotacion
	
	float dtheta = fabs(atan2(sin(deltaOdo.th),cos(deltaOdo.th)));
	
	float aux = (alfa1*dtheta + alfa2*deltatrans)/2;
	
	float deltarot1g  = (float)(normrnd(deltarot1,  aux));
	float deltatransg = (float)(normrnd(deltatrans, alfa3*deltatrans + alfa4*dtheta));
	float deltarot2g  = (float)(normrnd(deltarot2,  aux));
 	
	float aux2 = lastPose.th+deltarot1g;
	
	return pose(lastPose.x  + deltatransg * cos(aux2),
				lastPose.y  + deltatransg * sin(aux2),
				lastPose.th + deltarot1g + deltarot2g);
}

void vslamFilter::updatePPMap(const point& rpos, binMap& map, float disp, bool badloc){
	if (disp < th_low)
		map.set(rpos.x, rpos.y, true);
	else
		map.set(rpos.x, rpos.y, false);
	if(disp > th_high){
		for (int x = rpos.x-3; x<= rpos.x+3; x++)
			for (int y = rpos.y-3; y<= rpos.y+3; y++)
				map.set(x, y, false);
	}
}

void vslamFilter::updateIPMap(const point& rpos, binMap& map, float disp, bool badloc){
	if (disp > th_high)
		map.set(rpos.x, rpos.y, true);
	else if(!badloc){
		for (int x = rpos.x-4; x<= rpos.x+4; x++)
			for (int y = rpos.y-4; y<= rpos.y+4; y++)
				map.set(x, y, false);
	}
}


void vslamFilter::setLogName(const char* str){
	if (logstr)	delete[] logstr;
	logstr = new char[strlen(str)+1];
	strcpy(logstr,str);
}


void vslamFilter::setWindowName(const char* str){
	if (windowName)	delete[] windowName;
	windowName = new char[strlen(str)+1];
	strcpy(windowName,str);
}

void vslamFilter::disableRobotBase(int r){
	if (robotsEnabled[r]){
//		printf("[VSLAM] disabling robot %d...\n",r);
		rbase[r]->beginProduction();
		robotsEnabled[r]=false;
		rbase[r]->endProduction();
//		printf("[VSLAM] robot %d disabled\n",r);
	}
}
