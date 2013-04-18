
/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class virtualWorld
* 
*/

#pragma once
#ifndef __SIMULATED__MODEL__
#define __SIMULATED__MODEL__

#include <QObject>
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include "worldModelInterface.h"
#include "gridMapInterface.h"
#include "visualMap.h"

/**
* @brief Feature Struct. Contains the position and descriptor of a landmark
*
*/
typedef struct feature{
	pos3d pos;
	float* desc;
	int desclength;
	feature():desc(0),desclength(0){};
	feature(const pos3d& p, float* d, int l):pos(p),desclength(l){ 
		desc = new float[desclength]; 
		memcpy(desc,d,desclength*sizeof(float)); 
	};
	feature(const feature& f): pos(f.pos), desclength(f.desclength){
		desc = new float[desclength]; 
		memcpy(desc,f.desc,desclength*sizeof(float));
	};
	feature& operator=(const feature& f){
		pos = f.pos;
		if (desclength != f.desclength){
			desclength = f.desclength;
			if (desc) delete[] desc;
			desc = new float[desclength]; 
		}
		memcpy(desc,f.desc,desclength*sizeof(float));
		return *this;
	};
	~feature(){if (desc) delete[] desc;};
}feature;


/**
* @brief  This class implements a virtual world for robots where robots can move and observe the environment
*
*/

class simulatedModel:  public worldModelInterface{

	Q_OBJECT

signals:
	
	//void redraw(const QPixmap scenario);
	void changedPositions(rposes poses);

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

protected:

	void redrawing();

	int nBots;					// number of robots
	int maxNumRobots;
	int step;

	// current sensors data structures
	rangeSensorData* rsData;
	landmarksData* lmData;
	pose* odo;
	pose* gt;
	pose* lastodo;
	pose* lastgt;
	robotBase** rbase;
	bool* robotsEnabled;

	// actuators values
	speed* velo;

	char* logstr;

	float alfa1;					/// odometry params
	float alfa2;					/// odometry params
	float alfa3;					/// odometry params
	float alfa4;					/// odometry params
	float drifttrans;				/// odometry params

	// Datos de calibracion

	int imgwidth;
	int imgheight;

	float sigma2f; 
	float sigmaf; 
	float f; 
	float Realf;

	float sigma2I; 
	float sigmaI; 
	float I; 
	float RealI;

	float sigma2Cx; 
	float sigmaCx; 
	float Cx;
	float RealCx;

	float sigma2Cy; 
	float sigmaCy; 
	float Cy;
	float RealCy;

	float sigma2Cxp; 
	float sigmaCxp; 
	float Cxp;
	float RealCxp;

	float sigmar; 
	float sigmac; 
	float sigmacp; 
	float sigmad;
	float sigma2r; 
	float sigma2c; 
	float sigma2cp; 
	float sigma2d;

	float betaMAX;
	float gammaMAX;
	float distMAX;
	float distMIN;

	double camx;						/// camera position
	double camy;						/// camera position
	double camz;						/// camera position
	double rx;						/// camera position
	double ry;						/// camera position
	double rz;						/// camera position

	int numSensors;						/// sonar parameters
	float devError;
	float realError;
	float gammamax;
	float aperture;
	float maxDist;
	float minDist;

	float sample_time;
	float elapsedTime;

	std::vector<double> robot_footprint;					
	std::vector< std::vector<line> > robots_lines;		/// wall / obstacles

	bool endth;						/// stop condition
	ClMutex closing;					/// syncronization mutex on closing
	ClMutex newIdsMutex;
	int idCounter;

	// map objects
	std::vector<pose> robotPose;				/// robot positions
	std::vector<feature> landmarks;				/// landmarks
	std::vector<line> mapWall;				/// wall / obstacles
	int desclength;

	matrix Scam;						/// sensor covariance matrix

	float scalex;
	float scaley;
	float maxx, maxy, minx, miny;
	int width, height;

	bool showsimulation;

//	bool groupInitialPoses;
//	int maxInitialPoseDist;
//	double minInitialPoseDist;
	int numfeat;
	bool mapLoaded;

	gridMapInterface* omap;
	
	char* omapfilename;



//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:
	/// default constructor
	simulatedModel();
	/// destructor
	virtual ~simulatedModel();
	/// constructor
	simulatedModel(int nBots, ConfigFile& configfile, float sampletime);
	
	/// initializes a virtual world with "nBots" robots in the scenario "scene" and initial poses given by "test" 
	void initialize(int nagents, ConfigFile& configfile, float sampletime);	

	void reset();

	/// synchronizer
	production synchronizer;

	/// load a map file
	void loadMapFile(const ConfigFile& config);

	robotBase* createNewPlatform();
	void disablePlatform(const robotBase& r);

	/// returns the range sensor current data
	rangeSensorData* getRangeSensorData(int robot);
	/// returns the landmarks data
	landmarksData* getLandmarksData(int robot);
	/// returns the robot odometry
	pose* getOdometry(int robot);

	const char* getOMapFileName() const;

	/// sets the linear and angular speed for a robot
	void setSpeed(int r, float v, float w);
	/// sets the name of the log file
	void setLogName(const char*);

	int fire(int robot);

	int getNumRobots();
	void setNumRobots(int num);

	QPixmap* getPixmap();
	float getTime() const;
	float getSampleTime() const;
	void randomPoses(bool group, double maxInitialPoseDist, double minInitialPoseDist);
	
	rposes getPoses() const;

	double evaluateVMap(visualMap& vm) const;
	
private:

	IplImage* getMapImage();
	void drawRobots(IplImage*& img) ;

	/// set up for the execution thread
	int setup();			
	/// primary execution loop
	void execute();	
	/// called before stopping
	void onStop();	
	
	/// moves the robot with speed vel for time st
	void robotMotion(pose &pos, const speed& vel, float st);
	/// creates a new odometry reading
	void motionSample(const pose& lastPose, const pose& newPose, const pose& lastodo, pose& newodo);
	/// captures features from camPose
	void simCamAdquisition(const pose& robotpos, landmarksData& lmData, int nr);
	/// simulates sonar/laser readings
	void simRangeAdquisition(const pose& realPose, rangeSensorData& sonarData, int r); 
	/// auxiliary function to test when a feature is visible or not
	bool testVisibility(const pos3d& realPose, const pos3d& mark, int nr);
	/// returns the distance from ori to dest or the first wall in that direction
	float visibilityDistance(const pose& ori, const pose& dest, int nr);

	/// returns the pose of the camera
	pose3d camPose(const pose& robotpose);

	/// translates a position in the camera frame of reference to the robot's frame of reference
	pos3d cam2base(const pos3d& p);
	/// translates a position in the robot's frame of reference to the camera frame of reference
	pos3d base2cam(const pos3d& p);

	void update_footprint(int r);

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline rangeSensorData* simulatedModel::getRangeSensorData(int r)	{return new rangeSensorData(rsData[r]);}
inline landmarksData* simulatedModel::getLandmarksData(int r)		{return new landmarksData(lmData[r]);}
inline pose* simulatedModel::getOdometry(int r)				{return new pose(odo[r]);}
inline void simulatedModel::setSpeed(int r, float v, float w)		{if (r<nBots){velo[r].v=v;velo[r].w=w;}}
inline int simulatedModel::getNumRobots()				{return nBots;};
inline const char* simulatedModel::getOMapFileName() const		{return omapfilename;};
inline float simulatedModel::getTime() const				{return elapsedTime;};
inline float simulatedModel::getSampleTime() const			{return sample_time;};
#endif



