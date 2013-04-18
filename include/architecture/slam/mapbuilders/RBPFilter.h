
/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class RBPFilter
*
* 
*/
#pragma once
#ifndef __RAO_BLACKWELLIZED_PARTILCE_FILTER__
#define __RAO_BLACKWELLIZED_PARTILCE_FILTER__

#include "particle.h"
#include "ClMutex.h"
#include "robotBase.h"
#include "matFuns.h"
#include "vslamFilter.h"

/**
* @brief Implements an slam algorithm consisting on a rao-blackwellized particle filter
*
*/
class RBPFilter: public vslamFilter{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:
	
	int M;						/// number of particles
	
	particle* p;					/// particles data structure
	particle* newp;					/// last particles
	int index;					/// most probable particle
	
	float* dispRobot;				/// measure of uncertainty for each robot
	pose** odoData;					/// odometry values
	pose** lastOdoData;				/// last odometry values
	rangeSensorData** rsData;			/// captured range sensor data 
	landmarksData** lmData;				/// captured features
	
	ClMutex sampling;				/// sampling syncronization mutual exclusion
	ClMutex closing;				/// run/stop control
	bool endSlam;					/// stop condition
	
	bool onlyonegridmap;				/// flag to use only one grid map instead of one per particle
	gridMapInterface* omap;				/// a common occupancy grid for all particles
	binMap* ppmap;					/// a common precise pose binMap for all particles
	binMap* ipmap;					/// a common imprecise pose binMap for all particles
	
	matrix Identity3;				/// 3x3 identity matrix

	float mahTh;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  --------------------------------
///////////////////////////////////////////////////////////////////////

public:
	
	/// Constructor
	RBPFilter();
	/// Destructor
	virtual ~RBPFilter();
	/// Constructor
	RBPFilter(int nrobots, ConfigFile& configFile);
	
	///initializer	
	void initialize(int nrobots, ConfigFile& configfile);
	
	/// returns a copy of the current occupancy map
	gridMapInterface* getOMap();
	/// returns a copy of the current visual map
	visualMap* getVMap();
	/// return a copy of the current precise pose map
	binMap* getPreMap();
	/// returns a copy of the current imprecise pose map
	binMap* getImpMap();

	/// returns the current pose of the robot
	pose getPos(int robot) ;
	/// returns the current cell of the robot
	point getCell(int robot) ;
	/// returns the matrix that represents the covariance of the position of the robot
	matrix getCovariance(int robot) const;
	/// returns the matrix that represents the covariance of robots and marks
	Ematrix getGlobalCovariance() const;
	/// returns a measure of the dispersion of the robot
	float getDisp(int robot) const;

	QImage* getQImage(){return 0;};

	QPixmap* getPixmap();


private:

	/// set up for the execution thread
	int setup();			
	/// primary execution loop
	void execute();			
	/// called before stopping
	void onStop();			

	/// updates fast slam filter  
	void fastSlam(int robot, const landmarksData& lmData, const pose& lastOdo, const pose& odo);
	/// updates one particle of the RBPF
	void updateParticle(int robot, const landmarksData& lmData, const pose& lastOdo, const pose& deltaOdo, int i);
	/// performs the resampling step
	double resample();
	/// updates the occupancy, precise poses and imprecises poses maps
	void updateAll(const rangeSensorData& rsData, int robot);

	/// evaluates the uncertainty on the position of each robot
	void evalDisp(int r);
	
	/// jacobian, this is to update the EKF of each feature
	matrix jacobian(const pose& pos);

	void drawPoses (IplImage& im, float xorigin, float yorigin, float resolution);

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline float RBPFilter::getDisp(int robot) const	 				{return (robot<nBots)? dispRobot[robot] : 0;};

#endif
