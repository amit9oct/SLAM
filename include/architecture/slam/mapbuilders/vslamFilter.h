/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Abstract Class SLAM Filter
*
* 
*/

#pragma once
#ifndef __SLAM_FILTER__
#define __SLAM_FILTER__

#include "slamInterface.h"
#include "robotBase.h"
#include "matFuns.h"

/**
* @brief Implement common attributes and methods for slam algorithms
*
*/

class vslamFilter: public slamInterface{

	Q_OBJECT

signals:

	void slamUpdated();

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
protected:

	int nBots;					/// number of robots

	float th_high;					/// deslocalization threshold
	float th_low;					/// localization threshold

	float width;					/// real width
	float height;					/// real height
	float resolution;				/// grip map resolution
	int gmwidth;					/// grid map width
	int gmheight;					/// grid map height
	float xorigin;					/// x position in real coordinates of grid map origin
	float yorigin;					/// y position in real coordinates of grid map origin
	
	robotBase** rbase;				/// pointers to robots bases
	bool* robotsEnabled;
	worldModelInterface* scene;			/// pointer to world model

	bool* badlocalized;				/// localization state for each robot (well localized / bad localized)

	char* logstr;					/// name for log results file

	bool displayomap;				/// flag that indicates if the occupancy map must be displayed
	bool displayppmap;				/// flag that indicates if the occupancy map must be displayed
	bool displayipmap;				/// flag that indicates if the occupancy map must be displayed
	bool displayposes;				/// flag that indicates if the position of the robot for each particle must be displayed
	bool displayfeatures;				/// flag that indicates if the features must be displayed
	bool saveAviFile;

	float alfa1;					/// odometry params
	float alfa2;					/// odometry params
	float alfa3;					/// odometry params
	float alfa4;					/// odometry params
	float drifttrans;				/// odometry params

	int gmtype;					/// gridmap algorithm
	int vmtype;					/// visual map implementation
	bool perfectMatching;				/// type of data association
	int nmarks;					/// num of landmarks to reserve

	char* windowName;
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  --------------------------------
///////////////////////////////////////////////////////////////////////

public:
	
	/// Constructor
	vslamFilter();
	/// Destructor
	virtual ~vslamFilter();
	/// Constructor
	vslamFilter(int nrobots, ConfigFile& configFile);

	///initializer	
	void initialize(int nrobots, ConfigFile& configfile);
	
	/// sets the pointer to a robot base object
	void setRobotBase(int robot, robotBase& rbase);
	/// sets the world model
	void setWorldModel(worldModelInterface& scene);

	/// return the number of robots
	int getNumRobots() const;
	/// returns the width of the map
	float getWidth() const;
	/// return the height of the map
	float getHeight() const;
	/// return the x position of the origin of grid map in real coordinates
	float getXOrigin() const;
	/// return the y position of the origin of grid map in real coordinates
	float getYOrigin() const;
	/// returns the resolution of the gridmap
	float getResolution() const;
	/// returns the width in cells of the grid map
	int getGMWidth() const;
	/// returns the height in cells of the grid map
	int getGMHeight() const;

	/// sets the high uncertainty threshold for the localization state 
	void setHighThreshold(float thb);
	/// sets the low uncertainty threshold for the localization state 
	void setLowThreshold(float tha);
	/// return the current state of localization of the robot
	bool getLoc(int robot) const;

	/// returns the current simulation time
	float getTime() const;
	/// returns the period of the simulation time step
	float getSampleTime() const;

	/// returns a copy of the current occupancy map
	virtual gridMapInterface* getOMap() = 0;
	/// returns a copy of the current visual map
	virtual visualMap* getVMap() = 0;
	/// return a copy of the current precise pose map
	virtual binMap* getPreMap() = 0;
	/// returns a copy of the current imprecise pose map
	virtual binMap* getImpMap() = 0;

	/// returns the current pose of the robot
	virtual pose getPos(int robot) = 0 ;
	/// returns the current cell of the robot
	virtual point getCell(int robot) = 0 ;
	/// returns the matrix that represents the covariance of the position of the robot
	virtual matrix getCovariance(int robot) const = 0 ;
	/// returns the matrix that represents the covariance of robots and marks
	virtual Ematrix getGlobalCovariance() const = 0 ;
	/// returns a measure of the dispersion of the robot
	virtual float getDisp(int robot) const = 0;

	/// sets the name of the log file
	void setLogName(const char*);
	/// sets the name of the gridmap file	
	void setGridMapName(const char*){};

	void setWindowName(const char*);

	void disableRobotBase(int r);

	virtual QPixmap* getPixmap()=0;
	virtual QImage* getQImage()=0;


protected:

	/// set up for the execution thread
	virtual int setup()=0;			
	/// primary execution loop
	virtual void execute()=0;			
	/// called before stopping
	virtual void onStop()=0;			

	/// updates precise poses map
	void updatePPMap(const point& rpos, binMap& ppmap, float disp, bool badloc);
	/// updates imprecise poses map
	void updateIPMap(const point& rpos, binMap& ipmap, float disp, bool badloc);

	/// returns a likely evolution of the robot pose using the odometry model
	pose odometryModel(const pose& lastOdo, const pose& deltaOdo, const pose& lastPose);



//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline void vslamFilter::setRobotBase(int robot, robotBase& rb)				{rbase[robot] = &rb;};
inline void vslamFilter::setWorldModel (worldModelInterface& s)				{scene = &s;};
inline int vslamFilter::getNumRobots() const						{return nBots;};
inline float vslamFilter::getWidth() const						{return width;};
inline float vslamFilter::getHeight() const						{return height;};
inline float vslamFilter::getXOrigin() const						{return xorigin;};
inline float vslamFilter::getYOrigin() const						{return yorigin;};
inline int vslamFilter::getGMWidth() const						{return gmwidth;};
inline int vslamFilter::getGMHeight() const						{return gmheight;};
inline float vslamFilter::getResolution() const						{return resolution;};
inline void vslamFilter::setHighThreshold(float thb)					{th_high = thb;};
inline void vslamFilter::setLowThreshold(float tha)					{th_low = tha;};
inline bool vslamFilter::getLoc(int robot) const					{return (robot<nBots)? badlocalized[robot] : 0;};
inline float vslamFilter::getTime() const						{return scene->getTime();};
inline float vslamFilter::getSampleTime() const						{return scene->getSampleTime();};

#endif
