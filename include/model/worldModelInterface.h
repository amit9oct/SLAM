/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class worldModelInterface
*
*
*/

#pragma once
#ifndef __WORLD__MODEL__INTERFACE__
#define __WORLD__MODEL__INTERFACE__

#include "rangeSensorData.h"
#include "landmarksData.h"

#include "ClThread.h"
#include "ConfigFile.h"
#include "stepEvent.h"
#include "robotBase.h"
#include <QObject>


typedef std::vector< std::vector<line> > rlines;
typedef std::vector< pose > rposes;

/**
* @brief Defines the world model interface

*
*/
class worldModelInterface: public QObject, public ClThread, public production
{
	
///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  --------------------------------
///////////////////////////////////////////////////////////////////////
public:

	virtual ~worldModelInterface(){};

	worldModelInterface():production(1){};

	/// initializer
	virtual void initialize(int nBots, ConfigFile& configfile, float sampletime)=0;

	/// returns the range sensor current data
	virtual rangeSensorData* getRangeSensorData(int robot)=0;
	/// returns the landmarks data
	virtual landmarksData* getLandmarksData(int robot)=0;
	/// returns the robot odometry
	virtual pose* getOdometry(int robot)=0;

	/// sets the linear and angular speed for a robot
	virtual void setSpeed(int r, float v, float w)=0;
	/// sets the name of the log file
	virtual void setLogName(const char*)=0;

	virtual robotBase* createNewPlatform()=0;
	virtual void disablePlatform(const robotBase& r)=0;

	virtual int fire(int robot)=0;

	virtual int getNumRobots()=0;
	virtual void setNumRobots(int num)=0;


	virtual const char* getOMapFileName() const=0;

	virtual float getTime() const=0;
	virtual float getSampleTime() const=0;
	virtual void randomPoses(bool group, double maxInitialPoseDist, double minInitialPoseDist)=0;

	virtual rposes getPoses() const=0;


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

#endif

