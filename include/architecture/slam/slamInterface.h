/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class slam
*

*
*/

#pragma once
#ifndef __SLAM__INTERFACE__
#define __SLAM__INTERFACE__

#include "gridMapInterface.h"
#include "visualMap.h"
#include "stepEvent.h"
#include "ConfigFile.h"
#include <iostream>
#include <stdlib.h>
#include <loki/Factory.h>
#include <loki/Typelist.h>
#include <loki/Functor.h>
#include "ClThread.h"
#include "robotBase.h"
#include "worldModelInterface.h"
#include <QObject>
#include <QPixmap>

/**
* @brief Interface for landmark-based visual SLAM algorithms
*
*/
class slamInterface: public QObject, public ClThread, public stepEventManager{

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  --------------------------------
///////////////////////////////////////////////////////////////////////

public:
	
	virtual ~slamInterface(){};
	
	///initializer	
	virtual void initialize(int nrobots, ConfigFile& configfile)=0;
	
	/// sets the pointer to a robot base object
	virtual void setRobotBase(int robot, robotBase& rbase)=0;
	/// sets the world model
	virtual void setWorldModel(worldModelInterface& scene)=0;

	/// return the number of robots
	virtual int getNumRobots() const=0;
	/// returns the width of the map
	virtual float getWidth() const=0;
	/// return the height of the map
	virtual float getHeight() const=0;
	/// return the x position of the origin of grid map in real coordinates
	virtual float getXOrigin() const=0;
	/// return the y position of the origin of grid map in real coordinates
	virtual float getYOrigin() const=0;
	/// returns the resolution of the gridmap
	virtual float getResolution() const=0;
	/// returns the width in cells of the grid map
	virtual int getGMWidth() const=0;
	/// returns the height in cells of the grid map
	virtual int getGMHeight() const=0;

	/// sets the high uncertainty threshold for the localization state 
	virtual void setHighThreshold(float thb)=0;
	/// sets the low uncertainty threshold for the localization state 
	virtual void setLowThreshold(float tha)=0;
	
	/// returns a copy of the current occupancy map
	virtual gridMapInterface* getOMap()=0;
	/// returns a copy of the current visual map
	virtual visualMap* getVMap()=0;
	/// return a copy of the current precise pose map
	virtual binMap* getPreMap()=0;
	/// returns a copy of the current imprecise pose map
	virtual binMap* getImpMap()=0;

	/// returns the current pose of the robot
	virtual pose getPos(int robot)=0;
	/// returns the current cell of the robot
	virtual point getCell(int robot)=0;
	/// returns the matrix that represents the covariance of the position of the robot
	virtual matrix getCovariance(int robot) const=0;
	/// returns the matrix that represents the covariance of robots and marks
	virtual Ematrix getGlobalCovariance() const = 0 ;
	/// returns a measure of the dispersion of the robot
	virtual float getDisp(int robot) const=0;
	/// return the current state of localization of the robot
	virtual bool getLoc(int robot) const=0;

	/// returns the current simulation time
	virtual float getTime() const =0;
	/// returns the period of the simulation time step
	virtual float getSampleTime() const =0;

	/// sets the name of the log file
	virtual void setLogName(const char*)=0;
	/// sets the name of the gridmap file	
	virtual void setGridMapName(const char*)=0;

	virtual void setWindowName(const char*)=0;

	virtual void disableRobotBase(int r)=0;

	virtual QPixmap* getPixmap()=0;
	virtual QImage* getQImage()=0;


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

/// create object function format
typedef Loki::Functor<slamInterface*, LOKI_TYPELIST_2(int,ConfigFile&)> slamCreator;
/// Factory
typedef Loki::SingletonHolder< Loki::Factory< slamInterface, int, LOKI_TYPELIST_2(int,ConfigFile&)> > slamFactory;

#endif
