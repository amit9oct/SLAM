/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class robot
*
*/
#pragma once
#ifndef __ROBOT__
#define __ROBOT__

#include "slamInterface.h"
#include "reactive.h"
#include "planner.h"
#include "robotBase.h"
#include "worldModelInterface.h"
#include "stepEvent.h"
#include "ConfigFile.h"

namespace robots{
	const int EXPLORER = 0;
}


/**
* @brief Implements the architecture for a mobile robot
*/
class robot{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
protected:
	int number;					// robot number in the team
	char* strlog;
	slamInterface* mySlam;				// this class is not the owner of this pointer
	reactive* reac;
	planner* plan;
	worldModelInterface* scene;			// this class is not the owner of this pointer
	robotBase* rbase;
	const ConfigFile* config;			// this class is not the owner of this pointer

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:
	/// Default Constructor
	robot();
	/// Default Destructor
	virtual ~robot();

	/// Constructor
	/// \param scene reference to virtual world object
	/// \param slamProc reference to slam object
	robot(int number, slamInterface& slamProc, worldModelInterface& scene, const ConfigFile& config);
	/// Initializer
	/// \param scene virtual world
	/// \param slamProc pointer to slam object
	virtual void initialize(int number, slamInterface& slamProc, worldModelInterface& scene, const ConfigFile& config);
	
	/// returns the robot number
	int getNumber() const;

	/// returns the robot id
	int getId() const;

	/// sets the reference to the slam object
	void setSlam(slamInterface& slamProc);

	/// sets the reference to the virtual world object
	void setWorldModel(worldModelInterface& scene);

	/// sets the log file
	void setLogName(const char* str);
	/// returns the reference to the planner exploration concluded event manager
	stepEventManager& endEvent() const;

	/// Initiatites the hybrid exploration of the environment
	virtual void start()=0;

	/// stops all actions execution
	virtual void stop()=0;

	void disable();
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline int robot::getNumber() const					{return number;}
inline int robot::getId() const						{return rbase->getNumber();}
inline void robot::setSlam(slamInterface& slamProc)			{mySlam = &slamProc;}
inline void robot::setWorldModel(worldModelInterface& s)		{scene = &s;}
inline stepEventManager& robot::endEvent() const			{return plan->explorationFinished;}
inline void robot::disable()						{mySlam->disableRobotBase(number); scene->disablePlatform(*rbase); }

typedef Loki::Functor<robot*, LOKI_TYPELIST_4(int, slamInterface&, worldModelInterface&, const ConfigFile&)> robotCreator;
typedef Loki::SingletonHolder< Loki::Factory< robot, int, LOKI_TYPELIST_4(int, slamInterface&, worldModelInterface&, const ConfigFile&)> > robotFactory;


#endif

