
/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class team
*
*
*/

#pragma once
#ifndef __ROBOT__TEAM___
#define __ROBOT__TEAM___

#include "slamInterface.h"
#include "robot.h"
#include "worldModelInterface.h"
#include "ConfigFile.h"

/**
* @brief Implements a team of robots with global SLAM
*
*/
class team {

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:
	int nrobots;
	robot** theRobots;
	slamInterface* globalSlam;
	worldModelInterface *scene;			// this class is not the owner of this pointer
	char* teamName;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:

	/// Default constructor
	team();
	/// Default destructor
	virtual ~team();
	/// Builds a team of nrobots in the virtual world scene
	team(int nrobots, worldModelInterface& scene, ConfigFile& stratConfig, ConfigFile& slamConfig, int robotType, const char* teamName);
	/// initilizes a team of nrobots in the virtual world scene
	void initialize(int nrobots, worldModelInterface &scene, ConfigFile& stratConfig, ConfigFile& slamConfig, int robotType, const char* teamName);

	/// returns a reference to the robot r
	robot* getRobot(int r) const;
	/// returns the number of robots
	int getNRobots() const;
	/// returns a pointer to the global slam object
	slamInterface* getGlobalSlam() const;

	/// sets the reference to the virtual world
	void setWorldModel(worldModelInterface& scene);

	/// sets the name of the log file
	void setLogName(const char*);

	/// team exploration, it blocks the thread until the exploration is concluded
	void start();
	void waitForTaskFinished();

	/// stops the exploration
	void stop();
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline robot* team::getRobot(int r) const				{return (theRobots[r]);}
inline int team::getNRobots() const					{return nrobots;}
inline slamInterface* team::getGlobalSlam() const			{return globalSlam;}

#endif
