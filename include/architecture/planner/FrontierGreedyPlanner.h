/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class FrontierGreedyPlanner
*

*
*/

#pragma once
#ifndef __FBG__PLANNER__
#define __FBG__PLANNER__

#include <iostream>
#include <stdlib.h>
#include "ClThread.h"
#include "ClMutex.h"
#include "stepEvent.h"
#include "slamInterface.h"
#include "reactive.h"
#include "planner.h"
#include "pathPlanning.h"
#include "ConfigFile.h"

/**
* @brief Implements an exploration algorithm directing the robot to the nearest frontier cell
*
*/

class FrontierGreedyPlanner: public planner{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

private:

//	std::vector<ocell> relpath;			// planned path
	std::vector<point> path;			// planned path
	bool completedPath;				// Flag to set when path is finished
	bool showPlanner;				// Flag that indicates if the planning figure must be displayed
	bool endPlanner;				// Flag that indicates that the thread must conclude in the next iteration 
	ClMutex closing;				// Mutex for a right stopping of the thread
	
	// parameters
	float replanning_period;
	int inflate_obstacles;


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:

	/// Constructor
	FrontierGreedyPlanner(const ConfigFile& config);
	/// Destructor
	virtual ~FrontierGreedyPlanner();

private:

	int setup();					// Thread setup
	void onStop();					// Thread on stop
	void execute();					// Thead execution body

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

};

#endif

