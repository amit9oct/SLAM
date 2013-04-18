/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class FrontierGreedyPlanner
*
*
*/

#pragma once
#ifndef __FSA__PLANNER__
#define __FSA__PLANNER__

#include <iostream>
#include <stdlib.h>
#include "ClThread.h"
#include "ClMutex.h"
#include "stepEvent.h"
#include "slamInterface.h"
#include "reactive.h"
#include "planner.h"
#include "ConfigFile.h"


/**
* @brief Implements a Finite State Automata that performs a reactive behaviour based exploration
*
*/

class FSAPlanner: public planner{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

private:

	bool exploring,					// exploration estate
		returning,					// return to precise poses
		 escapingFromLocalMinimum,	// path following to the nearest frontier
		 returningEscape;			// path following to the nearest precise pose
	bool showPlanner;				// Flag that indicates if the planning figure must be displayed
	bool endPlanner;				// Flag that indicates that the thread must conclude in the next iteration 
	ClMutex closing;				// Mutex for a right stopping of the thread
	std::vector<point> path;		// escape from local minimum planned path
	bool completedPath;				// Flag to set when path is finished

	// parameters
	int inflate_obstacles;
	bool integrate_slam;
	bool escape_allowed;


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:

	/// Constructor
	FSAPlanner(const ConfigFile& config);
	/// Destructor
	virtual  ~FSAPlanner();

private:
	
	int setup();					// Thread setup
	void onStop();					// Thread on stop
	void execute();					// Thead execution body

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

#endif
