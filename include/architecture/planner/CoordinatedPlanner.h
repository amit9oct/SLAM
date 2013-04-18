/**
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class CoordinatedPlanner
*
*
*/
#pragma once
#ifndef __COORDINATED__PLANNER__
#define __COORDINATED__PLANNER__

#include <iostream>
#include <stdlib.h>
#include "ClThread.h"
#include "stepEvent.h"
#include "slamInterface.h"
#include "reactive.h"
#include "planner.h"
#include "pathPlanning.h"
#include "ConfigFile.h"

/**
*
* @brief Implements an exploration algorithm that chooses its target cell using a cost-utility model
*
* Value = Utility - Cost
* 
* Utility: is a function to the distance to the cells assigned to the other robots
*
* Cost: is the length of the path to arrive to the cell
*
* The target frontier cell that maximizes the value is chosen
*/

class CoordinatedPlanner: public planner{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

private: 

	ClMutex closing;				// Mutex for a right stopping of the thread
	bool endPlanner;				// Flag that indicates that the thread must conclude in the next iteration 
	bool showPlanner;				// Flag that indicates if the planning figure must be displayed
	bool completedPath;				// Flag to set when path is finished
	std::vector<point> path;			// planned path

	point* destinations;				// destinations selected by other robots

	// parameters
	float replanning_period;
	int inflate_obstacles;
	float influence_radius;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:

	/// Constructor
	CoordinatedPlanner(const ConfigFile& config);
	/// Destructor
	virtual ~CoordinatedPlanner();

private:

	/// Thread setup
	int setup();					
	/// Thread on stop
	void onStop();	
	/// Thead execution body				
	void execute();					

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

};

#endif

