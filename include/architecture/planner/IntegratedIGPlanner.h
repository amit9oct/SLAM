/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class IntegratedIGPlanner
*
*/

#pragma once
#ifndef __IIG__PLANNER__
#define __IIG__PLANNER__

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
* @brief Implements an exploration algorithm directing the robot to the most informative path
*
* VAlue = IG + LOCALIZ - Cost
* 
* IG: is the expected number of visible cells from that frontier cell
*
* LOCALIZ: Predicted localization level at the target point
*
* Cost: is the length of the path to arrive to the cell
*
* The target frontier cell that maximizes the value is chosen
*
*/
class IntegratedIGPlanner: public planner{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

private:

	//costMap* policy;
	bool completedPath;				// Flag to set when path is finished
	bool showPlanner;				// Flag that indicates if the planning figure must be displayed
	bool endPlanner;				// Flag that indicates that the thread must conclude in the next iteration 
	ClMutex closing;				// Mutex for a right stopping of the thread
	std::vector<point> path;			// planned path
	float vmax;

	// parameters
	float replanning_period;
	int inflate_obstacles;
	float utility_radius;
	float utility_weight;
	float cost_weight;
	float localization_weight;
	float camrange;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:

	/// Constructor
	IntegratedIGPlanner(const ConfigFile& config);
	/// Destructor
	virtual ~IntegratedIGPlanner();

private:

	int setup();					// Thread setup
	void onStop();					// Thread on stop
	void execute();					// Thead execution body

	float IGutility(const gridMapInterface& omap, const binMap& esz);
	float LOCutility(const point& oc, const binMap esz, visualMap& vmap, const Ematrix& P);
	
	matrix jacobianHm(const pose& pos);
	matrix jacobianHv(const pose& pos, const pos3d& mark);
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

#endif

