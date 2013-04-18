/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class InformationGainPlanner
*
*/

#pragma once
#ifndef __IG__PLANNER__
#define __IG__PLANNER__

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
* @brief Implements an exploration algorithm that chooses its target cell using a cost-utility model
*
* Value = IG - Cost
* 
* IG: is the expected number of visible cells from that frontier cell
*
* Cost: is the length of the path to arrive to the cell
*
* The target frontier cell that maximizes the value is chosen
*
*/
class InformationGainPlanner: public planner{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

private: 

	ClMutex closing;				// Mutex for a right stopping of the thread
	bool endPlanner;				// Flag that indicates that the thread must conclude in the next iteration 
	bool showPlanner;				// Flag that indicates if the planning figure must be displayed
	bool completedPath;				// Flag to set when path is finished
	std::vector<point> path;		// planned path

	// parameters
	float replanning_period;
	int inflate_obstacles;
	float utility_radius;
	float utility_weight;
	float cost_weight;
	
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:

	/// Constructor
	InformationGainPlanner(const ConfigFile& config);
	/// Destructor
	virtual ~InformationGainPlanner();

private:

	int setup();					// Thread setup
	void onStop();					// Thread on stop
	void execute();					// Thead execution body

	int utility(const gridMapInterface& omap, const binMap& esz); // evals the utility of a frontier

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

};

#endif

