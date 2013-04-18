/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class HybridPlanner
*
*/
#pragma once
#ifndef __HYBRID__PLANNER__
#define __HYBRID__PLANNER__

#include <iostream>
#include <stdlib.h>
#include "ClThread.h"
#include "ClMutex.h"
#include "stepEvent.h"
#include "slamInterface.h"
#include "treeNode.h"
#include "reactive.h"
#include "planner.h"
#include "ConfigFile.h"


/**
* @brief Implements a Hybrid Deliberative/Reactive Planner 
*
*/

class HybridPlanner: public planner{

private:

/// Gateway cell structure for hybrid planner
typedef struct door{
	float x;
	float y;
	float scale;
	door(): x(0.0f),y(0.0f), scale(0.0f){}
}door;


///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

protected:
	
	bool badlocalized;			// state flags
	int state;				// state of the planner
	bool showPlanner;			// Flag that indicates if the planning figure must be displayed
	bool endPlanner;			// Flag that indicates that the thread must conclude in the next iteration 
	ClMutex closing;			// Mutex for a right stopping of the thread
	IplImage* im;

	// parameters
	int eszdilationradius;			
	float actionradius;	
	int min_frontier_length;
	int min_gateway_length;
	float utility_radius;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:

	/// Constructor
	HybridPlanner(const ConfigFile& config);
	/// Destructor
	virtual ~HybridPlanner();

protected:

	int setup();					// Thread setup
	void onStop();					// Thread on stop
	void execute();					// Thead execution body

	/// creates an exploration tree
	treeNode* createExplorationTree(int numrobots, const point* robotcells, const point& pos, const gridMapInterface& omap, const binMap& precisemap, const binMap& imprecisemap);
	/// seeks for adjacent zones
	void seekChildren(int numrobots, const point* robotcells, const binMap& safezone, const binMap& filter, treeNode& node, const gridMapInterface& omap, const binMap& precisemap, const binMap& imprecisemap);
	/// clusters a binmap 
	int cluster(int numrobots, const point* robotcells, const binMap& map, treeNode& parent, int nodetype);
	/// recursive clustering
	bool clustering(const int &i, const int &j, binMap& aux, const binMap& map, door& cl, std::list<point> &seq, bool backfront);
	/// evaluates the exploration tree
	void evalTree(treeNode& tree);
	/// evals a zone of the tree
	void evalZone(int numrobots, const point* robotcells, const binMap& map, treeNode& node, const gridMapInterface& omap, const binMap& precisemap, const binMap& imprecisemap);


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

};

#endif

