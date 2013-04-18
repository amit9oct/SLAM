#pragma once
#ifndef __MARKET__PLANNER__
#define __MARKET__PLANNER__

#include <iostream>
#include <stdlib.h>
#include "ClThread.h"
#include "ClMutex.h"
#include "stepEvent.h"
#include "slamInterface.h"
#include "reactive.h"
#include "planner.h"
#include "pathPlanning.h"



/** 
* @brief Implements an explorative path planner using a cooperative market model for target selection
*
* Each robot has its own list of targets. 
* Periodically new targets are proposed for auction.
* The robot with the best bid includes the target in its list.
*  
*/
class MarketPlanner: public planner{

private: 

/** 
* @brief Target struct
*
* It is used for in the Market Planner target list
*
* This struct contains the information related with the profit of a target.
*/
typedef struct target{
	point cell;
	float utility;
	float cost;
	float profit;
	int auctionEnds;
	int bestOfferID;
	float bestOfferProfit;
	target():utility(0.0f),cost(0.0f),profit(0.0f),auctionEnds(0),bestOfferID(-1),bestOfferProfit(0.0f){}
	bool operator<(const target& t){return profit<t.profit;}
}target;


/** 
* @brief Public auction, data contains the target cell and the auction publisher
*/
typedef struct auct{
	point cell;
	int auctioner;
}auct;


///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////


	ClMutex closing;				// Mutex for a right stopping of the thread
	bool endPlanner;				// Flag that indicates that the thread must conclude in the next iteration 
//	bool showPlanner;				// Flag that indicates if the planning figure must be displayed
	bool completedPath;				// Flag to set when path is finished
	std::vector<point> path;			// planned path

	std::list<target> targetList;

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
	MarketPlanner(const ConfigFile& config);
	/// Destructor
	virtual ~MarketPlanner();

private:

	/// Thread setup
	int setup();					
	/// Thread on stop
	void onStop();					
	/// Thead execution body
	void execute();					
	/// Evals the utility of an area
	int utility(const gridMapInterface& omap, const binMap& esz); 

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

#endif

