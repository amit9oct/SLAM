/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2010
* 
* Class explorer
*
*/

#pragma once
#ifndef __EXPLORER__
#define __EXPLORER__

#include "robot.h"

#define HYBRIDPLANNER			0
#define FSAPLANNER			1
#define FRONTIERGREEDYPLANNER		2
#define IGPLANNER			3
#define COORDINATEDPLANNER		4
#define MARKETPLANNER			5
#define INTEGRATEDIGPLANNER		6

#define REACTIVEGAUSS			0
#define REACTIVEINV			1

/**
*
* @brief Implements the architecture of a mobile robot that explores the environment
*
*/

class explorer: public robot{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:

	int plantype;
	int reactype;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:
	/// Default Constructor
	explorer();
	/// Default Destructor
	virtual ~explorer();

	/// Constructor
	/// \param number ID number
	/// \param scene reference to virtual world object
	/// \param slamProc reference to slam object
	explorer(int number, slamInterface& slamProc, worldModelInterface& scene, const ConfigFile& config);
	/// Initializer
	/// \param number ID number
	/// \param scene virtual world
	/// \param slamProc pointer to slam object
	void initialize(int number, slamInterface& slamProc, worldModelInterface& scene, const ConfigFile& config);
	
	/// Initiatites the hybrid exploration of the environment
	void start();

	/// stops all actions execution
	void stop();
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};


#endif

