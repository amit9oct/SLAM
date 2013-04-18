/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Abstract Class Planner
*
*/

#pragma once
#ifndef __PLANNER__
#define __PLANNER__

#include <iostream>
#include <stdlib.h>
#include "ComThread.h"
#include "ClMutex.h"
#include "stepEvent.h"
#include "slamInterface.h"
#include "treeNode.h"
#include "reactive.h"

/**
* @brief Implements common methods for planning algorithms
*
*/

class planner: public ComThread{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

public:

	stepEventManager explorationFinished;

protected:

	slamInterface* mySlam;
	reactive* reac;
	robotBase* rbase;
	int number; 
	char* logstr;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:

	planner();
	virtual ~planner();
	planner(const ConfigFile& config){};

	void setSlam(slamInterface& slamProc);
	void setReactive(reactive& reac);
	void setRBase(robotBase& rb);

	void setRobot(int number);
	void setLogName(const char*);

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

typedef Loki::Functor<planner*, LOKI_TYPELIST_1(const ConfigFile&)> plannerCreator;
typedef Loki::SingletonHolder< Loki::Factory< planner, int, LOKI_TYPELIST_1(const ConfigFile&)> > plannerFactory;

#endif
