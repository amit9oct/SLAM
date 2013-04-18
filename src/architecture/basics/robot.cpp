#include <iostream>
#include <stdlib.h>

#include "robot.h"

#include "reactiveGauss.h"

#include "HybridPlanner.h"
#include "FrontierGreedyPlanner.h"
#include "FSAPlanner.h"
#include "CoordinatedPlanner.h"
#include "IGPlanner.h"
#include "MarketPlanner.h"
#include "IntegratedIGPlanner.h"

// constructor
robot::robot():
	number(0),
	strlog(0),
	mySlam(0),
	reac(0),
	plan(0),
	scene(0),
	rbase(0),
	config(0)
{
//	printf("[ROBOT] Robot created\n");
}

// constructor
robot::robot(int num, slamInterface& sp, worldModelInterface& s, const ConfigFile& conf):
	number(num),
	strlog(0),
	mySlam(&sp),
	reac(0),
	plan(0),
	scene(&s),
	rbase(s.createNewPlatform()),
	config(&conf)
{
	mySlam->setRobotBase(number,*rbase);
//	printf("[ROBOT] Robot created\n");
}

// destructor
robot::~robot(){
//	printf("[ROBOT] Robot destroyer...\n");
	if (plan){
		plan->stop();
		delete plan;
	}
	if (reac){
		reac->stop();
		delete reac;
	}
	if (strlog) delete[] strlog;
	if (rbase) delete rbase;
//	printf("[ROBOT] Robot destroyed\n");
}

// initializer
void robot::initialize(int num, slamInterface& sp, worldModelInterface& s, const ConfigFile& conf){
	number = num;
	if (strlog) delete[] strlog;
	setSlam(sp);
	if (plan){
		plan->stop();
		delete plan;
		plan = 0;
	}
	if (reac){
		reac->stop();
		delete reac;
		reac = 0;
	}
	setWorldModel(s);
	if (rbase) delete rbase;
	rbase = s.createNewPlatform();
	mySlam->setRobotBase(number,*rbase);
	config = &conf;
}

// sets the log file name
void robot::setLogName(const char* str){
	if (strlog) delete[] strlog;
	strlog = new char[strlen(str)+1];
	strcpy(strlog,str);
}


