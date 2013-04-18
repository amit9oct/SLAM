
#include "explorer.h"

#include "reactiveGauss.h"

#include "HybridPlanner.h"
#include "FrontierGreedyPlanner.h"
#include "FSAPlanner.h"
#include "CoordinatedPlanner.h"
#include "IGPlanner.h"
#include "MarketPlanner.h"
#include "IntegratedIGPlanner.h"

// Para registrar la clase en la factoria
namespace robots{
	robot* CreateExplorer(int num, slamInterface& sp, worldModelInterface& s, const ConfigFile& conf){
		return new explorer(num, sp, s, conf);
	};
	const bool registered = robotFactory::Instance().Register(EXPLORER, robotCreator(CreateExplorer));
}

// constructor
explorer::explorer():
	plantype(0),
	reactype(0)
{
	printf("[EXPLORER] Explorer Robot created\n");
}

// constructor
explorer::explorer(int num, slamInterface& sp, worldModelInterface& s, const ConfigFile& conf):
	robot(num, sp,s,conf),
	plantype(conf.read<int>("PLANNER")),
	reactype(conf.read<int>("REACTIVE"))
{
	printf("[EXPLORER] Explorer Robot created\n");	
}

// destructor
explorer::~explorer(){
//	printf("[EXPLORER] Explorer destroyer...\n");
	printf("[EXPLORER] Explorer finished\n");
}

// initializer
void explorer::initialize(int num, slamInterface& sp, worldModelInterface& s, const ConfigFile& conf){
	robot::initialize(num,sp,s,conf);
	plantype = config->read<int>("PLANNER");
	reactype = config->read<int>("REACTIVE");
}


// initiates the exploration
// this method acts as a factory for planner and reactive layers
// TODO: use default factory
void explorer::start(){

	reac = reactiveFactory::Instance().CreateObject(reactype,*config);

	reac->setRobot(number);
	reac->setRBase(*rbase);
	reac->setSlam(*mySlam);

	plan = plannerFactory::Instance().CreateObject(plantype,*config);

	if (strlog) plan->setLogName(strlog);
	plan->setRobot(number);
	plan->setRBase(*rbase);
	plan->setReactive(*reac);
	plan->setSlam(*mySlam);

	plan->run();
	reac->run();

}

// stops the exploration
void explorer::stop(){
//	printf("[EXPLORER] stoping...\n");
	if (plan) plan->stop();
	if (reac) reac->stop();
//	printf("[EXPLORER] stoped\n");
}



