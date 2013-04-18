/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class team
*
* Implements a team of robots with global SLAM
*
*/

#include "team.h"
#include "OGMReflectProb.h"

#ifndef WIN32
#include <unistd.h>
#endif

// Constructor
team::team():
	nrobots(0),
	theRobots(0),
	globalSlam(0),
	scene(0),
	teamName(0)
{
	printf("[TEAM] Team created\n");
}

// destructor
team::~team(){
//	printf("[TEAM] [%s] Team destroyer...\n", teamName);
	stop();
	if (globalSlam) delete globalSlam;
	if (teamName) delete teamName;
	if (theRobots){
		for (int i = 0; i < nrobots; i++) if (theRobots[i]) delete theRobots[i];
		delete[] theRobots;
	}
	printf("[TEAM] Team finished\n");
}

// constructor
team::team(int nr, worldModelInterface& s, ConfigFile& stratConfig, ConfigFile& slamConfig, int robotType, const char* name):
	nrobots(nr),
	theRobots(new robot*[nrobots]),
	globalSlam(slamFactory::Instance().CreateObject(slamConfig.read<int>("SLAM"),nr,slamConfig)),
	scene(&s),
	teamName(0)
{
	printf("[TEAM] Building the new team...\n");	
	for(int r =0; r < nrobots; r++){
		theRobots[r] = robotFactory::Instance().CreateObject(robotType, r, *globalSlam, *scene, stratConfig);
	}
	if (teamName)	delete[] teamName;
	teamName = new char[strlen(name)+1];
	strcpy(teamName,name);
	globalSlam->setWorldModel(s);
	globalSlam->setWindowName(teamName);
	char mapname[100];
	sprintf(mapname,"%somap.saved",teamName);
	globalSlam->setGridMapName(mapname);
	printf("[TEAM] Team created\n");
}

// initializer
void team::initialize(int nr, worldModelInterface& s, ConfigFile& stratConfig, ConfigFile& slamConfig, int robotType, const char* name){
	nrobots=nr;
	if (globalSlam) delete globalSlam;
	globalSlam = slamFactory::Instance().CreateObject(slamConfig.read<int>("SLAM"),nr,slamConfig);
	scene= &s;
	if (theRobots){
		for (int i = 0; i < nrobots; i++) delete theRobots[i];
		delete[] theRobots;
	}
	theRobots = new robot*[nrobots];
	for(int r =0; r< nr; r++){
//		theRobots[r].initialize(r, *globalSlam, *scene, config);
		theRobots[r] = robotFactory::Instance().CreateObject(robotType, r, *globalSlam, *scene, stratConfig);
	}
	if (teamName)	delete[] teamName;
	teamName = new char[strlen(name)+1];
	strcpy(teamName,name);
	globalSlam->setWorldModel(s);
	globalSlam->setWindowName(teamName);
}

// sets the name of the log file
void team::setLogName(const char* str){
	globalSlam->setLogName(str);
	for(int r =0; r< nrobots; r++)
		theRobots[r]->setLogName(str);
}

// set the world model
void team::setWorldModel(worldModelInterface& scene){
	for(int r =0; r< nrobots; r++){
		theRobots[r]->setWorldModel(scene);
	}
	globalSlam->setWorldModel(scene);
}

// team exploration, it blocks the thread until the exploration is concluded
void team::start(){

	globalSlam->run();

	int r;
	for(r =0; r< nrobots; r++){
		theRobots[r]->start();
	}

//	sleepms(100000);
}

void team::waitForTaskFinished(){
	for(int r =0; r< nrobots; r++){
		theRobots[r]->endEvent().waitForStep(1);
		theRobots[r]->disable();
	}
	printf("[TEAM] [%s] All robots finished, stopping SLAM...\n",teamName);
	globalSlam->stop();	
	printf("[TEAM] [%s] Task completed\n",teamName);
}

// stops the explortion
void team::stop(){
	//printf("[TEAM] stoping the robots...\n");
	for(int r =0; r< nrobots; r++){
		theRobots[r]->stop();
		theRobots[r]->disable();
	}
	//printf("[TEAM] stoping the slam...\n");
	globalSlam->stop();
//	printf("[TEAM] stoped\n");
}

