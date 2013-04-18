#include "planner.h"


planner::planner():
	ComThread(),
	mySlam(0),
	reac(0),
	rbase(0),
	number(0),
	logstr(0)
{

}
planner::~planner(){
	delete [] logstr;
}


void planner::setSlam(slamInterface& slamProc){
	mySlam = &slamProc;
}

void planner::setRobot(int n){
	number = n;
}

void planner::setReactive(reactive& r){
	reac = &r;
}

void planner::setLogName(const char* str){
	logstr = new char[strlen(str)+1];
	strcpy(logstr,str);
}

void planner::setRBase(robotBase& rb){
	rbase= &rb;
}
