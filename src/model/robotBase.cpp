/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class robotBase
*
* This class connects with the real robot in real experiments
*
* or runs a thread in a virtual world
*
*/


#include "robotBase.h"
#include <stdio.h>
#include "worldModelInterface.h"

// Constructors

robotBase::robotBase():
	scene(0),
	number(0),
	captured(false)
{
//	printf("[ROBOTBASE] RobotBase created\n");
}

robotBase::robotBase(int n):
	number(n),
	captured(false)
{
//	printf("[ROBOTBASE] RobotBase created\n");
}

robotBase::~robotBase(){
//	printf("[ROBOTBASE] RobotBase finished\n");
}

void robotBase::setWorldModel(void* s) {
	scene = (worldModelInterface*) s;
}


rangeSensorData* robotBase::getRangeSensorData(){
	return ((worldModelInterface*)scene)->getRangeSensorData(number);
}

landmarksData* robotBase::getLandmarksData() {
	return ((worldModelInterface*)scene)->getLandmarksData(number);
}

pose* robotBase::getOdometry() {
	return ((worldModelInterface*)scene)->getOdometry(number);
}

void robotBase::setSpeed(float v, float w){
	((worldModelInterface*)scene)->setSpeed(number,v,w);
}

int robotBase::fire(){
	return ((worldModelInterface*)scene)->fire(number);
}

