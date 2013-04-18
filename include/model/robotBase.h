/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class robotBase
*

*/

#pragma once
#ifndef __ROBOT__BASE__
#define __ROBOT__BASE__

#include "ClThread.h"
#include "stepEvent.h"
#include "rangeSensorData.h"
#include "landmarksData.h"
#include "robotTypes.h"

/**
* @brief This class reference a robotic plattform in the simulator
*
*/
class robotBase: public production{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:

	void* scene;
	int number;
	bool captured;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:

	/// Default constructor
	robotBase();
	/// Builds a robotBase with ID number and a vitualWorld associated 
	robotBase(int number);
	/// Default destructor
	virtual ~robotBase();

	/// sets the reference to the virtual world object 
	void setWorldModel(void* scence);
	/// sets the robot ID number
	void setNumber(int number);
	/// sets the robot ID number
	int getNumber() const;

	/// returns the range sensor current data
	rangeSensorData* getRangeSensorData();
	/// returns the landmarks data
	landmarksData* getLandmarksData();
	/// returns the robot odometry
	pose* getOdometry();

	/// sets the linear and angular speed
	void setSpeed(float v, float w);

	int fire();
	void setCaptured();
	bool isCaptured();


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline void robotBase::setNumber(int n)					{number = n;}
inline int robotBase::getNumber() const					{return number;}
inline void robotBase::setCaptured()					{captured = true;}
inline bool robotBase::isCaptured()					{return captured;}
#endif 

