/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class binMap
*
*/
#ifndef __RANGE__SENSOR__DATA__
#define __RANGE__SENSOR__DATA__

#include "robotTypes.h"
#include "ConfigFile.h"

/**
* @brief Implements the data returned by a range sensor like a laser
*
*/
class rangeSensorData{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:

	int numSensors;
	float devError;
	float gammamax;
	float aperture;
	float maxDist;
	float minDist;
	float* data;
	pose* sensorPos;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:

	/// Default constructor
	rangeSensorData();
	/// constructor from config file
	rangeSensorData(int numSensors, float devError, float gammamax, float aperture,	float maxDist, float minDist);
	/// Copy constructor
	rangeSensorData(const rangeSensorData&);
	/// Default destructor
	virtual ~rangeSensorData();
	
	/// assignment operator
	rangeSensorData& operator= (const rangeSensorData &rsd);  

	/// initializer
	void initialize(int numSensors, float devError, float gammamax, float aperture,	float maxDist, float minDist);

	/// returns the number of measurements
	int getNumSensors() const;
	/// returns the error of the measurement device
	float getDevError() const;
	/// returns aperture of the cone of measure
	float getAperture() const;
	/// returns the maximum range of the sensor
	float getMaxAngle() const;
	/// returns the maximum range of the sensor
	float getMaxDist() const;
	/// returns the minimum range of the sensor
	float getMinDist() const;
	/// returns the pose of a sensor s in robot coordinates
	pose& getSensorPose(int s) const;
	/// returns a sensor measurement
	float getSensorValue(int s) const;

	/// sets the pose of a sensor s in robot coordinates
	void setSensorPose(int s, const pose& pos);
	/// sets a sensor measurement
	void setSensorValue(int s, float val);

	/// returns a pointer to the array of sensor poses
	pose* getSensorPoses();
	/// returns a pointer to the array of data
	float* getData();

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline int rangeSensorData::getNumSensors() const					{return numSensors;}
inline float rangeSensorData::getDevError() const					{return devError;}
inline float rangeSensorData::getMaxAngle() const					{return gammamax;}
inline float rangeSensorData::getAperture() const					{return aperture;}
inline float rangeSensorData::getMaxDist() const					{return maxDist;}
inline float rangeSensorData::getMinDist() const					{return minDist;}	
inline pose& rangeSensorData::getSensorPose(int s) const			{return (s<numSensors)? sensorPos[s]: nullpose;}
inline float rangeSensorData::getSensorValue(int s) const			{return data[s];}
inline void rangeSensorData::setSensorPose(int s, const pose& pos)	{if (s<numSensors) sensorPos[s] = pos;}
inline void rangeSensorData::setSensorValue(int s, float val)		{data[s] = val;}
inline pose* rangeSensorData::getSensorPoses()						{return sensorPos;}
inline float* rangeSensorData::getData()							{return data;}

#endif

