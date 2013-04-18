#include "rangeSensorData.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "matFuns.h"

rangeSensorData::rangeSensorData():
	numSensors(0),
	devError(0),
	gammamax(0),
	aperture(0),
	maxDist(0),
	minDist(0),
	data(0),
	sensorPos(0)
{

}

rangeSensorData::rangeSensorData(int ns, float de, float gmax, float ap, float Maxd, float mind):
	numSensors(ns),
	devError(de),
	gammamax(gmax),
	aperture(ap),
	maxDist(Maxd),
	minDist(mind),
	data(new float[numSensors]),
	sensorPos(new pose[numSensors]())
{
	for (int j=0; j<numSensors; j++){
		data[j]=0.0f;
		sensorPos[j].x = 0.0f;
		sensorPos[j].y = 0.0f;
		sensorPos[j].th = j*2*gammamax/(numSensors-1)-gammamax;
	}
}

void rangeSensorData::initialize(int ns, float de, float gmax, float ap, float Maxd, float mind){

	numSensors = ns;
	devError = de;
	gammamax = gmax;
	aperture = ap;
	maxDist=Maxd;
	minDist=mind;
	if (data) delete[] data;
	data = new float[numSensors];
	if (sensorPos) delete[] sensorPos;
	sensorPos = new pose[numSensors]();

	for (int j=0; j<numSensors; j++){
		data[j]=0.0f;
		sensorPos[j].x = 0.0f;
		sensorPos[j].y = 0.0f;
		sensorPos[j].th = j*2*gammamax/(numSensors-1)-gammamax;
	}
}


rangeSensorData::rangeSensorData(const rangeSensorData& rsd):
	numSensors(rsd.numSensors),
	devError(rsd.devError),
	gammamax(rsd.gammamax),
	aperture(rsd.aperture),
	maxDist(rsd.maxDist),
	minDist(rsd.minDist),
	data(new float[numSensors]),
	sensorPos(new pose[numSensors]())
{
	memcpy(data,rsd.data,numSensors*sizeof(float));
	memcpy(sensorPos,rsd.sensorPos,numSensors*sizeof(pose));
}

rangeSensorData::~rangeSensorData(){
	if (data) delete[] data;
	if (sensorPos) delete[] sensorPos;
}

rangeSensorData& rangeSensorData::operator= (const rangeSensorData &rsd){
	if (numSensors != rsd.numSensors){
		if (data) delete[] data;
		data = new float[rsd.numSensors];
		if (sensorPos) delete[] sensorPos;
		sensorPos = new pose[rsd.numSensors]();
	}
	numSensors = rsd.numSensors;
	devError = rsd.devError;
	gammamax = rsd.gammamax;
	aperture = rsd.aperture;
	maxDist = rsd.maxDist;
	minDist = rsd.minDist;
	memcpy(data,rsd.data,numSensors*sizeof(float));
	memcpy(sensorPos,rsd.sensorPos,numSensors*sizeof(pose));
	
	// return the existing object  
    return *this; 
}



