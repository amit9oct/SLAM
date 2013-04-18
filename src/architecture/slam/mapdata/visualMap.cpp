/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class visualMap
*
* Implements a visual landmark 3d map
*
*/

#include "visualMap.h"
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "matFuns.h"

// Constructor
visualMap::visualMap():
	nlandmarks		(0),
	camx			(0.0f),
	camy			(0.0f),
	camz			(0.0f),
	rx			(0.0f),
	ry			(0.0f),
	rz			(0.0f),
//	range			(0.0f),
	mahTh			(0.0f),
	descTh			(0.0f)
{

}

// Constructor
visualMap::visualMap(int n, const ConfigFile& configFile):
	nlandmarks		(0),
	camx			(configFile.read<float>("camx")),
	camy			(configFile.read<float>("camy")),
	camz			(configFile.read<float>("camz")),
	rx			(configFile.read<float>("rx")),
	ry			(configFile.read<float>("ry")),
	rz			(configFile.read<float>("rz")),
//	range			(configFile.read<float>("SEARCHRANGE")),
	mahTh			(configFile.read<float>("MAHALANOBISTH")),
	descTh			(configFile.read<float>("DESCRIPTORTH"))
{

}

// copy constructor
visualMap::visualMap(const visualMap &map):
	nlandmarks		(map.nlandmarks),
	camx			(map.camx),
	camy			(map.camy),
	camz			(map.camz),
	rx			(map.rx),
	ry			(map.ry),
	rz			(map.rz),
//	range			(map.range),
	mahTh			(map.mahTh),
	descTh			(map.descTh)
{

}


// destructor
visualMap::~visualMap(){

}

// initializer
void visualMap::initialize(int n, const ConfigFile& configFile){
	nlandmarks		= 0;
	camx			= configFile.read<double>("camx");
	camy			= configFile.read<double>("camy");
	camz			= configFile.read<double>("camz");
	rx			= configFile.read<double>("rx");
	ry			= configFile.read<double>("ry");
	rz			= configFile.read<double>("rz");
//	range			= configFile.read<float>("SEARCHRANGE");
	mahTh			= configFile.read<float>("MAHALANOBISTH");
	descTh			= configFile.read<float>("DESCRIPTORTH");
}

// assignment operator
visualMap& visualMap::operator=(const visualMap& map){
	nlandmarks		= map.nlandmarks;
	camx			= map.camx;
	camy			= map.camy;
	camz			= map.camz;
	rx			= map.rx;
	ry			= map.ry;
	rz			= map.rz;
//	range			= map.range;
	mahTh			= map.mahTh;
	descTh			= map.descTh;
	return *this;
}


// distance between descriptors
float visualMap::descDist(const float* desc1, const float* desc2, int desclength){
	float dist_sq=0, tmp_val;
	for (int i = 0; i< desclength; i++){
		tmp_val = desc1[i] - desc2[i];
		dist_sq += tmp_val*tmp_val;
	}
	return sqrt(dist_sq);
}

// mahalanobis distance
float visualMap::mahalanobis(const pos3d& pos, const matrix& sigma){
//	float det = (sigma(0,0)*sigma(1,1)-sigma(1,0)*sigma(0,1));
//	return( (( (sigma(1,1)*pos.getX()-pos.getY()*sigma(0,1))*pos.getX() + (-pos.getX()*sigma(1,0)+sigma(0,0)*pos.getY())*pos.getY())/det + 1/(sigma(2,2))*pos.getZ()*pos.getZ() ));
	matrix x = pos;
	return (x.transpose()*((sigma.inverse())*x)).get(0,0);
}


// translates a position in the robot's frame of reference to the camera frame of reference
pos3d visualMap::base2cam(const pos3d& p) const{
	return pos3d(p.getX() - camx,
				 p.getY() - camy,
				 p.getZ() - camz);
}

// TODO: use calibration rx, ry, rz
// translates a position in the camera frame of reference to the robot's frame of reference
pos3d visualMap::cam2base(const pos3d& p) const{
	return pos3d(p.getX()+ camx,
				 p.getY()+ camy,
				 p.getZ()+ camz);
}
