/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class particle
*
* Implements a particle in a rao-blackwellized particle filter for visual slam
*
*
*
*/

#include "particle.h"
#include <stdio.h>
#include <math.h>
#include "gridMapInterface.h"

// Constructors
particle::particle():
	nrobots(0),
	width(0),
	height(0),
	realWidth(0),
	realHeight(0),
	xorigin(0),
	yorigin(0),
	resolution(0),
	rpos(0),
	rcell(0),
	vMap(0),
	oMap(0),
	precisePoseMap(0),
	imprecisePoseMap(0)
{

}

particle::particle(int nRobots, float w, float h, float res, float x, float y, int gmtype, int vmtype, int nmarks, const ConfigFile& conf):
	nrobots(nRobots),
	width((int)floor(w/res+0.5)),
	height((int)floor(h/res+0.5)),
	realWidth(w),
	realHeight(h),
	xorigin(x),
	yorigin(y),
	resolution(res),
	weight(1.0),
	sumLogWeight(0.0),
	rpos(new pose[nRobots]()),
	rcell(new point[nRobots]()),
	vMap(visualMapFactory::Instance().CreateObject(vmtype,nmarks,conf)),
	oMap(0),
	precisePoseMap(0),
	imprecisePoseMap(0)
{
	if (gmtype >=0){
		oMap = gridMapFactory::Instance().CreateObject(gmtype, w, h, res, x, y);
		precisePoseMap = new binMap(width, height, res, x, y);
		imprecisePoseMap = new binMap(width, height, res, x, y);
	}
}

particle::particle(const particle &part):
	nrobots			(part.nrobots),
	width			(part.width),
	height			(part.height),
	realWidth		(part.realWidth),
	realHeight		(part.realHeight),
	xorigin			(part.xorigin),
	yorigin			(part.yorigin),
	resolution		(part.resolution),
	weight			(part.weight),
	sumLogWeight	(part.sumLogWeight),
	rpos			(new pose[part.nrobots]()),
	rcell			(new point[part.nrobots]()),
	vMap			(part.vMap->clone()),
	oMap			(0),
	precisePoseMap	(0),
	imprecisePoseMap(0)
{
	if (part.oMap) oMap = part.oMap->clone();
	if (part.precisePoseMap) precisePoseMap = new binMap(*(part.precisePoseMap));
	if (part.imprecisePoseMap) imprecisePoseMap = new binMap(*(part.imprecisePoseMap));
	memcpy(rpos,part.rpos,nrobots*sizeof(pose));
	memcpy(rcell,part.rcell,nrobots*sizeof(point));
}

particle& particle::operator=(const particle& p){
	nrobots = p.nrobots;
	weight = p.weight;
	sumLogWeight = p.sumLogWeight;
	
	width = p.width;
	height = p.height;
	realWidth = p.realWidth;
	realHeight = p.realHeight;
	xorigin = p.xorigin;
	yorigin = p.yorigin;
	resolution = p.resolution;

	if (rpos) delete[] rpos;
	rpos = new pose[nrobots]();
	memcpy(rpos,p.rpos,nrobots*sizeof(pose));

	if (rcell) delete[] rcell;
	rcell = new point[nrobots]();
	memcpy(rcell,p.rcell,nrobots*sizeof(point));

	if (vMap) *vMap = *p.vMap;
	else vMap = p.vMap->clone();

	if (p.oMap){
		if (oMap) *oMap = *p.oMap;
		else oMap = p.oMap->clone();
	}
	if (p.precisePoseMap){
		if (precisePoseMap) *precisePoseMap = *p.precisePoseMap;
		else precisePoseMap = new binMap(*(p.precisePoseMap));
	}
	if (p.imprecisePoseMap){
		if (imprecisePoseMap) *imprecisePoseMap = *p.imprecisePoseMap;
		else imprecisePoseMap = new binMap(*(p.imprecisePoseMap));
	}

	return *this;
}

particle::~particle(){
	if (rpos) delete[] rpos;
	if (rcell) delete[] rcell;
	if (oMap) delete oMap;
	if (vMap) delete vMap;
	if (precisePoseMap) delete precisePoseMap;
	if (imprecisePoseMap) delete imprecisePoseMap;
}

// Initializers
void particle::initialize(int r, float w, float h, float res, float x, float y, int gmtype, int vmtype, int nmarks, const ConfigFile& conf){
	width		= (int)floor(w/res+0.5);
	height		= (int)floor(h/res+0.5);
	realWidth	= w;
	realHeight	= h;
	xorigin		= x;
	yorigin		= y;
	resolution	= res;
	if (rpos) delete[] rpos;
	rpos =  new pose[r]();
	if (rcell) delete[] rcell;
	rcell = new point[r]();
	if (vMap) delete vMap;
	vMap = visualMapFactory::Instance().CreateObject(vmtype,nmarks,conf);

	if (gmtype >=0){
		if (oMap) delete oMap;
		oMap = gridMapFactory::Instance().CreateObject(gmtype, w, h, res, x, y);
		if (precisePoseMap) delete precisePoseMap;
		precisePoseMap = new binMap(width, height, res, x, y);
		if (imprecisePoseMap) delete imprecisePoseMap;
		imprecisePoseMap = new binMap(width, height, res, x, y);
	}
	nrobots = r;
	weight = 1.0;
	sumLogWeight = 0.0;
}

// falta calcular la celda nueva
void particle::setPos(const pose& pos, int robot){

	rpos[robot].x = pos.x;
	rpos[robot].y = pos.y;
	rpos[robot].th = pos.th;

	rcell[robot].x = (int) floor((pos.x - xorigin)/resolution);
	rcell[robot].y = (int) floor((pos.y - yorigin)/resolution);
}
