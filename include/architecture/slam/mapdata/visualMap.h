/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class visualMap
*
*
*/

#pragma once
#ifndef __VISUAL_MAP__
#define __VISUAL_MAP__

#include <string.h>
#include <opencv2/opencv.hpp>
#include "landmarksData.h"
#include "matFuns.h"
#include "ConfigFile.h"
#include <loki/Factory.h>
#include <loki/Typelist.h>
#include <loki/Functor.h>

/**
* @brief Implements a visual landmark 3d map
*
*/
class visualMap {

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
protected:

	unsigned short nlandmarks;

	float camx;					/// camera position
	float camy;					/// camera position
	float camz;					/// camera position
	float rx;					/// camera position
	float ry;					/// camera position
	float rz;					/// camera position

//	float range;
	float mahTh;
	float descTh;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:
	
	/// default constructor
	visualMap();
	/// constructor
	visualMap(int n, const ConfigFile& config);
	/// copy constructor
	visualMap(const visualMap&);
	/// destructor
	virtual ~visualMap();
	
	/// initializer
	void initialize(int n, const ConfigFile& config);
	/// assignment operator
	virtual visualMap& operator=(const visualMap&);
	/// clone method
	virtual visualMap* clone() const=0;	

	/// Data functions
	virtual void addLandmark(const landmark& mark)=0;
	/// return the number of features in the map
	int getNLandmarks() const;
	
	/// data association, return a NEW copy of the matched mark in the map, the returned mark must be DELETE
	virtual landmark* dataAssociation(const matrix& ZT, const matrix& Rt, const pos3d& globalpos, const pose& robotPos, const matrix& H, const matrix& Htrans, const float* desc)=0;
	/// return return a NEW copy of the landmark with descriptor desc, the returned mark must be DELETE
	virtual landmark* getLandmarkByDesc(const float* desc)=0;
	/// return return a NEW copy of the landmark with id number, the returned mark must be DELETE
	virtual landmark* getLandmarkById(int number)=0;			
	/// changes a landmark 
	void changeLandmark(int id, const landmark& mark);
	/// changes a landmark 
	virtual void changeLastReturnedLandmark(const landmark& mark)=0;
	/// return the last returned id
	virtual int returnLastReturnedId() const =0;

	/// clear the map
	virtual void clear()=0;
	/// saves the map
	virtual void saveMap(const char* str)=0;
	/// draw the features in an opencv image
	virtual void drawMarks (IplImage& im, float xorigin, float yorigin, float resolution)=0;
	
	/// distance between descriptors
	static float descDist(const float* desc1, const float* desc2, int desclength);
	/// mahalanobis distance
	static float mahalanobis(const pos3d& pos, const matrix& sigma);

	/// translates a position in the camera frame of reference to the robot's frame of reference
	pos3d cam2base(const pos3d& p) const;
	/// translates a position in the robot's frame of reference to the camera frame of reference
	pos3d base2cam(const pos3d& p) const;
	
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline int visualMap::getNLandmarks() const		{return nlandmarks;}


typedef Loki::Functor<visualMap*, LOKI_TYPELIST_2(int,const ConfigFile&)> visualMapCreator;
typedef Loki::SingletonHolder< Loki::Factory< visualMap, int, LOKI_TYPELIST_2(int,const ConfigFile&)> > visualMapFactory;


#endif
