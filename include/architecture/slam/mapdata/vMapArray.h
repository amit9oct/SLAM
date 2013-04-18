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
#ifndef __VISUAL_MAP__ARRAY__IMPLEMENTATION__
#define __VISUAL_MAP__ARRAY__IMPLEMENTATION__

#include <string.h>
#include <opencv2/opencv.hpp>
#include "landmarksData.h"
#include "matFuns.h"
#include "ConfigFile.h"
#include "visualMap.h"

/**
* @brief Implements a visual landmark 3d map using a simple array of features
*
*/
class vMapArray: public visualMap{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:

	unsigned short nMarksReserved;
	int lastRetId;
	landmark* landmarks;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:
	
	/// default constructor
	vMapArray();
	/// constructor
	vMapArray(int n, const ConfigFile& config);
	/// copy constructor
	vMapArray(const vMapArray&);
	/// destructor
	virtual ~vMapArray();
	
	/// initializer
	void initialize(int n, const ConfigFile& config);
	/// assignment operator
	vMapArray& operator=(const visualMap&);
	/// clone method
	vMapArray* clone() const;

	/// Data functions
	void addLandmark(const landmark& mark);

	/// data association, return a NEW copy of the matched mark in the map, the returned mark must be DELETE
	landmark* dataAssociation(const matrix& ZT, const matrix& Rt, const pos3d& globalpos, const pose& robotPos, const matrix& H, const matrix& Htrans, const float* desc) ;
	/// return return a NEW copy of the landmark with descriptor desc, the returned mark must be DELETE
	landmark* getLandmarkByDesc(const float* desc) ;
	/// return return a NEW copy of the landmark with id number, the returned mark must be DELETE
	landmark* getLandmarkById(int number);
	/// changes a landmark 
	void changeLandmark(int id, const landmark& mark);
	/// changes a landmark in the kdtree
	void changeLastReturnedLandmark(const landmark& mark);

	/// clear the map
	void clear();
	/// saves the map
	void saveMap(const char* str);
	/// draw the features in an opencv image
	void drawMarks (IplImage& im, float xorigin, float yorigin, float resolution) ;

	/// return the last returned id
	int returnLastReturnedId() const;


//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline int vMapArray::returnLastReturnedId() const	{return lastRetId;}
inline vMapArray* vMapArray::clone() const			{return new vMapArray(*this);};

#endif
