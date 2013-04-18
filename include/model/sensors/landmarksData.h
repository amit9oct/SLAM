/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class landmarksData
*
*
*/
#pragma once
#ifndef __LANDMARKS__DATA__
#define __LANDMARKS__DATA__

#include "matFuns.h"

/// Landmark struct
typedef struct landmark{
	pos3d pos;
	matrix covariance;
	float* descriptor;
	unsigned short desclength;
	unsigned short repeated;

	landmark(): descriptor(0), desclength(0){};
	//landmark():pos(0.0f,0.0f,0.0f), sigma(3,3), repeated(0){};
	landmark(const pos3d& p, const matrix& s, const float* desc, unsigned short length, unsigned short r):
		pos(p),covariance(s),desclength(length),repeated(r)
	{
		descriptor = new float[desclength];
		memcpy(descriptor,desc,desclength*sizeof(float));
	}
	landmark(const landmark& l): pos(l.pos),covariance(l.covariance),desclength(l.desclength),repeated(l.repeated){
		descriptor = new float[desclength];
		memcpy(descriptor,l.descriptor,desclength*sizeof(float));
	}
	landmark& operator=(const landmark& l){
		pos = l.pos;
		covariance = l.covariance;
		repeated = l.repeated;
		if (desclength != l.desclength){
			desclength = l.desclength;
			if (descriptor) delete[] descriptor;
			descriptor = new float[desclength];	
		}
		memcpy(descriptor,l.descriptor,desclength*sizeof(float));
		return *this;
	}
	virtual ~landmark(){ if (descriptor) delete[] descriptor;}
}landmark;

static landmark nulllandmark;

/**
* @brief Implements an array of landmarks returned by a sensor
*/
class landmarksData{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:
	unsigned short nLandmarks;
	unsigned short marksReserved;
	landmark* landmarks;

	float distMAX;
	float distMIN;
	float gammaMAX;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:
	
	/// Default Constructor
	landmarksData();
	/// Builds the structure reserving space for nLandmarks
	landmarksData(int nLandmarks);
	/// Copy constructor
	landmarksData(const landmarksData&);
	/// Default destructor
	~landmarksData();
	
	/// asignment operator
	landmarksData& operator= (const landmarksData &lmd); 

	/// sets the landmark n of the array
	void addLandmark(const landmark& lm);
	/// returns the landmark n of the array
	landmark& getLandmark(int n) const;
	/// returns a pointer to the data
	landmark* getLandmarks();
	/// clear the data
	void clear();

	/// returns the number of landmarks
	int getNLandmarks() const;
	/// resize the array to nLandmarks (this operation deletes the previous data)
	void reserve(int nLandmarks);

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline void landmarksData::addLandmark(const landmark& lm)				{if (nLandmarks<marksReserved){landmarks[nLandmarks]=lm;nLandmarks++;}}
inline landmark& landmarksData::getLandmark(int n) const				{return (n<nLandmarks)? landmarks[n]: nulllandmark;}
inline landmark* landmarksData::getLandmarks()							{return landmarks;}
inline int landmarksData::getNLandmarks() const							{return nLandmarks;}
inline void landmarksData::clear()										{nLandmarks = 0;}
#endif

