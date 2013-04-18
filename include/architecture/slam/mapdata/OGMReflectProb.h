/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class OGMReflectProb
*
*
*/

#pragma once

#ifndef __MORAVEC__AND__ELFES_85__
#define __MORAVEC__AND__ELFES_85__

#include "binMap.h"
#include "robotTypes.h"
#include "occupancyGridMap.h"

///represents the occupation probability of a cell in a grid occupancy map
typedef struct cell{
	float occ;
	float total;
	float val;
	//float val2;
	cell():occ(0),total(0),val(50.0f)//,val2(50.0f)
	{};
}cell;

static cell nullcell;

/**
* @brief Implements an ocupancy grid map using a counting model
*/

class OGMReflectProb: public occupancyGridMap{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////



private:

	int totalsize;
	cell* cells;		// Data cell array
	float step;
	
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  --------------------------------
///////////////////////////////////////////////////////////////////////

public:
	/// Default Constructor
	OGMReflectProb();
	/// Constructor with size in meters and related to real coordinates
	OGMReflectProb(float w, float h, float res, float x, float y);
	/// Destructor
	virtual ~OGMReflectProb();
	
	/// Initializes with size in meters and related to real coordinates
	void initialize(float w, float h, float res, float x, float y);
	/// clone method
	OGMReflectProb* clone() const;
	
	/// assigment operator
	gridMapInterface& operator=(const gridMapInterface&);
	
	/// return the occupancy probability of the cell
	float getValue(int x, int y) const;
	/// Resets the occupation probability for the total map
	void reset();
	
	/// Updates the occupancy grid using the new data
	void update(const rangeSensorData& rsData, const pose& rpos, float disp=0);
	
	/// save a map to disk
	int saveMapToFile(const char* file) const;
	/// load a map from disk
	int loadMapFromFile(const char* file);

private:

	/// copy constructor
	OGMReflectProb(const OGMReflectProb&);
	/// Returns the occupation probability for a cell 
	cell& get(int x, int y) ;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline float OGMReflectProb::getValue(int x, int y) const{
	int idx(y*width+x);
	return (idx >=0 && idx < totalsize)? cells[idx].val : 50.0f;
};

inline cell& OGMReflectProb::get(int x, int y) {
	int idx(y*width+x);
	return (idx >=0 && idx < totalsize)? cells[idx]: nullcell;
};

inline OGMReflectProb* OGMReflectProb::clone() const {return new OGMReflectProb(*this);};

#endif
