/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class binMap
*
* Implements a binary gridmap
*
*/

#pragma once
#ifndef __BIN_MAP__
#define __BIN_MAP__

#include "robotTypes.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <bitset>
#include <list>

/** 
* @brief Implements a binary gridmap
*
*/
class binMap{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////
private:

	unsigned short width;					
	unsigned short height;
	unsigned short totalsize;
	float xorigin;							// Relation between discrete and real axis
	float yorigin;
	float resolution;
	RoI roi;								// Region of interest for full map operations
	std::vector< std::bitset<32> > cells;	// grid data (row by row)

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:

	/// Default Constructor
	binMap();
	/// Copy Constructor
	binMap(const binMap &map);
	/// Constructor with size in pixels
	binMap(int w, int h);
	/// Constructor with size in pixels and related to real coordinates
	binMap(int w, int h, float res, float x, float y);
	/// Constructor with size in meters and related to real coordinates
	binMap(float w, float h, float res, float x, float y);
	/// Destructor
	virtual ~binMap();
	
	/// Initializes with size in pixels
	void initialize(int w, int h);
	/// Initializes with size in pixels and related to real coordinates
	void initialize(int w, int h, float res, float x, float y);
	/// Initializes with size in meters and related to real coordinates
	void initialize(float w, float h, float res, float x, float y);

	/// assigment operator
	binMap& operator=(const binMap&);

	/// get pixel value
	bool get(int x, int y) const;
	/// set pixel value
	void set(int x, int y, bool value);
	/// set cells from cluster cell
	void set(const std::vector<clusterCell>& cl);

	/// checks if all cells in a mask of defined size is true 
	bool isInside(int x, int y, int size) const; 
	/// checks if a 1px diamant over that cell is inside the binmap
	bool isInsideD(int x, int y) const; 
	/// Checks if at least one cell in a mask of defined size is true
	bool isOver(int x, int y, int size) const;

	/// sets the x real coordinates of pixel(0,0)
	void setXOrigin(float xori);
	/// sets the y real coordinates of pixel(0,0)
	void setYOrigin(float yori);
	/// sets the grid map resolution in meters
	void setResolution(float res);

	/// returns the width of the grid map
	int getWidth() const;
	/// returns the height of the grid map
	int getHeight() const;
	/// returns the x real coordinates of pixel(0,0)
	float getXOrigin() const;
	/// returns the y real coordinates of pixel(0,0)
	float getYOrigin() const;
	/// returns the grid map resolution in meters
	float getResolution() const;

	/// subtraction cell to cell 
	void sub(const binMap& map);
	/// logic or cell to cell 
	void add(const binMap& map);
	/// logic and cell to cell 
	void times(const binMap& map);
	/// logic and cell to cell 
	void invert();
	/// clears the binMap
	void clear();
	/// computer vision opening function
	void opening(int rad);
	/// computer vision closing function
	void closing(int rad);
	/// computer vision dilation function
	void dilate(int rad);
	/// computer vision erosion function
	void erode(int rad);

	/// removes unconnected pixel from one point
	void removeUnconnected(int x, int y);

	/// Sets the region of interest 
	void setRoi(const RoI& roi);
	/// Returns the region of interest
	const RoI& getRoi() const;

	/// Draws a line
	void line(int x1, int y1, int x2, int y2);

	/// Save function
	void saveMapAsImage(const char* file) const;

	/// Show binMap
	void showMap(const char* windowname)const;

	/// Get the map as a new opencv image
	IplImage* getMapAsImage() const;

	/// Adds to the vector the coordinates of all the cells that have a 1 value. 
	int getPositives(std::vector<point>& positives) const;

	/// Recursive method to find clusters of connected components in the binMap
	int cluster(std::vector<clusterCell>& clusterList, int minSize) const;

	/// Counts the number of 1 values
	int count() const;

private:
	/// auxiliary method to remove unconnected pixels
	void remUnconnectRec(int x, int y, const binMap& prevData);
 	/// auxiliary method of the cluster function
	bool clustering(const int &i, const int &j, binMap& aux, clusterCell& cl, std::list<point> &seq, bool backfront) const;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

inline void binMap::setXOrigin(float xori)		{xorigin = xori;}
inline void binMap::setYOrigin(float yori)		{yorigin = yori;}
inline void binMap::setResolution(float res)		{resolution = res;}
inline int binMap::getWidth() const			{return width;}
inline int binMap::getHeight() const			{return height;}
inline float binMap::getXOrigin() const			{return xorigin;}
inline float binMap::getYOrigin() const			{return yorigin;}
inline float binMap::getResolution() const		{return resolution;}
inline const RoI& binMap::getRoi() const		{return roi;}
inline void binMap::setRoi(const RoI& r)		{this->roi = r;}

inline bool binMap::get(int x, int y) const{
	int idx = y*width+x;
	div_t idx32 = div (idx, 32);
	return (idx32.quot>=0 && idx32.quot<totalsize) ? cells[idx32.quot][idx32.rem] : false;
}

inline void binMap::set(int x, int y, bool value){
	int idx = y*width+x;
	div_t idx32 = div (idx, 32);
	if (idx32.quot>=0 && idx32.quot<totalsize && idx32.rem >=0 && idx32.rem<32) cells[idx32.quot].set(idx32.rem,value);
}

#endif
