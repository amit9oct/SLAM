/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class particle
*
*/

#ifndef __SLAM__PARTICLE__
#define __SLAM__PARTICLE__

#include "visualMap.h"
#include "occupancyGridMap.h"
#include "robotBase.h"
#include "ConfigFile.h"

/**
*
* @brief Implements a particle in a rao-blackwellized particle filter for visual slam
*
*/
class particle{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////

private:
	int nrobots;

	unsigned short width;
	unsigned short height;
	float realWidth;
	float realHeight;
	float xorigin;		// Relation between discrete and real axis
	float yorigin;
	float resolution;

	double weight;
	double sumLogWeight;

	pose* rpos;
	point* rcell;
	
	visualMap* vMap;

	gridMapInterface* oMap;
	binMap* precisePoseMap;
	binMap* imprecisePoseMap;

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////

public:
	
	/// Default Constructor
	particle();
	/// Constructor
	particle(int nRobots, float w, float h, float res, float x, float y, int gmtype, int vmtype,  int nmarks, const ConfigFile& conf);
	/// Copy Constructor
	particle(const particle&);
	/// Default Destructor
	virtual ~particle();

	/// initialization
	void initialize(int nRobots, float w, float h, float res, float x, float y, int gmtype, int vmtype, int nmarks, const ConfigFile& conf);
	
	particle& operator=(const particle&);

	/// returns the number of robots
	int getNumRobots();

	/// returns the pose of the robot
	const pose& getPos(int robot);
	/// sets the pose of a robot
	void setPos(const pose& pos, int robot);
	/// returns a pointer to the data array of poses 
	pose* getDataPos();

	/// returns the cell where the robot is located
	const point& getCell(int robot);
	/// sets the cell where the robot is located
	void setCell(const point& point, int robot);
	/// returns a pointer to the data array of cells 
	point* getDataCell();
	
	/// returns the visual map of landmarks
	visualMap& getVMap();
	/// returns the occupancy grid map
	gridMapInterface& getOMap();
	/// returns a binary map of past poses with low dispersion
	binMap& getPrecisePoseMap();
	/// returns a binary map of past poses with high dispersion
	binMap& getImprecisePoseMap();

	/// sets the weight of the particle
	void setWeight(double w);
	/// returns the weight of the particle
	double getWeight();

	/// sets the weight of the particle
	void setSumLogWeight(double w);
	/// returns the weight of the particle
	double getSumLogWeight();

	void releaseVMap();

};

inline int particle::getNumRobots()								{return nrobots;}
inline const pose& particle::getPos(int robot)					{return (rpos[robot]);}
inline pose* particle::getDataPos()								{return rpos;}
inline const point& particle::getCell(int robot)				{return (rcell[robot]);}
inline point* particle::getDataCell()							{return rcell;}
inline void particle::setCell(const point& point, int robot)	{rcell[robot].x = point.x;rcell[robot].y = point.y;}
inline visualMap& particle::getVMap()							{return *vMap;}
inline gridMapInterface& particle::getOMap()					{return *oMap;}
inline binMap& particle::getPrecisePoseMap()					{return *precisePoseMap;}
inline binMap& particle::getImprecisePoseMap()					{return *imprecisePoseMap;}
inline void particle::setWeight(double w)						{weight = w;}
inline double particle::getWeight()								{return weight;}
inline void particle::setSumLogWeight(double w)					{sumLogWeight = w;}
inline double particle::getSumLogWeight()						{return sumLogWeight;}
inline void particle::releaseVMap()								{vMap->clear();}


#endif


