/**
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class EKF
* 
*/

#pragma once
#ifndef __EKF__
#define __EKF__
#include "vslamFilter.h"


/**
* @brief Implements a slam algorithm consisting on a extended Kalman filter
*
*/
class EKFilterVel: public vslamFilter
{

///////////////////////////////////////////////////////////////////////
//-------------------------  Attributes  ------------------------------
///////////////////////////////////////////////////////////////////////.
private:

	Ematrix state;
	Ematrix covariance;

	point* robotcells;
	float* dispRobot;				// measure of uncertainty for each robot
	pose** odoData;					// odometry values
	pose** lastOdoData;				// last odometry values
	rangeSensorData** rsData;			// captured range sensor data 
	landmarksData** lmData;				// captured features

	ClMutex closing;				// run/stop control
	bool endSlam;					// stop condition
	
	gridMapInterface* omap;				// a common occupancy grid for all particles
	binMap* ppmap;					// a common precise pose binMap for all particles
	binMap* ipmap;					// a common imprecise pose binMap for all particles
	visualMap* vmap;				// visual map			

	matrix Identity3;				// 3x3 identity matrix
	
//	float range;
	float mahTh;
	float descTh;
	int nearestNeighbourOrderBy;
	float mediana;
	double determinante;

	float maxCorrect;
	float minIncorrect;

	int errorcount;
//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
//-------------------------  Methods  ---------------------------------
///////////////////////////////////////////////////////////////////////
public:
	/// Constructor
	EKFilterVel();
	/// Destructor
	virtual ~EKFilterVel();
	/// Constructor
	EKFilterVel(int nrobots, ConfigFile& configFile);

	///initializer	
	void initialize(int nrobots, ConfigFile& configfile);

	// maps
	/// returns a copy of the current occupancy map
	gridMapInterface* getOMap();
	/// returns a copy of the current visual map
	visualMap* getVMap();
	/// return a copy of the current precise pose map
	binMap* getPreMap();
	/// returns a copy of the current imprecise pose map
	binMap* getImpMap();

	/// returns the current pose of the robot
	pose getPos(int robot);
	/// returns the current velocity of the robot
	pose getVel(int r);
	/// returns the current cell of the robot
	point getCell(int robot);
	/// returns the matrix that represents the covariance of the position of the robot
	matrix getCovariance(int robot) const;
	/// returns the matrix that represents the covariance of robots and marks
	Ematrix getGlobalCovariance() const;
	/// returns a measure of the dispersion of the robot
	float getDisp(int robot) const;
	/// returns the map as a QImage
	QImage* getQImage();
	/// returns the map as a QPixmap
	QPixmap* getPixmap();

private:

	/// set up for the execution thread
	int setup();			
	/// primary execution loop
	void execute();			
	/// called before stopping
	void onStop();	
	
	/// evaluates the uncertainty on the position of each robot
	void evalDisp(int r);
	/// evaluates the uncertainty on the position of each robot
	float evalUncertainty();

	/// odometry noise matrix Q
	Ematrix noiseQ();
	/// jacobian F
	matrix jacobianF(const pose& pos, const pose& velocity);
	/// jacobian G
	Ematrix jacobianG(const pose& pos, const pose& velocity);
	/// jacobian H1
	matrix jacobianH1(const pose& pos, const pos3d& mark);
	/// jacobian H2
	matrix jacobianH2(const pose& pos);
	/// jacobian J1
	matrix jacobianJ1(const pose& pos, const pos3d& mark);
	/// Data association between the new observations and the map
	landmark* dataAssociation(const matrix& ZT, const matrix& Rt, const pos3d& globalpos, const pose& robotPos, const matrix& H, const matrix& Htrans, const float* desc, const Ematrix& Pcomp, int r);
	/// motion model
	matrix motionModel (const pose& pos, const pose& velocity);
	/// Draws the poses of the robots in an opencv image
	void drawPose (IplImage& im, const pose& pos, const matrix& covariance, float xorigin, float yorigin, float resolution);

//---------------------------------------------------------------------	
///////////////////////////////////////////////////////////////////////
};

#endif 
