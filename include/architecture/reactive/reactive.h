#ifndef __REACTIVE__INTERFACE___
#define __REACTIVE__INTERFACE___

#include "ClThread.h"
#include "robotTypes.h"
#include "slamInterface.h"
#include "localPotentialField.h"
#include "robotBase.h"
#include "ConfigFile.h"


/**
* @brief Reactive behaviours interface and common functions
*
*/

class reactive: public ClThread{

protected:

	slamInterface* mySlam;
	robotBase* rbase;
	int number;

	const float vmax;
	const float wmax;
	const float k1;
	const float k2;

	float resolution;

	bool avoidObstaclesEnabled;
	bool goToFrontierEnabled;
	bool goToUnexploredZonesEnabled;
	bool avoidOtherRobotsEnabled;
	bool goToPrecisePosesEnabled;
	bool goToGoalEnabled;

	float avoidObstaclesWeight;
	float goToFrontierWeight;
	float goToUnexploredZonesWeight;
	float avoidOtherRobotsWeight;
	float goToPrecisePosesWeight;
	float goToImprecisePosesWeight;
	float goToGoalWeight;

	float avoidObstaclesWidth;
	float goToFrontierWidth;
	float goToUnexploredZonesWidth;
	float avoidOtherRobotsWidth;
	float goToPrecisePosesWidth;
	float goToGoalWidth;

	float avoidObstaclesWidth_cells;
	float goToFrontierWidth_cells;
	float goToUnexploredZonesWidth_cells;
	float avoidOtherRobotsWidth_cells;
	float goToPrecisePosesWidth_cells;
	float goToGoalWidth_cells;

	bool localMinimumDetected;
	pointf getResponse(localPotentialField& lpf);

public:

	reactive(const ConfigFile& file);
	virtual ~reactive(){};

	speed regulator(const pose& pos, pointf& f);

	void enableAvoidObstacles();
	void enableGoToFrontier();
	void enableGoToUnexploredZones();
	void enableAvoidOtherRobots();
	void enableGoToPrecisePoses();
	void enableGoToGoal();

	void disableAvoidObstacles();
	void disableGoToFrontier();
	void disableGoToUnexploredZones();
	void disableAvoidOtherRobots();
	void disableGoToPrecisePoses();
	void disableGoToGoal();

	void setWeightAvoidObstacles(float weight);
	void setWeightGoToFrontier(float weight);
	void setWeightGoToUnexploredZones(float weight);
	void setWeightAvoidOtherRobots(float weight);
	void setWeightGoToPrecisePoses(float weight);
	void setWeightGoToGoal(float weight);

	void setWidthAvoidObstacles(float width);
	void setWidthGoToFrontier(float width);
	void setWidthGoToUnexploredZones(float width);
	void setWidthAvoidOtherRobots(float width);
	void setWidthGoToPrecisePoses(float width);
	void setWidthGoToGoal(float width);

	void disableAll();

	bool localMinimum();

	virtual void setGoal(point goal){};

	void setSlam(slamInterface& slamProc);
	void setRBase(robotBase& rb);
	void setRobot(int number);
};


inline void reactive::enableAvoidObstacles()						{avoidObstaclesEnabled=true;}
inline void reactive::enableGoToFrontier()						{goToFrontierEnabled=true;}
inline void reactive::enableGoToUnexploredZones()					{goToUnexploredZonesEnabled=true;}
inline void reactive::enableAvoidOtherRobots()						{avoidOtherRobotsEnabled=true;}
inline void reactive::enableGoToPrecisePoses()						{goToPrecisePosesEnabled=true;}
inline void reactive::enableGoToGoal()							{goToGoalEnabled=true;}

inline void reactive::disableAvoidObstacles()						{avoidObstaclesEnabled=false;}
inline void reactive::disableGoToFrontier()						{goToFrontierEnabled=false;}
inline void reactive::disableGoToUnexploredZones()					{goToUnexploredZonesEnabled=false;}
inline void reactive::disableAvoidOtherRobots()						{avoidOtherRobotsEnabled=false;}
inline void reactive::disableGoToPrecisePoses()						{goToPrecisePosesEnabled=false;}
inline void reactive::disableGoToGoal()							{goToGoalEnabled=false;}

inline void reactive::setWeightAvoidObstacles(float weight)				{avoidObstaclesWeight = weight;};
inline void reactive::setWeightGoToFrontier(float weight)				{goToFrontierWeight = weight;};
inline void reactive::setWeightGoToUnexploredZones(float weight)			{goToUnexploredZonesWeight = weight;};
inline void reactive::setWeightAvoidOtherRobots(float weight)				{avoidOtherRobotsWeight = weight;};
inline void reactive::setWeightGoToPrecisePoses(float weight)				{goToPrecisePosesWeight = weight;};
inline void reactive::setWeightGoToGoal(float weight)					{goToGoalWeight = weight;};

inline void reactive::setWidthAvoidObstacles(float width)				{avoidObstaclesWidth = width;};
inline void reactive::setWidthGoToFrontier(float width)					{goToFrontierWidth = width;};
inline void reactive::setWidthGoToUnexploredZones(float width)				{goToUnexploredZonesWidth = width;};
inline void reactive::setWidthAvoidOtherRobots(float width)				{avoidOtherRobotsWidth = width;};
inline void reactive::setWidthGoToPrecisePoses(float width)				{goToPrecisePosesWidth = width;};
inline void reactive::setWidthGoToGoal(float width)					{goToGoalWidth = width;};

inline void reactive::setSlam(slamInterface& slamProc)					{
	mySlam = &slamProc;
	resolution = mySlam->getResolution();
	avoidObstaclesWidth_cells = avoidObstaclesWidth/resolution;
	goToFrontierWidth_cells = goToFrontierWidth/resolution;
	goToUnexploredZonesWidth_cells = goToUnexploredZonesWidth/resolution;
	avoidOtherRobotsWidth_cells = avoidOtherRobotsWidth/resolution;
	goToPrecisePosesWidth_cells = goToPrecisePosesWidth/resolution;
	goToGoalWidth_cells = goToGoalWidth/resolution;
}
inline void reactive::setRBase(robotBase& rb)						{rbase = &rb;}
inline void reactive::setRobot(int n)							{number = n;}
inline bool reactive::localMinimum()							{return localMinimumDetected;}


typedef Loki::Functor<reactive*, LOKI_TYPELIST_1(const ConfigFile&)> reactiveCreator;
typedef Loki::SingletonHolder< Loki::Factory< reactive, int, LOKI_TYPELIST_1(const ConfigFile&)> > reactiveFactory;

#endif
