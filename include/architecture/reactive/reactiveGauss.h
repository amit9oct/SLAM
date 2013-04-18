
#ifndef __REACTIVE___GAUSS___
#define __REACTIVE___GAUSS___

#include "binMap.h"
#include "gridMapInterface.h"
#include "localPotentialField.h"
#include "reactive.h"
#include "ConfigFile.h"

/**
* @brief Reactive behaviours modelled as a sum of gaussians
*
*/

class reactiveGauss : public reactive{

private:

	pose currentPos;
	bool useesz;
	int eszdilationradius;
	int actionradius;

	int setup();
	void onStop();
	void execute();
	ClMutex closing;
	bool endReactive;

	point goal;

	const int lzwidth;
	const int lzheight;
	
	localPotentialField total;
	speed velo;

	pointf update ();

	void processOMap(point& cell, gridMapInterface& omap, binMap& esz, localPotentialField& avObs, localPotentialField& goFro, localPotentialField& goUZ);
	void processGoPre(point& poscell, binMap& ppmap, binMap&  esz, localPotentialField& goPre);
	void processGoGoal(point& pos, point& goal, binMap& esz, localPotentialField& goGoal);
	void processAvRob(int numrobots, point* robotcells, binMap& esz, localPotentialField& avRob);

	void updateField (localPotentialField& global);
	
public:

	reactiveGauss(const ConfigFile& conf);
	virtual ~reactiveGauss();

	void setGoal(point goal);


};

#endif
