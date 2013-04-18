#include "landmarksData.h"

#include <assert.h>
#include <string.h>

landmarksData::landmarksData():
	nLandmarks(0),
	marksReserved(0),
	landmarks(0)
{

}

landmarksData::landmarksData(int nl):
	nLandmarks(0),
	marksReserved(nl),
	landmarks(new landmark[nl]())
{
	
}

landmarksData::landmarksData(const landmarksData& lmd):
	nLandmarks(lmd.nLandmarks),
	marksReserved(lmd.marksReserved),
	landmarks(new landmark[marksReserved]())
{
	for (int i = 0; i< nLandmarks; i++){
		landmarks[i] = lmd.landmarks[i];
	}
}

landmarksData::~landmarksData(){
	if (landmarks) delete[] landmarks;
}

landmarksData& landmarksData::operator= (const landmarksData &lmd){
	if (marksReserved != lmd.marksReserved){
		if (landmarks) delete[] landmarks;
		marksReserved = lmd.marksReserved;
		landmarks = new landmark[marksReserved]();
	}
	nLandmarks = lmd.nLandmarks;
	for (int i = 0; i< nLandmarks; i++)
		landmarks[i] = lmd.landmarks[i];
	
	// return the existing object  
    return *this; 
}

void landmarksData::reserve(int nl){
	nLandmarks = 0;
	marksReserved = nl;
	if (landmarks) delete[] landmarks;
	landmarks = new landmark[nl]();
}
