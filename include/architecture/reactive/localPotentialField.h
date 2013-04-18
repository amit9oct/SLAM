
#ifndef __LOCAL_POTENTIAL_FIELD__
#define __LOCAL_POTENTIAL_FIELD__

#include "matFuns.h"

/**
* @brief Local Potential Fields for reactive navigation
* 
*/

class localPotentialField{

private:

	int width;
	int height;

	matrix m;

public:

	localPotentialField();
	localPotentialField(int width, int height);
	localPotentialField(const localPotentialField &lff);
	virtual ~localPotentialField();

	localPotentialField& operator=(const localPotentialField&);
	localPotentialField operator*(const float&);
	localPotentialField operator+(const localPotentialField&);
	localPotentialField& operator+=(const localPotentialField&);
	localPotentialField& operator*=(const float&);

	void set(int i, int j, float val);
	float get(int i, int j);

	int getWidth();
	int getHeight();

	void reset();

	/// Save function
	void savePotentialAsImage(char* file);
	void normalize();

};

inline float localPotentialField::get(int i, int j)				{return (i>=0 && i<width && j>=0 && j<height)? m.get(i,j) : 0.0f;}
inline void localPotentialField::set(int i, int j, float val)	{if(i>=0 && i<width && j>=0 && j<height) m.set(i,j,val);}
inline int localPotentialField::getWidth()						{return width;}
inline int localPotentialField::getHeight()						{return height;}

#endif
