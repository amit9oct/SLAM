#pragma once
#ifndef __MAT__FUNS__
#define __MAT__FUNS__

#include "robotTypes.h"
#include <math.h>

#ifndef PI
#define PI 3.1415926535f	// Con 1 palo y 5 ladrillos se pueden hacer mil cosas
#define PIx2 6.28318530f
#define PImed 1.57079632f
#endif

#ifndef EPS
#define EPS 0.000001
#endif

/**
*
* @brief Implents a common matrix and operations
*
*/
class matrix {

protected:

	unsigned short rows;
	unsigned short cols;
	float* val;

public:
	matrix();
	matrix(int rows, int cols);
	virtual ~matrix();
	matrix(const matrix&);
	matrix(const pos3d&);
	matrix(const posCil3d&);

	void set(int r, int c, float val);
	void set(int r, int c, const matrix& mat);
	float get(int r, int c) const;
	float operator()(int r, int c) const;

	int getRows() const;
	int getCols() const;

	void clear();
	
	matrix& operator= (const matrix&);
	matrix& operator= (const pos3d&);
	matrix& operator= (const posCil3d&);

	matrix& operator+= (const matrix&);
	matrix& operator-= (const matrix&);
	matrix& operator*= (const float&);

	matrix operator+ (const matrix&) const;
	matrix operator- (const matrix&) const;
	matrix operator* (const matrix&) const;
	matrix operator* (const float&) const;
	
	matrix transpose() const;
	matrix inverse() const;
	float det() const;

	void print(const char* str=0) const;

	pos3d toPos3d() const;
	pose toPose() const;

	static const matrix identity(int size);

};

inline float matrix::operator()(int r, int c) const		{return (r < rows && c < cols)? val[r*cols+c] :	0.0f;};
inline void matrix::set(int r, int c, float v)			{if (r < rows && c < cols) val[r*cols+c] = v;};
inline float matrix::get(int r, int c) const			{return (r < rows && c < cols)? val[r*cols+c] :	0.0f;};
inline int matrix::getRows() const						{return rows;}
inline int matrix::getCols() const						{return cols;}

//*******************************************************************************************************************

/// translades from local measure to global position
pos3d base2global(const pos3d& base, const pose& pos);
/// translades from global position to local measure
pos3d global2base(const pos3d& global, const pose& pos);

posCil3d cart2cil(const pos3d& xyz);
double normrnd(double mu=0.0, double sigma=1.0);
float gauss(int px, int py, float centerx, float centery, float sigma);
bool intersect(line& line1, line& line2, pointf& result);
float poly(const double* pol, const double x, int grad);

//********************************************************************************************************************

/**
*
* @brief Implents an expansible matrix and its operations
*
* This size reserved for the matrix data is independent of the current size
* This matrices can me expanded without relocating the data
*/
class Ematrix{

protected:

	unsigned short reservedRows;
	unsigned short reservedCols;
	unsigned short rows;
	unsigned short cols;
	float* val;

public:

	Ematrix();
	Ematrix(int rows, int cols, int reserveRows, int reserveCols);
	Ematrix(int rows, int cols);
	virtual ~Ematrix();
	Ematrix(const Ematrix&);
	Ematrix(const matrix&);
	Ematrix(const pos3d&);
	Ematrix(const posCil3d&);

	void set(int r, int c, float val);
	void set(int r, int c, const matrix& mat);
	void set(int r, int c, const Ematrix& mat);
	float get(int r, int c) const;
	float operator()(int r, int c) const;

	matrix subMat(int rini, int rend, int cini, int cend) const;
	void extend(int rinc, int cinc);
	void initialize(int rows, int cols, int reserveRows, int reserveCols);

	void clear();
	
	Ematrix& operator= (const Ematrix&);
	Ematrix& operator= (const matrix&);
	Ematrix& operator= (const pos3d&);
	Ematrix& operator= (const posCil3d&);

	Ematrix& operator+= (const Ematrix&);
	Ematrix& operator-= (const Ematrix&);

	Ematrix operator+ (const Ematrix&) const;
	Ematrix operator- (const Ematrix&) const;
	Ematrix operator* (const Ematrix&) const;
	Ematrix mul2 (const Ematrix&) const;

	Ematrix& operator*= (const float&);
	Ematrix operator* (const float&) const;
	
	void addSubMat(int r, int c, const Ematrix& mat);
	void addSubMat(int r, int c, const matrix& mat);
	
	Ematrix transpose() const;
	Ematrix inverse() const;
	float det() const;

	void print(const char* str=0) const;

	pos3d toPos3d() const;
	pose toPose() const;

	static const Ematrix identity(int size);

	int getNumRows() const;
	int getNumCols() const;

	float totalsum() const;

};

inline float Ematrix::operator()(int r, int c) const		{return (r < rows && c < cols)? val[r*reservedCols+c] :	0.0f;};
inline void Ematrix::set(int r, int c, float v)			{if (r < rows && c < cols) val[r*reservedCols+c] = v;};
inline float Ematrix::get(int r, int c) const			{return (r < rows && c < cols)? val[r*reservedCols+c] :	0.0f;};

inline int Ematrix::getNumRows() const				{return rows;};
inline int Ematrix::getNumCols() const				{return cols;};


#endif


