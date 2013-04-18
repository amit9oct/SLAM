#pragma once
#ifndef ___ROBOT_TYPES___
#define ___ROBOT_TYPES___

#include <string.h>
#include <iostream>
#include <stdio.h>

/// 3D position
typedef struct pos3d{
	pos3d(){
		data[0] = 0; data[1] = 0; data[2] = 0;
	}
	pos3d(float nx, float ny, float nz){ 
		data[0] = nx; data[1] = ny; data[2] = nz;
	}
	float data[3];

	pos3d(const pos3d& p3){
		memcpy(data,p3.data,3*sizeof(float));
	}

	pos3d& operator= (const pos3d& p3){
		memcpy(data,p3.data,3*sizeof(float));
		return *this;
	}

	float& getX(){return data[0];}
	float& getY(){return data[1];}
	float& getZ(){return data[2];}

	const float& getX() const {return data[0];}
	const float& getY() const {return data[1];}
	const float& getZ() const {return data[2];}

	pos3d operator+(const pos3d& p) const{
		pos3d res(*this);
		res.data[0] += p.data[0];
		res.data[1] += p.data[1];
		res.data[2] += p.data[2];
		return res;
	}
	pos3d operator-(const pos3d& p) const{
		pos3d res(*this);
		res.data[0] -= p.data[0];
		res.data[1] -= p.data[1];
		res.data[2] -= p.data[2];
		return res;
	}
	void print(char* str=0) const {printf("%sx: %f, y: %f, z: %f\n",str,data[0],data[1],data[2]);};

}pos3d;


/// pose
typedef struct pose{
	float x; 
	float y; 
	float th;
	pose():	x(0.0f),y(0.0f),th(0.0f){}
	pose(float nx, float ny, float nth): x(nx),y(ny),th(nth){}
	pose(const pose& p): x(p.x),y(p.y),th(p.th){}
	pose& operator= (const pose& p) {x = p.x; y = p.y; th = p.th; return *this;}
	pose  operator+ (const pose& p) {return pose(x+p.x, y+p.y, th+p.th);}
	pose  operator- (const pose& p) {return pose(x-p.x, y-p.y, th-p.th);}
	pose& operator+=(const pose& p) {x+=p.x; y+=p.y; th+=p.th; return *this;}
	pose& operator-=(const pose& p) {x-=p.x; y-=p.y; th-=p.th; return *this;}
	void print(char* str=0) const {printf("%sx: %f, y: %f, th: %f\n",str,x,y,th);}
	pos3d toPos3d(){return pos3d(x,y,0);}
}pose;

static pose nullpose;


/// pose in 3d
typedef struct pose3d{
	pose3d(): pos(0.0f,0.0f,0.0f),th(0.0){}
	pose3d(const pos3d& p, float o):pos(p),th(o){}
	pose3d(float nx, float ny, float nz, float nth): pos(nx,ny,nz),th(nth){}
	pos3d pos; 
	float th;
}pose3d;

/// posCil3d
typedef struct posCil3d{
	posCil3d(){ 
		data[0] = 0; data[1] = 0; data[2] = 0;
	}
	posCil3d(float nr, float nalfa, float nz){ 
		data[0] = nr; data[1] = nalfa; data[2] = nz;
	}
	float data[3];

	posCil3d(const posCil3d& p3){
		memcpy(data,p3.data,3*sizeof(float));
	}

	posCil3d& operator= (const posCil3d& p3){
		memcpy(data,p3.data,3*sizeof(float));
		return *this;
	}

	float& getR(){return data[0];}
	float& getAlfa(){return data[1];}
	float& getZ(){return data[2];}

	const float& getR() const {return data[0];}
	const float& getAlfa() const {return data[1];}
	const float& getZ() const {return data[2];}

}posCil3d;

/// speed
typedef struct speed{
	speed(): v(0),w(0){}
	speed(float nv, float nw): v(nv),w(nw){}
	float v;
	float w;
}speed;

/// discrete point (pixel)
typedef struct point{
	point(): x(0),y(0){}
	point(unsigned short nx, unsigned short ny): x(nx),y(ny){}
	unsigned short x;
	unsigned short y;
}point;

/// cluster cell strcuture
typedef struct clusterCell{
	unsigned short x;
	unsigned short y;
	int scale;
	clusterCell(): x(0),y(0), scale(0){}
}clusterCell;

/// float point (coordinates)
typedef struct pointf{
	pointf(): x(0),y(0){}
	pointf(float nx, float ny):	x(nx),y(ny){}
	float x;
	float y;
}pointf;

/// line
typedef struct line{
	line():	x1(0.0f), y1(0.0f), x2(0.0f), y2(0.0f){};
	line(float nx1, float ny1, float nx2, float ny2): x1(nx1), y1(ny1), x2(nx2), y2(ny2){};
	line(pointf p1, pointf p2): x1(p1.x), y1(p1.y), x2(p2.x), y2(p2.y){};			
	float x1;
	float y1;
	float x2;
	float y2;
}line;

/// region of interest
typedef struct RoI{
	RoI(): x(0),y(0),width(0),height(0){}
	RoI(int nx, int ny, int nw, int nh): x(nx),y(ny),width(nw),height(nh){}
	int x;
	int y;
	int width;
	int height;
}RoI;


#endif

