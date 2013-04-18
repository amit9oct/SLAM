#pragma once
#ifndef __PATH__PLANNING__FUNS___
#define __PATH__PLANNING__FUNS___

#include "robotTypes.h"
#include "binMap.h"
#include "gridMapInterface.h"

#define VHMOV			100
#define DMOV			142
#define COSTOCC			100
#define COSTOBS			1000


//typedef struct ocell{
//	unsigned short x;
//	unsigned short y;
//	short th;  // 0, 45, 90, 135, 180, 225, 270, 315
//}ocell;

/// Cost cell struct used for path planning with heuristics
typedef struct costCellH{
	long int F;
	long int G;
	long int H;
	bool closed;
	bool open;
	void* parent;
}costCellH;

/// Cost cell struct used for path planning
typedef struct costCell{
	long int C;
	void* parent;
	bool closed;
	bool open;
	costCell():C(999999999L),parent(0){}
}costCell;

//class costMap{
//private:
//	costCell* data;
//	std::vector<ocell>* closedlist;
//	int step;
//	int angles;
//	long maxcost;
//public:
//	costMap(costCell* d, int s, int ang, std::vector<ocell>* cl, long max): data(d), closedlist(cl), step(s), angles(ang), maxcost(max){};
//	//costMap* createPolicy(const ocell& target, const gridMapInterface& omap);
//	virtual ~costMap(){if (data){ delete[] data; delete closedlist;}};
//	long getCost(int i, int j){
//		long min=data[j*step+i*angles].C;
//		for (int a = 1; a < angles; a++){
//			int idx = j*step+i*angles+a;
//			if (data[idx].C<min) min = data[idx].C;
//		}
//		return min;
//	}
//	int getPath(const ocell& dest, std::vector<ocell>& resultpath){
//		resultpath.clear();
//		const ocell* p = &dest;
//		while (p!=0){
//			resultpath.push_back(*p);
//			long idxnext = p->y*step+p->x*angles+p->th;
//			p = (ocell*) data[idxnext].parent;
//		}
//		resultpath.pop_back();

//		return  resultpath.size();
//	}
//	long getMaxCost(){return maxcost;};
//};


/**
* @brief Stores the cost to reach each cell of the map
*/
class costMapSimple{
private:
	costCell* data;
	std::vector<point>* closedlist;
	int step;
	long maxcost;
public:
	costMapSimple(costCell* d, int s, std::vector<point>* cl, long max): data(d), closedlist(cl), step(s), maxcost(max){};
	//costMapSimple* createPolicy(const point& target, const gridMapInterface& omap);
	virtual ~costMapSimple(){if (data){ delete[] data; delete closedlist;}};
	long getCost(int i, int j){
		return data[j*step+i].C;
	}
	int getPath(const point& dest, std::vector<point>& resultpath){
		resultpath.clear();
		const point* p = &dest;
		while (p!=0){
			resultpath.push_back(*p);
			long idxnext = p->y*step+p->x;
			p = (point*) data[idxnext].parent;
		}
		resultpath.pop_back();

		return  resultpath.size();
	}
	long getMaxCost(){return maxcost;};
};

//const int avancex[8] = {1, 1, 0, -1, -1, -1, 0, 1};
//const int avancey[8] = {0, 1, 1,  1,  0, -1, -1, -1};

int AStarGrid(const point& origin, const point& target, const gridMapInterface& omap, std::vector<point>& resultpath, bool trackUnknown);

int DijkstraGrid(const point& origin, const binMap& objectives, const gridMapInterface& omap, std::vector<point>& resultpath, int inflaterad);

costMapSimple* createCostMap(const point& origin, const binMap& objectives, const gridMapInterface& omap, std::vector<point>& reachable, int inflaterad);

//int DijkstraGridWithHeading(const ocell& origin, const binMap& objectives, const gridMapInterface& omap, std::vector<ocell>& resultpath);

//costMap* createCostMapOri(const ocell& origin, const binMap& objectives, const gridMapInterface& omap, std::vector<ocell>& reachable);

//costMap* createPolicy(const ocell& target, const gridMapInterface& omap);

#endif
