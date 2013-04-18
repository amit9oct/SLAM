
#include "pathPlanning.h"
#include <math.h>

using namespace std;

int AStarGrid(const point& rcell, const point& target, const gridMapInterface& omap, std::vector<point>& path, bool trackUnknown){

	point last;
	point* p;

	long int mincost, newcost;
	int ci, cj, i;
	bool stop = false;

	// costmap
	int len = omap.getWidth()*omap.getHeight();
	int mapstep = omap.getWidth();
	costCellH* costmap = new costCellH[len]();
	// clean cost map
	for (i=0; i < len; i++){
		costmap[i].open = false;
		costmap[i].closed = false;
	}

	// lists
	vector<point> openlist;
	vector<point> closedlist;
	vector<point>::iterator iter;
	vector<point>::iterator pointer;
	openlist.reserve(len);
	closedlist.reserve(len);
	path.reserve(omap.getWidth()+omap.getHeight());

	point next = rcell;
	int idxnext = next.y*mapstep+next.x;

	// mapa de costes inicial
	//binMap obs;
	//omap.occupiedCells(obs,2); 

	//1) Add the starting square (or node) to the open list.
	openlist.push_back(next);

	costmap[idxnext].parent = 0;
	costmap[idxnext].G = 0;
	costmap[next.y*mapstep+next.x].H = VHMOV * (abs(target.x-next.x)+abs(target.y-next.y)); // Heruristic cost
	costmap[idxnext].F = costmap[idxnext].G + costmap[idxnext].H;
	costmap[idxnext].open = true;

	// 2) Repeat the following:
	do{
		mincost = LONG_MAX;
		// a) Look for the lowest F cost square on the open list. We refer to this as the current square.
		for ( iter = openlist.begin( ); iter != openlist.end( ); iter++ ){
			int idxit = iter->y*mapstep+iter->x;
			if (mincost > costmap[idxit].F){
				mincost = costmap[idxit].F;
				pointer = iter;
			}
		}

		// b) Switch it to the closed list.
		memcpy(&last, &(*pointer), sizeof(point));
		closedlist.push_back(last);
		openlist.erase(pointer);
		int idxlast = last.y*mapstep+last.x;
		costmap[idxlast].closed = true;
		costmap[idxlast].open = false;

		// c) For each of the 8 squares adjacent to this current square
		for (ci=last.x-1; ci<= last.x+1; ci++){
			if (ci<0 || ci >=  omap.getWidth()) continue;
			for (cj=last.y-1; cj<= last.y+1; cj++){
				if (cj<0 || cj >=  omap.getHeight()) continue;
				if (ci!=last.x || cj!=last.y){
					idxnext = cj*mapstep+ci;
					//*If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following. 
					if( ( omap.isfree(ci,cj) || (trackUnknown && !omap.isoccupied(ci,cj)) ) && costmap[idxnext].closed==false){
						//*If it isn't on the open list, add it to the open list. Make the current square the parent of this square. Record the F, G, and H costs of the square. 
						if (costmap[idxnext].open==false){
//							cout << " closed" << endl;
							next.x = ci;
							next.y = cj;
							openlist.push_back(next);
							costmap[idxnext].parent = &closedlist.back();
							
							// G = lastcost + movement cost + penalize occupation probability
							if (fabs((float)(last.x-ci))+fabs((float)(last.y-cj))>1){	// diagonal
								costmap[idxnext].G = costmap[idxlast].G + DMOV + COSTOCC*omap.getValue(next.x,next.y,3); 
					 		}
							else{							// horizontal
								costmap[idxnext].G = costmap[idxlast].G + VHMOV + COSTOCC*omap.getValue(next.x,next.y,3);
							}
							costmap[idxnext].H = VHMOV * (abs(target.x-next.x)+abs(target.y-next.y)); // Heruristic cost
							costmap[idxnext].F = costmap[idxnext].G + costmap[idxnext].H;
							costmap[idxnext].open = true;
						}
						// If it is on the open list already, check to see if this path to that square is better, using G cost as the measure. 
						// A lower G cost means that this is a better path. If so, change the parent of the square to the current square, 
						// and recalculate the G and F scores of the square. If you are keeping your open list sorted by F score, you may need
						// to resort the list to account for the change.
						else{
							// newcost
							if (fabs((float)(last.x-ci))+fabs((float)(last.y-cj))>1){ // diagonal
								newcost = costmap[idxlast].G + DMOV + COSTOCC*omap.getValue(ci,cj,3); 
							}
							else{									// horizontal
								newcost = costmap[idxlast].G + VHMOV + COSTOCC*omap.getValue(ci,cj,3);
							}

							// if this new path is shorter...
							if(costmap[idxnext].G > newcost){
								costmap[idxnext].G = newcost;
								costmap[idxnext].F = costmap[idxnext].G + costmap[idxnext].H;
								costmap[idxnext].parent = &closedlist.back();
							}
						}
					}
				}
			}  // cj
		} //ci

		//  d) Stop when you:
		//    * Add the target square to the closed list, in which case the path has been found (see note below), or
		//    * Fail to find the target square, and the open list is empty. In this case, there is no path.   
		if (last.x == target.x && last.y == target.y){
			stop = true;
		}

	} while(!openlist.empty() && stop==false);
	//3) Save the path. Working backwards from the target square, go from each square to its parent square until you reach the starting square. That is your path. 
	path.clear();

	int sizePath = 0;
	if (stop){
		p = &closedlist.back();
		point np;
		while (p!=0){
			np.x = p->x;
			np.y = p->y;
			path.push_back(np);
			p = (point*) costmap[np.y*mapstep+np.x].parent;
		};
		sizePath = path.size();
	}

	delete[] costmap;
	return sizePath;
};

int DijkstraGrid(const point& rcell, const binMap& objectives, const gridMapInterface& omap, std::vector<point>& path, int inflaterad){

	point last;
	point* p;

	long int mincost, newcost;
	int ci, cj, i;
	bool stop = false;

	// costmap
	int len = omap.getWidth()*omap.getHeight();
	int mapstep = omap.getWidth();
	costCell* costmap = new costCell[len]();
	// clean cost map
	for (i=0; i < len; i++){
		costmap[i].open = false;
		costmap[i].closed = false;
	}

	// lists
	vector<point> openlist;
	vector<point> closedlist;
	vector<point>::iterator iter;
	vector<point>::iterator pointer;
	openlist.reserve(len);
	closedlist.reserve(len);
	path.reserve(omap.getWidth()+omap.getHeight());

	point next = rcell;
	int idxnext = next.y*mapstep+next.x;

	// mapa de costes inicial
	//binMap obs;
	//omap.occupiedCells(obs,2); 

	//1) Add the starting square (or node) to the open list.
	openlist.push_back(next);

	costmap[idxnext].parent = 0;
	costmap[idxnext].C = 0;
	costmap[idxnext].open = true;

	// 2) Repeat the following:
	do{
		mincost = LONG_MAX;
		// a) Look for the lowest F cost square on the open list. We refer to this as the current square.
		for ( iter = openlist.begin( ); iter != openlist.end( ); iter++ ){
			int idxit = iter->y*mapstep+iter->x;
			if (mincost > costmap[idxit].C){
				mincost = costmap[idxit].C;
				pointer = iter;
			}
		}

		// b) Switch it to the closed list.
		memcpy(&last, &(*pointer), sizeof(point));
		closedlist.push_back(last);
		openlist.erase(pointer);
		int idxlast = last.y*mapstep+last.x;
		costmap[idxlast].closed = true;
		costmap[idxlast].open = false;

		// c) For each of the 8 squares adjacent to this current square
		for (ci=last.x-1; ci<= last.x+1; ci++){
			for (cj=last.y-1; cj<= last.y+1; cj++){
				if (ci!=last.x || cj!=last.y){
					idxnext = cj*mapstep+ci;
					//*If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following. 
					if( (omap.isfree(ci,cj)) && (costmap[idxnext].closed==false)){
						//*If it isn't on the open list, add it to the open list. Make the current square the parent of this square. Record the F, G, and H costs of the square. 
						if (costmap[idxnext].open==false){
								next.x = ci;
								next.y = cj;
								openlist.push_back(next);
								costmap[idxnext].parent = &closedlist.back();
								costmap[idxnext].C = costmap[idxlast].C + omap.getValue(next.x,next.y,inflaterad)*COSTOCC + ( ((abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV: VHMOV);
								/*
								// C = lastcost + movement cost + penalize occupation probability
								if (fabs((float)(last.x-ci))+fabs((float)(last.y-cj))>1){	// diagonal
									costmap[idxnext].C = costmap[idxlast].C + DMOV + COSTOCC*omap.getValue(next.x,next.y,3); 
								}
								else{														// horizontal
									costmap[idxnext].C = costmap[idxlast].C + VHMOV + COSTOCC*omap.getValue(next.x,next.y,3);
								}*/
								costmap[idxnext].open = true;


						}
						// If it is on the open list already, check to see if this path to that square is better, using G cost as the measure. 
						// A lower G cost means that this is a better path. If so, change the parent of the square to the current square, 
						// and recalculate the G and F scores of the square. If you are keeping your open list sorted by F score, you may need
						// to resort the list to account for the change.
						else{
							// newcost
							newcost = costmap[idxlast].C + omap.getValue(next.x,next.y,inflaterad)*COSTOCC + ( ((abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV: VHMOV);
							/*if (fabs((float)(last.x-ci))+fabs((float)(last.y-cj))>1){ // diagonal
								newcost = costmap[idxlast].C + DMOV + COSTOCC*omap.getValue(ci,cj,3); 
							}
							else{									// horizontal
								newcost = costmap[idxlast].C + VHMOV + COSTOCC*omap.getValue(ci,cj,3);
							}*/

							// if this new path is shorter...
							if(costmap[idxnext].C > newcost){
								costmap[idxnext].C = newcost;
								costmap[idxnext].parent = &closedlist.back();
							}
						}
					}
				}
			}  // cj
		} //ci

		//  d) Stop when you:
		//    * Add the target square to the closed list, in which case the path has been found (see note below), or
		//    * Fail to find the target square, and the open list is empty. In this case, there is no path.   

		if( objectives.get(last.x,last.y)){ // if cell is a frontier
			stop = true;
		}

	} while(!openlist.empty() && stop==false);
	//3) Save the path. Working backwards from the target square, go from each square to its parent square until you reach the starting square. That is your path. 
	path.clear();

	int sizePath = 0;
	if (stop){
		p = &closedlist.back();
		point np;
		while (p!=0){
			np.x = p->x;
			np.y = p->y;
			path.push_back(np);
			p = (point*) costmap[np.y*mapstep+np.x].parent;
		};
		sizePath = path.size();
	}

	delete[] costmap;
	return sizePath;
};

//int DijkstraGridWithHeading(const ocell& rcell, const binMap& objectives, const gridMapInterface& omap, std::vector<ocell>& resultpath){
//	
//	ocell last;
//	long int mincost, newcost;
//	int i;
//	bool stop = false;
//	
//	// costmap
//	int len = omap.getWidth()*omap.getHeight()*8;
//	int ystep = omap.getWidth()*8;
//	costCell* costmap = new costCell[len]();
//	for (i=0; i < len; i++){		// clean cost map
//		costmap[i].open = false;
//		costmap[i].closed = false;
//	}
//	
//	// mapa de obstaculos
//	//binMap obs;
//	//omap.occupiedCells(obs,2); 
//	//obs.showMap("OBSTACLES");

//	// lists
//	vector<ocell> openlist;
//	openlist.reserve(len);
//	vector<ocell> closedlist;
//	closedlist.reserve(len);
//	
//	vector<ocell>::iterator pointer;
//	vector<ocell>::iterator iter;
//	
//	resultpath.reserve(4*(omap.getWidth()+omap.getHeight()));
//	
//	//1) Add the starting square (or node) to the open list.
//	ocell next = rcell;
//	int idxnext = next.y*ystep+next.x*8+next.th;
//	costmap[idxnext].parent = 0;
//	costmap[idxnext].C = 0;
//	costmap[idxnext].open = true;
//	openlist.push_back(next);

//	// 2) Repeat the following:
//	do{
//		mincost = 99999999;
//		// a) Look for the lowest C cost square on the open list. We refer to this as the current square.
//		for ( iter = openlist.begin(); iter != openlist.end(); iter++ ){
//			int cost = costmap[iter->y*ystep+iter->x*8+iter->th].C;
//			if (mincost > cost ){
//				mincost = cost;
//				pointer = iter;
//			}
//		}
//		
//		// b) Switch it to the closed list.
//		last = *pointer;
//		closedlist.push_back(last);
//		openlist.erase(pointer);
//		int idxlast = last.y*ystep+last.x*8+last.th;
//		costmap[idxlast].closed = true;
//		costmap[idxlast].open = false;
//		
//		// c) For each of the oriented cells accesible from the current square
//		for (i = -1 ; i < 2; i++){
//			next.th = last.th + i;
//			if (next.th > 7) next.th -= 8;
//			else if (next.th < 0) next.th += 8;
//			next.x = last.x + avancex[next.th]; 
//			next.y = last.y + avancey[next.th];
//			idxnext = next.y*ystep+next.x*8+next.th;
//			
//			//*If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following. 
//			if( omap.isfree(next.x,next.y) && (!costmap[idxnext].closed)){
//				//*If it isn't on the open list, add it to the open list. Make the current square the parent of this square.
//				if (!costmap[idxnext].open){
//					openlist.push_back(next);
//					costmap[idxnext].parent = &closedlist.back();
//					costmap[idxnext].C = costmap[idxlast].C + omap.getValue(next.x,next.y,3)*COSTOCC + ( ((abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV: VHMOV);
//					costmap[idxnext].open = true;
//				}
//				// If it is on the open list already, check to see if this path to that square is better.
//				else{
//					// newcost
//					newcost = costmap[idxlast].C + omap.getValue(next.x,next.y,3)*COSTOCC + (( (abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV : VHMOV);

//					// if this new path is shorter...
//					if(costmap[idxnext].C > newcost){
//						costmap[idxnext].C = newcost;
//						costmap[idxnext].parent = &closedlist.back();
//					}
//				}
//				//printf("l = %d, c = %d ", costmap[idxlast].C,costmap[idxnext].C);
//			}
//		}

//		//  d) Stop when you:
//		//    * Add the target square to the closed list, in which case the path has been found (see note below), or
//		//    * Fail to find the target square, and the open list is empty. In this case, there is no path.   
//		if( objectives.get(last.x,last.y) && costmap[idxlast].C > 100) // if cell is a frontier
//			stop = true;

//	} while(!openlist.empty() && !stop);

//	//3) Save the path. Working backwards from the target square, go from each square to its parent square until you reach the starting square. That is your path. 
//	resultpath.clear();
//	int sizePath = 0;

//	ocell* p;
//	if (stop){
//		if (closedlist.size()>0){
//			p = &closedlist.back();
//			while (p!=0){
//				resultpath.push_back(*p);
//				idxnext = p->y*ystep+p->x*8+p->th;
//				p = (ocell*)costmap[idxnext].parent;
//			}
//		}
//		sizePath = resultpath.size();
//	}
//	else if (closedlist.size()==1){
//		printf("no hay camino, media vuelta\n");
//		next.th=rcell.th+4;
//		if (next.th > 7) next.th -= 8;
//		else if (next.th < 0) next.th += 8; 
//		next.x = last.x + avancex[next.th]; 
//		next.y = last.y + avancey[next.th];
//		resultpath.push_back(next);
//		sizePath = resultpath.size();
//	}
//	printf("size path=%d\n",sizePath);
//	delete[] costmap;
//	return sizePath;
//};

//costMap* createPolicy(const ocell& target, const gridMapInterface& omap){

//	ocell last;
//	long int mincost, newcost;
//	int i;
//	bool stop = false;
//	
//	// costmap
//	int len = omap.getWidth()*omap.getHeight()*8;
//	int ystep = omap.getWidth()*8;
//	costCell* costmap = new costCell[len]();
//	for (i=0; i < len; i++){		// clean cost map
//		costmap[i].open = false;
//		costmap[i].closed = false;
//	}
//	
//	// mapa de obstaculos
//	binMap obs;
//	omap.occupiedCells(obs,2); 
//	//obs.showMap("OBSTACLES");

//	// lists
//	vector<ocell> openlist;
//	openlist.reserve(len);
//	vector<ocell>* closedlist = new vector<ocell>();
//	closedlist->reserve(len);

//	//vector<point> targets;
//	//int numtargets = objectives.getPositives(targets);

//	vector<ocell>::iterator pointer;
//	vector<ocell>::iterator iter;
//	//vector<point>::iterator targetiter;
//	
//	//1) Add the starting square (or node) to the open list.
//	ocell next = target;
//	int idxnext = next.y*ystep+next.x*8+next.th;
//	costmap[idxnext].parent = 0;
//	costmap[idxnext].C = 0;
//	costmap[idxnext].open = true;
//	openlist.push_back(next);

//	// 2) Repeat the following:
//	long maxcost = 0;
//	do{
//		mincost = 99999999;
//		// a) Look for the lowest C cost square on the open list. We refer to this as the current square.
//		for ( iter = openlist.begin(); iter != openlist.end(); iter++ ){
//			int cost = costmap[iter->y*ystep+iter->x*8+iter->th].C;
//			if (mincost > cost ){
//				mincost = cost;
//				pointer = iter;
//			}
//		}
//		
//		// b) Switch it to the closed list.
//		last = *pointer;
//		closedlist->push_back(last);
//		openlist.erase(pointer);
//		int idxlast = last.y*ystep+last.x*8+last.th;
//		costmap[idxlast].closed = true;
//		costmap[idxlast].open = false;
//		
//		// c) For each of the oriented cells accesible from the current square
//		for (i = -4 ; i <= 3; i++){
//			next.th = last.th;// + i;
//			if (next.th > 7) next.th -= 8;
//			else if (next.th < 0) next.th += 8;
//			next.x = last.x + avancex[next.th]; 
//			next.y = last.y + avancey[next.th];
//			idxnext = next.y*ystep+next.x*8+next.th;
//			
//			// If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following. 
//			if( omap.isfree(next.x,next.y) && (!costmap[idxnext].closed)){
//				// If it isn't on the open list, add it to the open list. Make the current square the parent of this square.
//				if (!costmap[idxnext].open){
//					openlist.push_back(next);
//					costmap[idxnext].parent = &closedlist->back();
//					costmap[idxnext].C = costmap[idxlast].C + omap.getValue(next.x,next.y,3)*COSTOCC + obs.get(next.x,next.y)*COSTOBS+ ( ((abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV: VHMOV);
//					costmap[idxnext].open = true;
//				}
//				// If it is on the open list already, check to see if this path to that square is better.
//				else{
//					// newcost
//					newcost = costmap[idxlast].C + omap.getValue(next.x,next.y,3)*COSTOCC + + obs.get(next.x,next.y)*COSTOBS+(( (abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV : VHMOV);

//					// if this new path is shorter...
//					if(costmap[idxnext].C > newcost){
//						costmap[idxnext].C = newcost;
//						costmap[idxnext].parent = &closedlist->back();
//					}
//				}
//				//printf("l = %d, c = %d ", costmap[idxlast].C,costmap[idxnext].C);
//			}
//		}

//		//  d) Stop when you:
//		//    * Add the target square to the closed list, in which case the path has been found (see note below), or
//		//    * Fail to find the target square, and the open list is empty. In this case, there is no path.
//	} while(!openlist.empty());

//	costMap* result = new costMap(costmap,ystep,8,closedlist,maxcost);

//	return result;
//}

// costMap* createCostMapOri(const ocell& origin, const binMap& objectives, const gridMapInterface& omap, vector<ocell>& reachable){

//	ocell last;
//	long int mincost, newcost;
//	int i;
//	bool stop = false;
//	
//	// costmap
//	int len = omap.getWidth()*omap.getHeight()*8;
//	int ystep = omap.getWidth()*8;
//	costCell* costmap = new costCell[len]();
//	for (i=0; i < len; i++){		// clean cost map
//		costmap[i].open = false;
//		costmap[i].closed = false;
//	}
//	
//	// mapa de obstaculos
//	//binMap obs;
//	//omap.occupiedCells(obs,2); 
//	//obs.showMap("OBSTACLES");

//	// lists
//	vector<ocell> openlist;
//	openlist.reserve(len);
//	vector<ocell>* closedlist = new vector<ocell>();
//	closedlist->reserve(len);
//	vector<point> targets;
//	int numtargets = objectives.getPositives(targets);

//	vector<ocell>::iterator pointer;
//	vector<ocell>::iterator iter;
//	vector<point>::iterator targetiter;
//	
//	//1) Add the starting square (or node) to the open list.
//	ocell next = origin;
//	int idxnext = next.y*ystep+next.x*8+next.th;
//	costmap[idxnext].parent = 0;
//	costmap[idxnext].C = 0;
//	costmap[idxnext].open = true;
//	openlist.push_back(next);

//	// 2) Repeat the following:
//	long maxcost = 0;
//	do{
//		mincost = 99999999;
//		// a) Look for the lowest C cost square on the open list. We refer to this as the current square.
//		for ( iter = openlist.begin(); iter != openlist.end(); iter++ ){
//			int cost = costmap[iter->y*ystep+iter->x*8+iter->th].C;
//			if (mincost > cost ){
//				mincost = cost;
//				pointer = iter;
//			}
//		}
//		
//		// b) Switch it to the closed list.
//		last = *pointer;
//		closedlist->push_back(last);
//		openlist.erase(pointer);
//		int idxlast = last.y*ystep+last.x*8+last.th;
//		costmap[idxlast].closed = true;
//		costmap[idxlast].open = false;
//		
//		// c) For each of the oriented cells accesible from the current square
//		for (i = -4 ; i <= 3; i++){
////		for (i = -1 ; i < 2; i++){
//			next.th = last.th + i;
//			if (next.th > 7) next.th -= 8;
//			else if (next.th < 0) next.th += 8;
//			next.x = last.x + avancex[next.th]; 
//			next.y = last.y + avancey[next.th];
//			idxnext = next.y*ystep+next.x*8+next.th;
//			
//			// If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following. 
//			if( omap.isfree(next.x,next.y) && (!costmap[idxnext].closed)){
//				// If it isn't on the open list, add it to the open list. Make the current square the parent of this square.
//				if (!costmap[idxnext].open){
//					openlist.push_back(next);
//					costmap[idxnext].parent = &closedlist->back();
//					costmap[idxnext].C = costmap[idxlast].C + omap.getValue(next.x,next.y,3)*COSTOCC + ( ((abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV: VHMOV);
//					costmap[idxnext].open = true;
//				}
//				// If it is on the open list already, check to see if this path to that square is better.
//				else{
//					// newcost
//					newcost = costmap[idxlast].C + omap.getValue(next.x,next.y,3)*COSTOCC + (( (abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV : VHMOV);

//					// if this new path is shorter...
//					if(costmap[idxnext].C > newcost){
//						costmap[idxnext].C = newcost;
//						costmap[idxnext].parent = &closedlist->back();
//					}
//				}
//				//printf("l = %d, c = %d ", costmap[idxlast].C,costmap[idxnext].C);
//			}
//		}

//		//  d) Stop when you:
//		//    * Add the target square to the closed list, in which case the path has been found (see note below), or
//		//    * Fail to find the target square, and the open list is empty. In this case, there is no path.
//		if( objectives.get(last.x,last.y)){ // if cell is a frontier
//			for ( targetiter = targets.begin(); targetiter != targets.end(); targetiter++ ){
//				if (targetiter->x == last.x && targetiter->y == last.y){
//					reachable.push_back(last);
//					targets.erase(targetiter);
//					numtargets--;
//					if (!numtargets) stop = true;
//					if (maxcost < costmap[idxlast].C) maxcost = costmap[idxlast].C;
//					break;
//				}
//			}
//		}
//	} while(!openlist.empty() && !stop);

//	costMap* result = new costMap(costmap,ystep,8,closedlist,maxcost);

//	return result;
//}


costMapSimple* createCostMap(const point& origin, const binMap& objectives, const gridMapInterface& omap, vector<point>& reachable, int inflaterad){

	point last;
	long int mincost, newcost;
	int i;
	bool stop = false;
	
	// costmap
	int len = omap.getWidth()*omap.getHeight();
	int ystep = omap.getWidth();
	costCell* costmap = new costCell[len]();
	for (i=0; i < len; i++){		// clean cost map
		costmap[i].open = false;
		costmap[i].closed = false;
	}
	
	// mapa de obstaculos
	//binMap obs;
	//omap.occupiedCells(obs,2); 
	//obs.showMap("OBSTACLES");

	// lists
	vector<point> openlist;
	openlist.reserve(len);
	vector<point>* closedlist = new vector<point>();
	closedlist->reserve(len);
	vector<point> targets;
	int numtargets = objectives.getPositives(targets);

	vector<point>::iterator pointer;
	vector<point>::iterator iter;
	vector<point>::iterator targetiter;
	
	//1) Add the starting square (or node) to the open list.
	point next = origin;
	int idxnext = next.y*ystep+next.x;
	costmap[idxnext].parent = 0;
	costmap[idxnext].C = 0;
	costmap[idxnext].open = true;
	openlist.push_back(next);

	// 2) Repeat the following:
	long maxcost = 0;
	do{
		mincost = 99999999;
		// a) Look for the lowest C cost square on the open list. We refer to this as the current square.
		for ( iter = openlist.begin(); iter != openlist.end(); iter++ ){
			int cost = costmap[iter->y*ystep+iter->x].C;
			if (mincost > cost ){
				mincost = cost;
				pointer = iter;
			}
		}
		
		// b) Switch it to the closed list.
		last = *pointer;
		closedlist->push_back(last);
		openlist.erase(pointer);
		int idxlast = last.y*ystep+last.x;
		costmap[idxlast].closed = true;
		costmap[idxlast].open = false;
		
		// c) For each of the 8 squares adjacent to this current square
		for (next.x=last.x-1; next.x<= last.x+1; next.x++){
			for (next.y=last.y-1; next.y<= last.y+1; next.y++){
				if (next.x!=last.x || next.y!=last.y){
					idxnext = next.y*ystep+next.x;
	
					// If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following. 
					if( omap.isfree(next.x,next.y) && (!costmap[idxnext].closed)){
						// If it isn't on the open list, add it to the open list. Make the current square the parent of this square.
						if (!costmap[idxnext].open){
							openlist.push_back(next);
							costmap[idxnext].parent = &closedlist->back();
							costmap[idxnext].C = costmap[idxlast].C + omap.getValue(next.x,next.y,inflaterad)*COSTOCC + ( ((abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV: VHMOV);
							costmap[idxnext].open = true;
						}
						// If it is on the open list already, check to see if this path to that square is better.
						else{
							// newcost
							newcost = costmap[idxlast].C + omap.getValue(next.x,next.y,inflaterad)*COSTOCC + (( (abs(last.x-next.x)+abs(last.y-next.y))>1)? DMOV : VHMOV);

							// if this new path is shorter...
							if(costmap[idxnext].C > newcost){
								costmap[idxnext].C = newcost;
								costmap[idxnext].parent = &closedlist->back();
							}
						}
					}
				}
			}
		}

		//  d) Stop when you:
		//    * Add the target square to the closed list, in which case the path has been found (see note below), or
		//    * Fail to find the target square, and the open list is empty. In this case, there is no path.
		if( objectives.get(last.x,last.y)){ // if cell is a frontier
			//printf("target found [%d, %d]\n",last.x,last.y);
			for ( targetiter = targets.begin(); targetiter != targets.end(); targetiter++ ){
				if (targetiter->x == last.x && targetiter->y == last.y){
					reachable.push_back(last);
					targets.erase(targetiter);
					numtargets--;
					if (!numtargets) stop = true;
					if (maxcost < costmap[idxlast].C) maxcost = costmap[idxlast].C;
					break;
				}
			}
		}
	} while(!openlist.empty() && !stop);
	costMapSimple* result = new costMapSimple(costmap,ystep,closedlist,maxcost);

	return result;
}
