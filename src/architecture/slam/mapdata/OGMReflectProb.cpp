/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class OGMReflectProb
*
* Implements an ocupancy grid map
*
*
*
*/

#include <math.h> 
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "OGMReflectProb.h"
#include <iostream>


// Para registrar la clase en la factoria
namespace{
	gridMapInterface* CreateReflectProb(float w, float h, float res, float x, float y){
		return new OGMReflectProb(w,h,res,x,y);
	};
	const int REFLECTPROB = 3;
	const bool registered = gridMapFactory::Instance().Register(REFLECTPROB, gridMapCreator(CreateReflectProb));
}

using namespace std;

// Constructors
OGMReflectProb::OGMReflectProb():
	occupancyGridMap(),
	cells(0),
	step(1.0f)
{
};

OGMReflectProb::OGMReflectProb(float w, float h, float res, float x, float y):
	occupancyGridMap(w,h,res,x,y),
	totalsize(width*height),
	cells(new cell[totalsize]),
	step(1.0f)
{	
	reset();
}

OGMReflectProb::~OGMReflectProb(){
	if (cells) delete[] cells;
}

OGMReflectProb::OGMReflectProb(const OGMReflectProb& gmap):
	occupancyGridMap(gmap.realWidth, gmap.realHeight, gmap.resolution, gmap.xorigin, gmap.yorigin),
	totalsize(gmap.totalsize),
	cells (new cell[gmap.totalsize])
{
	memcpy(cells,gmap.cells,totalsize*sizeof(cell));
}

gridMapInterface& OGMReflectProb::operator=(const gridMapInterface& gmap){
    assert(typeid(gmap) == typeid(*this));
    const OGMReflectProb& map = dynamic_cast<const OGMReflectProb&>(gmap);
    occupancyGridMap::operator=(gmap);
	if (totalsize != map.totalsize){
		totalsize = map.totalsize;
		if (cells) delete[] cells;
		cells = new cell[totalsize];
	}
	memcpy(cells,map.cells,totalsize*sizeof(cell));
	return *this;
}

// Initializer
void OGMReflectProb::initialize(float w, float h, float res, float x, float y){
	width  = (int)floor(w/res+0.5);
	height = (int)floor(h/res+0.5);
	realWidth  = w;
	realHeight = h;
	xorigin = x;
	yorigin = y;
	resolution = res;
	if (cells) delete[] cells;
	totalsize =width*height;
	cells = new cell[totalsize];
	reset();
};

// Data operations
void OGMReflectProb::reset(){
	step=1.0f;
	for(int i=0;i<totalsize;i++){
		cells[i].occ = 0;
		cells[i].total = 0;
		cells[i].val = 50.0f;
		//cells[i].val2 = 50.0f;
	}
}

// TODO:SONAR AND LASER MODEL ARE DIFERENT!!!
void OGMReflectProb::update(const rangeSensorData& rsData, const pose& rpos, float disp){

	bool laser=true;
	if (laser){
		point rcell,pcell;
		pointf p;
		int size, markCells;
		const int timeHorizon = 33; 

		float markingRange =  rsData.getMaxDist() - (rsData.getDevError()*rsData.getMaxDist());;

		rcell= toCell(rpos.x, rpos.y); // robot cell		

		// for each sensor
		for(int s=0; s < rsData.getNumSensors(); s++){
			// position of the sensed point
			float th = rpos.th + rsData.getSensorPose(s).th;
			p.x = rpos.x + rsData.getSensorValue(s)*cos(th);
			p.y = rpos.y + rsData.getSensorValue(s)*sin(th);
			pcell = toCell(p.x, p.y);
			point* line = getLine(rcell.x,rcell.y,pcell.x,pcell.y,size);

			markCells = floor((rsData.getDevError()*rsData.getSensorValue(s))/resolution)+1;

			// clearing
			for (int i = 0; i < size-markCells; i++){
				if (get(line[i].x,line[i].y).total < timeHorizon) 
					get(line[i].x,line[i].y).total += 1.0f;
				else if (get(line[i].x,line[i].y).occ > 1) 
					get(line[i].x,line[i].y).occ -= 1.0f;
				//get(line[i].x,line[i].y).val   = 100.0f*get(line[i].x,line[i].y).occ/get(line[i].x,line[i].y).total;
				float valtmp   = 100.0f*get(line[i].x,line[i].y).occ/get(line[i].x,line[i].y).total;
				get(line[i].x,line[i].y).val    = valtmp;
			}
			delete line;
		}
		for(int s=0; s < rsData.getNumSensors(); s++){		
			// position of the sensed point
			float th = rpos.th + rsData.getSensorPose(s).th;
			p.x = rpos.x + rsData.getSensorValue(s)*cos(th);
			p.y = rpos.y + rsData.getSensorValue(s)*sin(th);
			pcell = toCell(p.x, p.y);
			point* line = getLine(rcell.x,rcell.y,pcell.x,pcell.y,size);
	
			markCells = floor((rsData.getDevError()*rsData.getSensorValue(s))/resolution)+1;

			// marking
			if (rsData.getSensorValue(s) < markingRange){
				for (int i = size-markCells; i < size ; i++){
					if (get(line[i].x,line[i].y).total < timeHorizon){
						get(line[i].x,line[i].y).total += 1.0f;
						get(line[i].x,line[i].y).occ   += 1.0f;
					}
					else if (get(line[i].x,line[i].y).occ < timeHorizon){
						get(line[i].x,line[i].y).occ += 1.0f;
					}
					//get(line[i].x,line[i].y).val   = 100.0f*get(line[i].x,line[i].y).occ/get(line[i].x,line[i].y).total;
					float valtmp  = 100.0f*get(line[i].x,line[i].y).occ/get(line[i].x,line[i].y).total;
					get(line[i].x,line[i].y).val   = valtmp;
				}
			}
			delete line;
		}
	}
	else{
		//float conffactor = 1.0f;

		int s,x,y;
		float xini, xend, yini, yend;
		int ixini, ixend, iyini, iyend;
		pose pSens, pSensRel;
		pointf p1, p2, p3;
		float d, xposdif, yposdif, ang, apmax, dmax, dmin, eps; //er, ea ;
		float auxsin, auxcos, auxemp, auxocc;
		float difangle;

		apmax = rsData.getAperture()/2;
		dmax  = rsData.getMaxDist();
		dmin  = rsData.getMinDist();
		eps   = 3*rsData.getDevError(); 

		double limval = dmax-eps-3*resolution; // 3*sigma
		double limvalmin = dmin+resolution;

		// clear the position of the robot
		// TODO: clear the complete footprint =>
		// 	- read the footprint from the config file
		// 	- draw lines for the footprint poligon
		// 	- filling algorithm
		// 	- translate it to the occupancy gridmap
		int rx = (int) floor((rpos.x - xorigin)/resolution);
		int ry = (int) floor((rpos.y - yorigin)/resolution);
		get(rx,ry).total++;						
		get(rx,ry).val = 100.0f*get(rx,ry).occ/get(rx,ry).total;
		if (get(rx,ry).val == 50.0f) get(rx,ry).val = 49.0f;

		for(s=0; s < rsData.getNumSensors(); s++){  // for each sensor

			// position of the sensor
			auxcos = cos(rpos.th);
			auxsin = sin(rpos.th);
		
			pSens.x  = rpos.x + rsData.getSensorPose(s).x * auxcos - rsData.getSensorPose(s).y * auxsin;
			pSens.y  = rpos.y + rsData.getSensorPose(s).x * auxsin + rsData.getSensorPose(s).y * auxcos;
			pSens.th = rpos.th + rsData.getSensorPose(s).th;

			auxemp = (rsData.getSensorValue(s) - eps);
			auxocc = (rsData.getSensorValue(s) + eps);
			//float aux0 = auxemp-dmin;

			// cone limits
			difangle = pSens.th + apmax;	// one extreme of the beam
			auxcos = cos(difangle);
			auxsin = sin(difangle);
			p3.x = pSens.x + auxocc * auxcos;
			p3.y = pSens.y + auxocc * auxsin;
			difangle = pSens.th - apmax;	// the other
			auxcos = cos(difangle);
			auxsin = sin(difangle);
			p1.x = pSens.x + auxocc * auxcos;
			p1.y = pSens.y + auxocc * auxsin;
			difangle = pSens.th;			// the center
			auxcos = cos(difangle);
			auxsin = sin(difangle);
			p2.x = pSens.x + auxocc * auxcos;
			p2.y = pSens.y + auxocc * auxsin;
		
			// cogemos el mayor y el menor para definir un area rectangular que actualizar
			xini = (pSens.x < p3.x)? ((pSens.x < p2.x)? ((pSens.x < p1.x)? pSens.x : p1.x):
														((p2.x < p1.x)?    p2.x    : p1.x) ) :    
									 ((p3.x < p2.x)?    ((p3.x < p1.x)?    p3.x    : p1.x):
														((p2.x < p1.x)?    p2.x    : p1.x) ) ;
			xend = (pSens.x > p3.x)? ((pSens.x > p2.x)? ((pSens.x > p1.x)? pSens.x : p1.x):
														((p2.x > p1.x)?    p2.x    : p1.x) ) :    
									 ((p3.x > p2.x)?    ((p3.x > p1.x)?    p3.x    : p1.x):
														((p2.x > p1.x)?    p2.x    : p1.x) ) ;

			yini = (pSens.y < p3.y)? ((pSens.y < p2.y)? ((pSens.y < p1.y)? pSens.y : p1.y):
														((p2.y < p1.y)?    p2.y    : p1.y) ) :    
									 ((p3.y < p2.y)?    ((p3.y < p1.y)?    p3.y    : p1.y):
														((p2.y < p1.y)?    p2.y    : p1.y) ) ;
			yend = (pSens.y > p3.y)? ((pSens.y > p2.y)? ((pSens.y > p1.y)? pSens.y : p1.y):
														((p2.y > p1.y)?    p2.y    : p1.y) ) :    
									 ((p3.y > p2.y)?    ((p3.y > p1.y)?    p3.y    : p1.y):
														((p2.y > p1.y)?    p2.y    : p1.y) ) ;

			// to cells
			ixini = (int) floor((xini - xorigin)/resolution)-1;
			ixend = (int) ceil ((xend - xorigin)/resolution)+1;
			iyini = (int) floor((yini - yorigin)/resolution)-1;
			iyend = (int) ceil ((yend - yorigin)/resolution)+1;
		
			float posomapsensx = xorigin - pSens.x;
			float posomapsensy = yorigin - pSens.y;

			//int sizex = ixend-ixini+1;
			//int sizey = iyend-iyini+1;
			//printf("size: %d x %d\n", sizex, sizey);

			//float normalizesum = 0.0;

			for(x=ixini; x<=ixend; x++){
				for(y=iyini; y<=iyend; y++){

					xposdif = posomapsensx + resolution * (x+0.5f);
					yposdif = posomapsensy + resolution * (y+0.5f);
					ang = atan2(yposdif,xposdif) - pSens.th;
					if (ang>PImed) ang -= PIx2;
					if (ang<-PImed) ang += PIx2;
					ang = fabs(ang);
				
					if (ang <= apmax){				// check angle
						d = sqrt(xposdif*xposdif+yposdif*yposdif);
						if (d <= limval && d >= limvalmin){	// check distance
							if (d < auxemp){
								get(x,y).val = 0.0f;
								get(x,y).total++;						
							}
							else if (d < auxocc){		// occupied zone
								get(x,y).val = 100.0f;
								get(x,y).occ+=3;
								get(x,y).total+=3;						
							}
						}
						/*if (d <= limval && d >= limvalmin){	// check distance
							if (d < auxemp)
								get(x,y).total++;						
							else if (d < auxocc){		// occupied zone
								get(x,y).occ+=3;
								get(x,y).total+=3;						
							}
							get(x,y).val = 100.0f*get(x,y).occ/get(x,y).total;
							if (get(x,y).val == 50.0f) get(x,y).val = 49.0f;
						}*/
					}
				} //y
			} //x
		}// sensor
	}
}

/// save a map to disk
int OGMReflectProb::saveMapToFile(const char* file) const{
	try{
		ofstream outfile;
		outfile.open (file, ios::out | ios::binary);
		if (outfile.good()){
			outfile.write((char *) &width, sizeof width);
			outfile.write((char *) &height, sizeof height);
			outfile.write((char *) &realWidth, sizeof realWidth);
			outfile.write((char *) &realHeight, sizeof realHeight);
			outfile.write((char *) &xorigin, sizeof xorigin);
			outfile.write((char *) &yorigin, sizeof yorigin);
			outfile.write((char *) &resolution, sizeof resolution);
			outfile.write((char *) &totalsize, sizeof totalsize);
			outfile.write((char *) cells, totalsize*sizeof(cell));
			outfile.close();
			return true;
		}
		else{
			printf("error saving map file\n");		
			return false;
		}
	}
	catch(const ios::failure &problem)
	{
		cout << problem.what();
		return -1;
	}
}

/// load a map from disk
int OGMReflectProb::loadMapFromFile(const char* file){
	try{
		ifstream inputfile;	
		inputfile.open(file, ios::in | ios::binary);
		if (inputfile.good()){
			inputfile.read((char *) &width, sizeof width);
			inputfile.read((char *) &height, sizeof height);
			inputfile.read((char *) &realWidth, sizeof realWidth);
			inputfile.read((char *) &realHeight, sizeof realHeight);
			inputfile.read((char *) &xorigin, sizeof xorigin);
			inputfile.read((char *) &yorigin, sizeof yorigin);
			inputfile.read((char *) &resolution, sizeof resolution);
			inputfile.read((char *) &totalsize, sizeof totalsize);
			if (cells) delete[] cells;
			cells = new cell[totalsize]();
			inputfile.read((char *) cells, totalsize*sizeof(cell));
			inputfile.close();
			return true;
		}
		else{
			printf("error loading map file\n");		
			return false;
		}
	}
	catch(const ios::failure &problem)
	{
		cout << problem.what();
		return -1;
	}
}


