/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class binMap
*
* Implements a binary gridmap
*
*/
#include "binMap.h"

using namespace std;

// Constructors

// Default constructor
binMap::binMap():
width(0),
height(0),
totalsize(0),
xorigin(0),
yorigin(0),
resolution(1),
roi(0,0,0,0),
cells(0)
{

}
// Builds a binary grid map of size w x h
binMap::binMap(int w, int h):
width(w),
height(h),
totalsize((width*height)/32 + 1),
xorigin(0),
yorigin(0),
resolution(1),
roi(0,0,w,h)
{
	cells.resize(totalsize);
	clear();
};
// Builds a binary grid map of size w x h with origin in real coordinates (x,y) and resolution res
binMap::binMap(int w, int h, float res, float x, float y):
width(w),
height(h),
totalsize((width*height)/32 + 1),
xorigin(x),
yorigin(y),
resolution(res),
roi(RoI(0,0,w,h))
{
	cells.resize(totalsize);
	clear();
};
// Builds a binary grid map of size w x h (in meters) with origin in real coordinates (x,y) and resolution res
binMap::binMap(float w, float h, float res, float x, float y):
width((int)floor(w/res+0.5)),
height((int)floor(h/res+0.5)),
totalsize((width*height)/32 + 1),
xorigin(x),
yorigin(y),
resolution(res),
roi(RoI(0,0,width,height))
{
	cells.resize(totalsize);
	clear();
};
// Copy contructor
binMap::binMap(const binMap &map):
width(map.width),
height(map.height),
totalsize(map.totalsize),
xorigin(map.xorigin),
yorigin(map.yorigin),
resolution(map.resolution),
roi(map.roi),
cells(map.cells)
{

};
// Destructor
binMap::~binMap(){

}
// Initializers
void binMap::initialize(int w, int h){
	initialize(w,h,0,0,0);
};
void binMap::initialize(int w, int h, float res, float x, float y){
	width  = w;
	height = h;
	totalsize = (width*height)/32 + 1;
	xorigin = x;
	yorigin = y;
	resolution = res;
	cells.resize(totalsize);
	roi.x = 0;
	roi.y = 0;
	roi.width = width;
	roi.height = height;
	clear();
};
void binMap::initialize(float w, float h, float res, float x, float y){
	int wd  = (int)floor(w/res+0.5);
	int ht = (int)floor(h/res+0.5);
	initialize(wd, ht, res, x, y);
};
binMap& binMap::operator=(const binMap& map){
	width = map.width;
	height = map.height;
	totalsize = map.totalsize;
	xorigin = map.xorigin;
	yorigin = map.yorigin;
	resolution = map.resolution;
	roi = map.roi;
	cells = map.cells;
	return *this;
}


// Data operations

bool binMap::isInside(int x, int y, int size) const{
	for (int i = x-size; i<= x+size; i++)
		for (int j = y-size; j<= y+size; j++)
			if (!get(i,j))
				return false;
	return true;
}

bool binMap::isInsideD(int x, int y) const {
	int i,j;
	i = x; j = y;
	if (!get(i,j))
		return false;
	i = x+1; j = y;
	if (!get(i,j))
		return false;
	i = x-1; j = y;
	if (!get(i,j))
		return false;
	i = x; j = y+1;
	if (!get(i,j))
		return false;
	i = x; j = y-1;
	if (!get(i,j))
		return false;
	return true;
}

bool binMap::isOver(int x, int y, int size) const {
	for (int i = x-size; i<= x+size; i++)
		for (int j = y-size; j<= y+size; j++)
			if (get(i,j)){
				return true;
			}
	return false;
}

// Full map operations
void binMap::sub(const binMap& map){
	for (int i = 0; i< (int)cells.size(); i++){
		cells[i] &= (~map.cells[i]);
	}
}
void binMap::times(const binMap& map){
	for (int i = 0; i< (int)cells.size(); i++){
		cells[i] &= map.cells[i];
	}
}
void binMap::invert(){
	for (int i = 0; i< (int)cells.size(); i++){
		cells[i].flip();
	}
}

void binMap::add(const binMap& map){
	for (int i = 0; i< (int)cells.size(); i++){
		cells[i] |= map.cells[i];
	}
	if (roi.x > map.roi.x){
		roi.width += roi.x-map.roi.x; 
		roi.x = map.roi.x;
	}
	if (roi.y > map.roi.y){
		roi.height += roi.y-map.roi.y; 
		roi.y = map.roi.y;	
	}
	if (roi.x + roi.width <= map.roi.x + map.roi.width){
		roi.width += (map.roi.x + map.roi.width) - (roi.x + roi.width);
	}
	if (roi.y + roi.height <= map.roi.y + map.roi.height){
		roi.height += (map.roi.y + map.roi.height) - (roi.y + roi.height);
	}
}
void binMap::clear(){
	for (int i = 0; i< (int)cells.size(); i++){
		cells[i].reset();
	}
}
void binMap::closing(int rad){
	dilate(rad);
	erode(rad);
}
void binMap::opening(int rad){
	erode(rad);
	dilate(rad);
}
void binMap::dilate(int rad){
	int i,j;
	binMap prevData(*this);
	roi.x -= rad;
	roi.y -= rad;
	roi.width += 2*rad;
	roi.height += 2*rad;
	if (roi.x < 0){ roi.width  += roi.x; roi.x = 0; }
	if (roi.y < 0){ roi.height += roi.y; roi.y = 0; }
	if (roi.x + roi.width  > width)  {roi.width  = width  - roi.x;} 
	if (roi.y + roi.height > height) {roi.height = height - roi.y;} 
	for (i = roi.x ; i < roi.x + roi.width ; i++){
		for (j = roi.y ; j < roi.y + roi.height ; j++){   // for each cell				
			if(prevData.isOver(i,j,rad)){
				set(i,j,true);
			}
			else{
				set(i,j,false);
			}
		}
	}
}
void binMap::erode(int rad){
	int i,j;
	binMap prevData(*this);
	roi.x += rad;
	roi.y += rad;
	roi.width -= 2*rad;
	roi.height -= 2*rad;
	for (i = roi.x ; i < roi.x + roi.width ; i++){
		for (j = roi.y ; j < roi.y + roi.height ; j++){   // for each cell				
			if(prevData.isInside(i,j,rad)){
				set(i,j,true);
			}
			else{
				set(i,j,false);
			}
		}
	}
}

void binMap::remUnconnectRec(int x, int y, const binMap& prevData){
	for (int i = x-1 ; i <= x+1; i++){
		for (int j = y-1 ; j <= y+1 ; j++){   // for each cell				
			if(prevData.get(i,j) && !get(i,j)) {
				set(i,j,true);
				remUnconnectRec(i,j,prevData);
			}
		}
	}
}

void binMap::removeUnconnected(int x, int y){
	binMap prevData(*this);
	clear();
	set(x,y,true);
	remUnconnectRec(x,y,prevData);
}


// Drawing Functions
void binMap::line(int x1, int y1, int x2, int y2){
	int i,j ;
	float incx, incy, auxx, auxy;
	auxx = fabs((float)(x1-x2));
	auxy = fabs((float)(y1-y2));
	incx = auxx/auxy;
	incy = auxy/auxx;

	int count = 0;

	if (incx > incy){
		j = y1;

		count=1;
		if (x1 < x2){
			for(i = x1; i <= x2; i++){
				if (fabs((float)(i-x1))>=count*incx) {
					count++;
					if (y1<y2)
						j++;
					else
						j--;
				}
				set(i,j,true);
			}
		}
		else{
			for(i = x1; i >= x2; i--){
				if (fabs((float)(x1-i))>=count*incx) {
					count++;
					if (y1<y2)
						j++;
					else
						j--;
				}
				set(i,j,true);
			}
		}
	}

	else{
		j = x1;

		count=1;
		if (y1 < y2){
			for(i = y1; i <= y2; i++){
				if (fabs((float)(i-y1))>=count*incy) {
					count++;
					if (x1<x2)
						j++;
					else
						j--;
				}
				set(j,i,true);
			}
		}
		else{
			for(i = y1; i >= y2; i--){
				if (fabs((float)(y1-i))>=count*incy) {
					count++;
					if (x1<x2)
						j++;
					else
						j--;
				}
				set(j,i,true);
			}
		}
	}
}

void binMap::saveMapAsImage(const char* file) const{

	IplImage* img = getMapAsImage();
	cvSaveImage(file, img);
	cvReleaseImage(&img);
}

void binMap::showMap(const char* windowname)const{
	IplImage* img = getMapAsImage();
	cvNamedWindow(windowname, 0);
	cvShowImage(windowname,img);
	cvReleaseImage(&img);

}

IplImage* binMap::getMapAsImage() const {
	IplImage* img = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3); 

	for (int i = 0; i< width; i++)
		for(int j = 0; j< height; j++){
			uchar greyvalue = (uchar) (255.0f*get(i,j));
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i] = greyvalue;		
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i+1] = greyvalue;		
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i+2] = greyvalue;		
		}
	return img;
}

int binMap::getPositives(vector<point>& positives) const{
	int idx = 0;
	int count=0;
	positives.reserve(width*height);
	for (int j = 0; j< height; j++){
		for (int i = 0; i< width; i++){
			if (cells[idx/32][idx%32]){
				positives.push_back(point(i,j));
				count++;
			}
			idx++;
		}
	}
	return count;
}

int binMap::cluster(vector<clusterCell>& clusterList, int minSize) const{

	//mapa en blanco de celdas analizadas
	binMap aux(getWidth(), getHeight(), getResolution(), getXOrigin(), getYOrigin());

	clusterCell cl;
	int count = 0;	// numero de clusters encontrado

	for (int i = getRoi().x ; i < getRoi().x + getRoi().width ; i++){
		for (int j = getRoi().y ; j < getRoi().y + getRoi().height ; j++){   // for each cell
			
			std::list<point> seq;
			clustering(i,j,aux,cl,seq,true);
			
			if (cl.scale >= minSize){ // cluster de al menos 3 celdas

				// nos quedamos con la celda central del cluster
				std::list<point>::iterator it;
				int k;
				for( k = 0, it = seq.begin(); it != seq.end() && k <= cl.scale/2; it++ , k++);
				it--;
				k--;
				cl.x = it->x;
				cl.y = it->y;

				count++;
	
				clusterList.push_back(cl);
			}
			cl.x=0;
			cl.y=0;
			cl.scale=0;
		}
	}

	return count;
}

bool binMap::clustering(const int &i, const int &j, binMap& aux, clusterCell& cl, std::list<point> &seq, bool backfront) const{

	if(!aux.get(i,j) && get(i,j) ){  // if ipoint or door
		point p(i,j);
		if (backfront)	seq.push_back(p);
		else	seq.push_front(p);

		cl.x += i;
		cl.y += j;
		cl.scale++;
		aux.set(i,j,true);

		int x, y;
		for (x = i-1; x<=i+1; x++){
			for (y = j-1; y<=j+1; y++){				
				if (clustering(x,y,aux,cl,seq,backfront)){
					backfront = !backfront;
				}
			}
		}
		return true; 
	}
	else{
		aux.set(i,j,true);
		return false; 
	}
}

void binMap::set(const vector<clusterCell>& cl){
	clear();
	vector<clusterCell>::const_iterator clit;
	for(clit = cl.begin(); clit != cl.end(); clit++)
		set(clit->x, clit->y, true);
}

int binMap::count() const{
	int count=0;
	for (int i = getRoi().x ; i < getRoi().x + getRoi().width ; i++)
		for (int j = getRoi().y ; j < getRoi().y + getRoi().height ; j++)   // for each cell
			if (get(i,j)) count++;
	return count;
}

