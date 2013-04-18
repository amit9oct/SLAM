#include "occupancyGridMap.h"
#include <math.h> 
#include <stdio.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include "binMap.h"
#include "ClThread.h"

// Constructors
occupancyGridMap::occupancyGridMap():
	width(0),
	height(0),
	realWidth(0),
	realHeight(0),
	xorigin(0),
	yorigin(0),
	resolution(0)
{

}

occupancyGridMap::occupancyGridMap(float w, float h, float res, float x, float y):
	width((int)floor(w/res+0.5)),
	height((int)floor(h/res+0.5)),
	realWidth(w),
	realHeight(h),
	xorigin(x),
	yorigin(y),
	resolution(res)
{
	
}

gridMapInterface& occupancyGridMap::operator=(const gridMapInterface& gmap){
    assert(typeid(gmap) == typeid(*this));
    const occupancyGridMap& map = dynamic_cast<const occupancyGridMap&>(gmap);
	width = map.width;
	height = map.height;
	realWidth = map.realWidth;
	realHeight = map.realHeight;
	xorigin = map.xorigin;
	yorigin = map.yorigin;
	resolution = map.resolution;
	return *this;
}

occupancyGridMap::~occupancyGridMap(){};


// Search
int occupancyGridMap::frontiers(binMap& newfrontiers) const{
	int i,j;
	int nfront=0;

	newfrontiers.initialize(width,height,resolution, xorigin, yorigin);

	for (i =0 ; i < width ; i++){
		for (j = 0 ; j < height ; j++){  				// for each cell
			if (isfrontier(i,j)) {						// celda en la zona segura
				newfrontiers.set(i,j,true);
				nfront++;			
			}
		}
	}
	return nfront;
}

int occupancyGridMap::frontiersIn(const binMap& vz,  binMap& newfrontiers) const{
	int i,j;
	int nfront=0;

	newfrontiers.initialize(vz.getWidth(),vz.getHeight(),vz.getResolution(), vz.getXOrigin(), vz.getYOrigin());
	RoI roi = vz.getRoi();
	newfrontiers.setRoi(roi);

	for (i = roi.x ; i < roi.x + roi.width ; i++){
		for (j = roi.y ; j < roi.y + roi.height ; j++){  				// for each cell
			if (vz.get(i,j) && isfrontier(i,j)) {						// celda en la zona segura
				newfrontiers.set(i,j,true);
				nfront++;			
			}
		}
	}
	return nfront;
}

void occupancyGridMap::esz(int x, int y, binMap& map, int dilaterad, int max) const{
	float max2 = (float)(max*max);
//	printf("[ESZ] map initialize...\n");
	map.initialize(width, height, resolution, xorigin, yorigin);
//	printf("[ESZ] map initialize OK\n");
	RoI roi;
	roi.x = x;
	roi.y = y;
	roi.width = 1;
	roi.height = 1;
	map.set(x,y,true);

	// trazamos rayos desde el centro hasta los bordes de un cuadrado de area 2max x 2max
	int i,j,ii,jj;
	bool visible;
	// linea superior
	j = y-max;
//	printf("[ESZ] 0'\n");
	//printf("new safe zone\n");
	for (i = x-max; i <= x+max; i++){
		visible = true;
		int size;
//		printf("[ESZ] get line...\n");
		point* line = getLine(x,y,i,j,size);
//		printf("[ESZ] get line OK\n");
		//printf("new line\n");
		for (int k = 1; k < size; k++){
			ii = line[k].x; jj = line[k].y;
			if (ii>=0 && jj>=0 && ii<width && jj< height){
				if(visible == true && (isoccupied(ii,jj,dilaterad) || (x-ii)*(x-ii)+(y-jj)*(y-jj)>max2)){ visible = false; break;}
				//printf("[%d,%d] visible = %d\n",ii,jj,visible);
				map.set(ii,jj,visible);
				if (visible){
					if(ii > roi.x+roi.width-1)		roi.width = ii-roi.x+1;
					else if(ii < roi.x) 			{roi.width  += roi.x-ii; roi.x    = ii;}
					if(jj > roi.y+roi.height-1)		roi.height = jj-roi.y+1;
					else if(jj < roi.y) 			{roi.height += roi.y-jj; roi.y    = jj;}
				}
			}
		}
		delete[] line;
	}

	// linea inferior
	j = y+max;
//	printf("[ESZ] 1\n");
	for (i = x-max; i <= x+max; i++){
		visible = true;
		int size;
//		printf("[ESZ] get line...\n");
		point* line = getLine(x,y,i,j,size);
//		printf("[ESZ] get line OK\n");
		for (int k = 1; k < size; k++){
			ii = line[k].x; jj = line[k].y;
			if (ii>=0 && jj>=0 && ii<width && jj< height){
				if(visible == true && (isoccupied(ii,jj,dilaterad) || (x-ii)*(x-ii)+(y-jj)*(y-jj)>max2)) visible = false;
				map.set(ii,jj,visible);
				if (visible){
					if(ii > roi.x+roi.width-1)		roi.width = ii-roi.x+1;
					else if(ii < roi.x) 			{roi.width  += roi.x-ii; roi.x    = ii;}
					if(jj > roi.y+roi.height-1)		roi.height = jj-roi.y+1;
					else if(jj < roi.y) 			{roi.height += roi.y-jj; roi.y    = jj;}
				}
			}
		}
		delete[] line;
	}

	// linea derecha
	i = x-max;
//	printf("[ESZ] 2\n");
	for (j = y-max+1; j <= y+max-1; j++){
		visible = true;
		int size;
//		printf("[ESZ] get line...\n");
		point* line = getLine(x,y,i,j,size);
//		printf("[ESZ] get line OK\n");
		for (int k = 1; k < size; k++){
			ii = line[k].x; jj = line[k].y;
			if (ii>=0 && jj>=0 && ii<width && jj< height){
				if(visible == true && (isoccupied(ii,jj,dilaterad) || (x-ii)*(x-ii)+(y-jj)*(y-jj)>max2)) visible = false;
				map.set(ii,jj,visible);
				if (visible){
					if(ii > roi.x+roi.width-1)		roi.width = ii-roi.x+1;
					else if(ii < roi.x) 			{roi.width  += roi.x-ii; roi.x    = ii;}
					if(jj > roi.y+roi.height-1)		roi.height = jj-roi.y+1;
					else if(jj < roi.y) 			{roi.height += roi.y-jj; roi.y    = jj;}
				}
			}
		}
		delete[] line;
	}

	// linea izquierda
	i = x+max;
//	printf("[ESZ] 3\n");
	for (j = y-max+1; j <= y+max-1; j++){
		visible = true;
		int size;
//		printf("[ESZ] get line...\n");
		point* line = getLine(x,y,i,j,size);
//		printf("[ESZ] get line OK\n");
		for (int k = 1; k < size; k++){
			ii = line[k].x; jj = line[k].y;
			if (ii>=0 && jj>=0 && ii<width && jj< height){
				if(visible == true && (isoccupied(ii,jj,dilaterad) || (x-ii)*(x-ii)+(y-jj)*(y-jj)>max2)) visible = false;
				map.set(ii,jj,visible);
				if (visible){
					if(ii > roi.x+roi.width-1)		roi.width = ii-roi.x+1;
					else if(ii < roi.x) 			{roi.width  += roi.x-ii; roi.x    = ii;}
					if(jj > roi.y+roi.height-1)		roi.height = jj-roi.y+1;
					else if(jj < roi.y) 			{roi.height += roi.y-jj; roi.y    = jj;}
				}
			}
		}
		delete[] line;
	}
//	printf("[ESZ] 4\n");

	map.setRoi(roi);
	map.set(x,y,true);
//	printf("[ESZ] dilate...\n");
	map.dilate(dilaterad);
//	printf("[ESZ] dilate OK\n");
}

point* occupancyGridMap::getLine(int x1, int y1, int x2, int y2, int& points) const{
	int i,j,k;

	float incx, incy, auxx, auxy;
	auxx = fabs((float)(x1-x2))+0.0001;
	auxy = fabs((float)(y1-y2))+0.0001;
	incx = auxx/auxy;
	incy = auxy/auxx;

	int count = 0;
	point* pointbuf;

	if (incx > incy){
		j = y1;

		points = (int)(auxx+1);
		pointbuf = new point[points]();
		count=1;
		if (x1 < x2){
			for(i = x1,k=0; i <= x2; i++,k++){
				if (fabs((float)(i-x1))>=count*incx) {
					count++;
					(y1<y2)? j++ : j--;
				}
				pointbuf[k].x = i;
				pointbuf[k].y = j;
			}
		}
		else{
			for(i = x1,k=0; i >= x2; i--,k++){
				if (fabs((float)(x1-i))>=count*incx) {
					count++;
					(y1<y2)? j++ : j--;
				}
				pointbuf[k].x = i;
				pointbuf[k].y = j;
			}
		}
	}
	
	else{
		j = x1;
		points = (int)(auxy+1);
		pointbuf = new point[points]();

		count=1;
		if (y1 < y2){
			for(i = y1,k=0; i <= y2; i++,k++){
				if (fabs((float)(i-y1))>=count*incy) {
					count++;
					(x1<x2)? j++ : j--;
				}
				pointbuf[k].x = j;
				pointbuf[k].y = i;
			}
		}
		else{
			for(i = y1,k=0; i >= y2; i--,k++){
				if (fabs((float)(y1-i))>=count*incy) {
					count++;
					(x1<x2)? j++ : j--;
				}
				pointbuf[k].x = j;
				pointbuf[k].y = i;
			}
		}
	}
	if (points != k) printf("error!!!\n");
	return pointbuf;
}

void occupancyGridMap::gateways(const binMap& vz, binMap& newgateways) const {
	int i,j;
	newgateways.initialize(vz.getWidth(),vz.getHeight(),vz.getResolution(), vz.getXOrigin(), vz.getYOrigin());
	RoI roi = vz.getRoi();
	newgateways.setRoi(roi);
	
	for (i = roi.x ; i < roi.x + roi.width ; i++){
		for (j = roi.y ; j < roi.y + roi.height ; j++){  		 	// for each cell
			if (vz.get(i,j) && (!vz.isInsideD(i,j)) && isfreeD(i,j)) {	// celda en la zona segura
				newgateways.set(i,j,true);
			}
		}
	}
}

void occupancyGridMap::clearUnconnect(binMap& vz,int x, int y) const{
	RoI roi = vz.getRoi();
	for (int i = roi.x ; i < roi.x + roi.width ; i++)
		for (int j = roi.y ; j < roi.y + roi.height ; j++)  		 	// for each cell
			if (!isfree(i,j)) vz.set(i,j,false);
	vz.removeUnconnected(x,y);
}

void occupancyGridMap::occupiedCells(binMap& occupied, int dilateRad) const {
	int i,j;

	occupied.initialize(width,height,resolution, xorigin, yorigin);
	
	for (i = 0 ; i < width ; i++){
		for (j =0 ; j < height ; j++){  		 // for each cell
			if (isoccupied(i,j,dilateRad)){	// celda ocupada
				occupied.set(i,j,true);
			}
		}
	}
}

inline bool occupancyGridMap::isfree(int x, int y, int size) const {
	for (int i = x-size; i<= x+size; i++)
		for (int j = y-size; j<= y+size; j++)
			if (!isfree(i,j))
				return false;
	return true;
}

bool occupancyGridMap::isfreeD(int x, int y) const {
	int i,j;

	i = x; j = y;
	if (!isfree(i,j))	return false;

	i = x+1; j = y;
	if (!isfree(i,j))	return false;

	i = x-1; j = y;
	if (!isfree(i,j))	return false;

	i = x; j = y+1;
	if (!isfree(i,j))	return false;

	i = x; j = y-1;
	if (!isfree(i,j))	return false;

	return true;
}

bool occupancyGridMap::isoccupied(int x, int y, int size) const {
	for (int i = x-size; i<= x+size; i++){
		for (int j = y-size; j<= y+size; j++){
			if (isoccupied(i,j)) return true;
		}
	}
	return false;
}

float occupancyGridMap::getValue(int x, int y, int range) const {
	float val=0.0f;
	int count=0;	
	for (int i = x-range; i<= x+range; i++){
		for (int j = y-range; j<= y+range; j++){
			val+=getValue(i,j);
			count++;
		}
	}
	return val/count;
}

bool occupancyGridMap::isunknown(int x, int y, int size) const {
	for (int i = x-size; i<= x+size; i++)
		for (int j = y-size; j<= y+size; j++)
			if (isunknown(i,j))
				return true;
	return false;
}

void occupancyGridMap::saveMapAsImage(const char* file) const {
	IplImage* img = getMapAsImage();
	cvSaveImage(file, img);
	cvReleaseImage(&img);
}

IplImage* occupancyGridMap::getMapAsImage() const {
	IplImage* img = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3); 

	for (int i = 0; i< width; i++)
		for(int j = 0; j< height; j++){
			uchar greyvalue = (uchar) (255-2.55f*getValue(i,j));
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i] = greyvalue;		
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i+1] = greyvalue;		
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[3*i+2] = greyvalue;		
		}
	return img;
}

void occupancyGridMap::showMap(const char* windowname)const{
	IplImage* img = getMapAsImage();
	cvNamedWindow(windowname, 0);
	cvShowImage(windowname,img);
	cvReleaseImage(&img);
}

int occupancyGridMap::countAccessible(const point* p, int numpoints, binMap& accessible) const{
	// N = some floodfill algorithm counting
	//printf("[omap] [countAccessible] npoints :%d, map size (%d, %d)\n", numpoints, getWidth(), getHeight());
	accessible.initialize(getWidth(), getHeight());
	int N=0;
	for (int i=0; i< numpoints; i++){
	//	printf("[omap] [countAccessible] %d\n",i);
		accessible.set(p[i].x,p[i].y,true);
		N++;
		N+= floodfill(p[i].x,p[i].y,accessible);
	}
	//printf("[omap] [countAccessible] result %d\n",N);
	return N;
}

int occupancyGridMap::floodfill(const int& nx, const int& ny, binMap& processed) const{
	int N=0;
	//printf("[TARGETPROBMAPPER]  para el punto (%d, %d), acumulado: %d\n", nx,ny,N);
	//printf("[TARGETPROBMAPPER]  size (%d x %d)\n", processed.getWidth(), processed.getHeight());
	for (int x = nx-1; x <= nx+1; x++){
//		printf("[TARGETPROBMAPPER]  x=%d\n",x);
		if(x < 0 ||  x>= getWidth()-1) continue;
		for (int y = ny-1; y <= ny+1; y++){
//			printf("[TARGETPROBMAPPER]  y=%d\n",y);
			if(y < 0 || y >= getHeight()-1) continue;
//			printf("[TARGETPROBMAPPER]  considering point (%d, %d) processed= %d, is occupied= %d\n", x,y, processed.get(x,y), omap.isoccupied(x,y));
			if(!processed.get(x,y)){
				processed.set(x,y,true);
				if(!isoccupied(x,y)){
//					omap.showMap("proc");
//					printf("[TARGETPROBMAPPER]  iteration ... point (%d, %d) => (%d, %d)\n", nx, ny, x, y);	
//					cvWaitKey(2);
					N += floodfill(x, y, processed)+1;
//					printf("[TARGETPROBMAPPER]  N=%d\n",N);
				}
			}
		}
	}
	return N;
}

