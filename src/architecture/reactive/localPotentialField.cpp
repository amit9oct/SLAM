#include <opencv2/opencv.hpp>
#include "localPotentialField.h"
#include <stdio.h>

localPotentialField::localPotentialField(int lzwidth, int lzheight):
	width(lzwidth),
	height(lzheight),
	m(lzwidth, lzheight)
{
}

localPotentialField::localPotentialField(const localPotentialField &lpf):
	width(lpf.width),
	height(lpf.height),
	m(lpf.m)
{
}

localPotentialField::localPotentialField(){
}

localPotentialField::~localPotentialField(){
}

localPotentialField& localPotentialField::operator=(const localPotentialField& p){
	m = ((localPotentialField*) &p)->m;
	return *this;
}

localPotentialField localPotentialField::operator*(const float& f){
	localPotentialField np(*this);
	np*=f;
	return np;
}

localPotentialField localPotentialField::operator+(const localPotentialField& p){
	localPotentialField np(*this);
	np += p;
	return np;
}

localPotentialField& localPotentialField::operator+=(const localPotentialField& p){
	m += ((localPotentialField*) &p)->m;
	return *this;
}

localPotentialField& localPotentialField::operator*=(const float& f){
	m *= f;
	return *this;
}

void localPotentialField::reset(){
	m.clear();
}

void localPotentialField::normalize(){
	
	float max=m.get(0,0);
	float min=m.get(0,0);
	int i,j;

	for (i = 0; i< width; i++){
		for(j = 0; j< height; j++){
			if (max<get(i,j)) max=get(i,j);
			if (min>get(i,j)) min=get(i,j);
		}
	}

	for (i = 0; i< width; i++)
		for(j = 0; j< height; j++){
			if(max>min)
				m.set(i,j,(m.get(i,j)-min)/(max-min));	
			else
				m.set(i,j,0);	
		}
}

void localPotentialField::savePotentialAsImage(char* file){

	IplImage* img = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1); 
	
	float max=m.get(0,0);
	float min=m.get(0,0);
	int i,j;

	for (i = 0; i< width; i++){
		for(j = 0; j< height; j++){
			if (max<get(i,j)) max=get(i,j);
			if (min>get(i,j)) min=get(i,j);
		}
	}

	for (i = 0; i< width; i++)
		for( j = 0; j< height; j++)
			((uchar*)(img->imageData + img->widthStep*(height-1-j)))[i] = (uchar) (255.0f*(m.get(i,j)-min)/(max-min));		

	cvSaveImage(file, img);

	cvReleaseImage(&img);
}

