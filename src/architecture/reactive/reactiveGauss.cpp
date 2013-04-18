#include "reactiveGauss.h"
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#ifndef WIN32 
#include <sys/time.h>
#endif


// Para registrar la clase en la factoria
namespace reactiveLayers{
	reactive* CreateReactiveGauss(const ConfigFile& conf){
		return new reactiveGauss(conf);
	};
	const int REACTIVEGAUSS	= 0;
	const bool registered = reactiveFactory::Instance().Register(REACTIVEGAUSS, reactiveCreator(CreateReactiveGauss));
}

using namespace std;

reactiveGauss::reactiveGauss(const ConfigFile& config):
	reactive			(config),
	useesz				(config.read<bool>("USEESZ")),
	eszdilationradius		(config.read<int>("ESZDILATIONRADIUS")),
	actionradius			(config.read<float>("ESZRADIUS")),
	endReactive			(true),
	lzwidth				(config.read<int>("LOCALPOTENTIALWIDTH")),
	lzheight			(config.read<int>("LOCALPOTENTIALHEIGHT")),
	total				(lzwidth,lzheight)
{
	
}

reactiveGauss::~reactiveGauss(){
	stop();
}

// codigo del hilo se arranca/para con run/stop

// si no devuelve 0 el hilo no empieza
int reactiveGauss::setup(){
	prio = 50;
	return 0;
}

// se ejecuta al cancelar la ejecucion del hilo por el metodo stop
void reactiveGauss::onStop(){
	if (!endReactive){
		endReactive = true;
		closing.lock();
		closing.unlock();
	}
	printf("[REACTIVEGAUSS] [robot %d] Stopped\n", rbase->getNumber());
	if (rbase) rbase->setSpeed(0,0);
}

// el codigo a ejecutar por el hilo
void reactiveGauss::execute(){
	printf("[REACTIVEGAUSS] \t\t\t\t\t REACTIVEGAUSS (%d) running\n",  rbase->getNumber());

	closing.lock();
	endReactive=false;

	int nextStep;
#ifdef SHOWPOTENTIAL
	char mystr[250];
	char mystr2[250];
	sprintf(mystr,"POT%d",number);
	sprintf(mystr2,"POT%d.jpg",number);
	cvNamedWindow(mystr,0);
#endif
	nextStep=0;

	while(!endReactive){
			
		nextStep = mySlam->getStep() +1;
		//printf("[REACTIVEGAUSS]  [robot %d reactive] waiting for step %d\n", rbase->getNumber(), nextStep);		
		mySlam->waitForStep(nextStep);
		updateField(total);
		pointf f = getResponse(total);
		speed vel = regulator(currentPos,f);
		rbase->setSpeed(vel.v, vel.w);
		//usleep(100000); // simulate command transmision delay

		//printf("[REACTIVEGAUSS] [robot %d] vel: %f, %f \n", rbase->getNumber(), vel.v, vel.w);

#ifdef SHOWPOTENTIAL		
		total.savePotentialAsImage(mystr2);
		IplImage* im = cvLoadImage(mystr2,3);
		cvShowImage(mystr,im);
		cvWaitKey(10);
		cvReleaseImage(&im);
#endif

	}

#ifdef SHOWPOTENTIAL
	cvDestroyWindow(mystr);
#endif

	//printf("[REACTIVEGAUSS] [robot %d reac] Stopping reactive...\n", rbase->getNumber());
	closing.unlock();
	//printf("[REACTIVEGAUSS] \t\t\t\t\t\t\t [robot %d] reac stopped\n", rbase->getNumber());

}

void reactiveGauss::updateField (localPotentialField& global){
	
	global.reset();

	gridMapInterface* omap		= mySlam->getOMap();
	binMap *ppmap			= mySlam->getPreMap();
	int numrobots			= mySlam->getNumRobots();
	point poscell			= mySlam->getCell(number);
	currentPos			= mySlam->getPos(number);

	point* robotcells = new point[numrobots]();
	for(int r=0; r<numrobots; r++)
		robotcells[r] = mySlam->getCell(r);
	binMap esz;
	
	int actionradius_cells = ((int)floor(actionradius/omap->getResolution()+0.5));

	if (useesz){
		omap->esz(poscell.x, poscell.y, esz, eszdilationradius,actionradius_cells);
	}
	else{
		esz.initialize(omap->getWidth(), omap->getHeight(), omap->getResolution(), omap->getXOrigin(), omap->getYOrigin());
		for(int i=poscell.x-actionradius_cells; i < poscell.x+actionradius_cells; i++){
			for(int j=poscell.y-actionradius_cells; j < poscell.y+actionradius_cells; j++){
				esz.set(i,j,true);
			}
		}
		RoI roi;
		roi.x      = (poscell.x-actionradius_cells>0)?                  poscell.x-actionradius_cells                      : roi.x = 0;
		roi.width  = (poscell.x+actionradius_cells<omap->getWidth())?   poscell.x+actionradius_cells - roi.x              : omap->getWidth() - roi.x;
		roi.y      = (poscell.y-actionradius_cells>0)?                  roi.y = poscell.y-actionradius_cells              : roi.y = 0;
		roi.height = (poscell.y+actionradius_cells< omap->getHeight())? roi.height = poscell.y+actionradius_cells - roi.y : roi.height = omap->getHeight() - roi.y;
		esz.setRoi(roi);
	}

	//printf("[REACTIVEGAUSS] ESZ ROI: %i, %i, %i, %i\n", esz.getRoi().x, esz.getRoi().y, esz.getRoi().width, esz.getRoi().height);

	localPotentialField* avObs = new localPotentialField(lzwidth,lzheight); 
	localPotentialField* goFro = new localPotentialField(lzwidth,lzheight); 
	localPotentialField* goUZ  = new localPotentialField(lzwidth,lzheight); 

	processOMap(poscell, *omap, esz, *avObs, *goFro, *goUZ);
	
	global += *avObs * avoidObstaclesWeight;
	global += *goFro * goToFrontierWeight;
	global += *goUZ  * goToUnexploredZonesWeight;

	delete avObs;
	delete goFro;
	delete goUZ;

	localPotentialField* avRob = new localPotentialField(lzwidth,lzheight); 
	processAvRob(numrobots, robotcells, esz, *avRob);
	global += *avRob * avoidOtherRobotsWeight;
	delete avRob;

	localPotentialField* goPre = new localPotentialField(lzwidth,lzheight); 
	processGoPre(poscell, *ppmap, esz, *goPre);
	global += *goPre * goToPrecisePosesWeight;
	delete goPre;

	localPotentialField* goGoal = new localPotentialField(lzwidth,lzheight);
	processGoGoal(poscell, goal, esz, *goGoal);
	global += *goGoal * goToGoalWeight;	
	delete goGoal;

	delete[] robotcells;
	delete omap;
	delete ppmap;

}

void reactiveGauss::processOMap(point& pos, gridMapInterface& omap, binMap& esz, localPotentialField& avObs, localPotentialField& goFro, localPotentialField& goUZ){
	
	if(goToUnexploredZonesEnabled || goToFrontierEnabled || avoidObstaclesEnabled){

		int i,j,ci,cj,x,y;
		int wws = (lzwidth-1)/2;
		int whs = (lzheight-1)/2;
		//printf("[REACTIVEGAUSS] %d, %d, %d, %d\n", esz.getRoi().x, esz.getRoi().y,esz.getRoi().width,esz.getRoi().height);
		for (i = esz.getRoi().x-1; i <= esz.getRoi().x + esz.getRoi().width ; i++){
			for (j = esz.getRoi().y-1; j <= esz.getRoi().y + esz.getRoi().height ; j++){			

				if(omap.isunknown(i,j) && goToUnexploredZonesEnabled) {					// unexplored
					if (esz.get(i,j)){ // if esz

						for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
							for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				

								goUZ.set(x,y, goUZ.get(x,y)-gauss(ci,cj, (float)i,(float)j, goToUnexploredZonesWidth_cells));
							}
						}
					}
				}
				else if(omap.isfrontier(i,j) && goToFrontierEnabled) {				// frontier
					if (esz.get(i,j)){ // if esz
						for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
							for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
								goFro.set(x,y, goFro.get(x,y)-gauss(ci,cj,(float) i,(float)j, goToFrontierWidth_cells));
							}
						}
					}
				}
				else if (omap.isoccupied(i,j) && avoidObstaclesEnabled) {										// occupied
					if (esz.isOver(i,j,eszdilationradius)){ // if next to esz
						for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
							for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
								avObs.set(x,y, avObs.get(x,y)+ gauss(ci,cj,(float) i,(float)j, avoidObstaclesWidth_cells));
							}
						}
					}
				}
			}
		}
	}

	avObs.normalize();
	goFro.normalize();
	goUZ.normalize();
}

void reactiveGauss::processGoPre(point& pos, binMap& ppmap, binMap&  esz, localPotentialField& goPre){

	if(goToPrecisePosesEnabled){
		int i,j,ci,cj,x,y;
		int wws = (lzwidth-1)/2;
		int whs = (lzheight-1)/2;	

		for (i = esz.getRoi().x ; i < esz.getRoi().x + esz.getRoi().width-1 ; i++){
			for (j = esz.getRoi().y ; j < esz.getRoi().y + esz.getRoi().height-1 ; j++){			

				if(ppmap.get(i,j) && esz.get(i,j)) {	
					for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
						for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
							goPre.set(x,y, goPre.get(x,y)-gauss(ci,cj, (float)i,(float)j, goToPrecisePosesWidth_cells));
						}
					}
				}
			}
		}
	}
	goPre.normalize();
}

void reactiveGauss::processGoGoal(point& pos, point& goal, binMap& esz, localPotentialField& goGoal){
	if(goToGoalEnabled){
		int ci,cj,x,y;
		int wws = (lzwidth-1)/2;
		int whs = (lzheight-1)/2;
	//	if(esz.get(goal.x,goal.y)) {	
			for (x = 0, ci = pos.x - wws ; ci <= pos.x + wws ; ci++, x++){
				for (y = 0, cj = pos.y - whs ; cj <= pos.y + whs ; cj++, y++){   // for each local cell				
					goGoal.set(x,y, goGoal.get(x,y)-gauss(ci,cj, goal.x,goal.y, goToGoalWidth_cells));
				}
			}
	//	}
	}
	goGoal.normalize();
}

void reactiveGauss::processAvRob(int numrobots, point* robotcells, binMap& esz, localPotentialField& avRob){
	if(avoidOtherRobotsEnabled){
		
		if (numrobots>1){

			int r, r2, ci,cj, x,y; 

			int wws = (lzwidth-1)/2;
			int whs = (lzheight-1)/2;
			
			r=number;
			for (r2 = 0; r2<numrobots; r2++){  // for each other robot r2
				if (r2 != r){
					if (esz.get(robotcells[r2].x, robotcells[r2].y)){	
						for (x = 0, ci = robotcells[r].x - wws ; ci <= robotcells[r].x + wws ; ci++, x++){
							for ( y = 0, cj = robotcells[r].y - whs ; cj <= robotcells[r].y + whs ; cj++, y++){   // for each local cell				
								avRob.set(x,y, avRob.get(x,y)+gauss(ci,cj, robotcells[r2].x, robotcells[r2].y, avoidOtherRobotsWidth_cells));
							}
						} // local cell j
					}
				}// r2 != r
			} // other robot
		}
	}
	avRob.normalize();
}

void reactiveGauss::setGoal(point goal)			{this->goal = goal;}

