
#include "FSAPlanner.h"
#include "gridMapInterface.h"
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <list>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "pathPlanning.h"
#include "StringTokenizer.h"

// Para registrar la clase en la factoria
namespace planners{
	planner* CreateFSAPlanner(const ConfigFile& conf){
		return new FSAPlanner(conf);
	};
	const int FSAPLANNER =	1 ;
	const bool registered = plannerFactory::Instance().Register(FSAPLANNER, plannerCreator(CreateFSAPlanner));
}


using namespace std;

// constructor
FSAPlanner::FSAPlanner(const ConfigFile& config):	
	planner(),
	inflate_obstacles(config.read<int>("INFLATE_OBSTACLES")),
	integrate_slam(config.read<bool>("INTEGRATE_SLAM")),
	escape_allowed(config.read<bool>("ESCAPE_FROM_LOCAL_MINIMA")),
	endPlanner(true)
{}

// destructor
FSAPlanner::~FSAPlanner(){stop();}

// thread setup
int FSAPlanner::setup(){
//	printf("arranco planificador\n");
	prio = 50;
	completedPath = true;
	return 0;
}

// thread on stop
void FSAPlanner::onStop(){
	if (!endPlanner){
		endPlanner = true;
		closing.lock();
		closing.unlock();
	}
	printf("[BEHAVIOUR_BASED_PLANNER] [robot %d] Stopped\n", rbase->getNumber());
}

// thread main code
void FSAPlanner::execute(){
	printf("[BEHAVIOUR_BASED_PLANNER] \t\t\t\t BEHAVIOUR_BASED_PLANNER (%d) running\n", rbase->getNumber());
	closing.lock();
	endPlanner = false;
	point goal;
	int nextStep=number;

	// initial state
	exploring = true;
	returning = false;
	escapingFromLocalMinimum = false;
	returningEscape = false;

	// init logs
	FILE* plannerlog=0;
	if (logstr){
		char myfilestr[100];
		sprintf(myfilestr,"%splanR%d.log",logstr,number);
		plannerlog = fopen(myfilestr,"w");
	}

//	char mystr[250];
//	if (showPlanner){
//		sprintf(mystr,"plan%d.jpg",number);
//		cvNamedWindow(mystr,0);
//	}

	bool endExploration = false;
	while(!endPlanner && !endExploration){
		
		nextStep += 1;
//		printf("[robot %d planner] waiting for step %d\n", number, nextStep);
		mySlam->waitForStep(nextStep);

		if (nextStep > 20000) endExploration= true;

		gridMapInterface* omap	= mySlam->getOMap();
		binMap* ppmap			= mySlam->getPreMap();
		//int numrobots			= mySlam->getNumRobots();
		point poscell			= mySlam->getCell(number);

//		printf("[robot %d] Dispersion: %f\n", number, mySlam->getDisp(number));


		// leer mensajes
		string msg;
		int msglen;
		while((msglen=getMessage(msg))){
			//cout << "Message: >> " << msg << endl;
			StringTokenizer st(&msg[0]);
			string messagetype(st.nextToken());
			if (messagetype == "END"){
				endExploration = true;
				completedPath = true;
			}
		}
		if (completedPath){
			exploring = true;
			returning = false;
			escapingFromLocalMinimum = false;
			returningEscape = false;
			completedPath = false;
		}
		if (integrate_slam){ 
			if ((exploring || escapingFromLocalMinimum)  && mySlam->getLoc(number)){
				exploring = false;
				returning = true;
				escapingFromLocalMinimum = false;
				returningEscape = false;
				path.clear();
			}
			if ((returning || returningEscape) && !mySlam->getLoc(number)){
				exploring = true;
				returning = false;
				escapingFromLocalMinimum = false;
				returningEscape = false;
				path.clear();
			}
		}

		// si hay minimo planificamos
		if (escape_allowed){
			if( (exploring || returning) && reac->localMinimum()){
	//			printf("[FSAPlanner] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Planificando >>>> robot %d\n",number);
		
				int lenpath;
				if(exploring){
					binMap objectives;
					vector<clusterCell> clusterList;
					omap->frontiers(objectives);
					objectives.cluster(clusterList,4);
					objectives.set(clusterList);
					lenpath = DijkstraGrid(poscell, objectives, *omap, path, inflate_obstacles);	// path to frontier
				}
				else{ // returning 
					lenpath = DijkstraGrid(poscell, *ppmap, *omap, path, inflate_obstacles);	// path to precise pose
				}

				if (lenpath == 0){
	//				printf("No path found, stopping exploration\n");
					endExploration = true;
					completedPath = true;
					char sms[100];
					sprintf(sms,"END %d", getDir()); 
					sendMessage(string(sms),-1);
				}
				else{
					if(exploring){
						escapingFromLocalMinimum = true;
						returningEscape = false;
					}
					else{
						escapingFromLocalMinimum = false;
						returningEscape = true;
					}
					exploring = false;
					returning = false;
				}
	//			printf("[FSAPlanner] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Planning end. len = %d >>>> robot %d\n",path.size(), number);
			}
		}
//		if (showPlanner){	// Visualizacion del path
//			IplImage* im = omap->getMapAsImage();
//			for (unsigned int i = 0;i < path.size(); i++)
//				cvCircle(im,cvPoint(path[i].x, omap->getHeight()-1-path[i].y),0,CV_RGB(255,0,0),-1);
//			cvShowImage(mystr,im);
//			cvWaitKey(5);
//			cvReleaseImage(&im);
//		}


		// si estamos en planificado quitamos los puntos que hayamos recorrido
		if (escapingFromLocalMinimum || returningEscape){
	
			// verificamos que el path sea transitable
			bool obstructedPath = false;
			vector<point>::reverse_iterator riter;
			if(path.size()>1) { 
				for ( riter = path.rbegin(); riter != path.rend(); riter++ )
					if (omap->isoccupied(riter->x,riter->y)) { 
//						printf("[FSAPlanner] [robot %d] obstructed\n",rbase->getNumber()); 
						obstructedPath = true;
						break;
					}
			}

			// verificar distancia al destino || obstruccion para path completado
			if((sqrt(pow((float)(path.front().x-poscell.x),2) + pow((float)(path.front().y-poscell.y),2)) <= 3.0)  || obstructedPath){
//				printf("[FSAPlanner] [robot %d] completed\n",rbase->getNumber()); 
				completedPath = true;
			}
			else { // No hemos llegado al final seguimos avanzando
				bool found=false;				

				//IplImage* im = (showPlanner)? omap->getMapAsImage() : 0;
				//if (showPlanner){
				//	cvCircle(im,cvPoint(poscell.x, omap->getHeight()-1-poscell.y),0,CV_RGB(255,0,0),-1); //posicion rojo	
				//	for (unsigned int i = 0;i < path.size(); i++)
				//		cvCircle(im,cvPoint(path[i].x, omap->getHeight()-1-path[i].y),0,CV_RGB(255,0,255),-1); // path magenta
				//	cvShowImage(mywindowstr,im);
				//}
				
				if(path.size()>=1) {
					for ( riter = path.rbegin(); riter != path.rend(); riter++ ){
						bool visible = true;
						int size;
						point* line = omap->getLine(poscell.x,poscell.y,riter->x,riter->y,size);
						
						for (int p = 0; p < size; p++){
							if (p > 1 && omap->isoccupied(line[p].x,line[p].y,1)){ 
								visible = false; 
								break;
							}
						//	else
						//		cvCircle(im,cvPoint(line[p].x, omap->getHeight()-1-line[p].y),0,CV_RGB(0,255,0),-1); // visual green
						}
						//cvShowImage(mywindowstr,im);
						//cvWaitKey(2);
						delete [] line;
						if (visible) { // is visible
							goal.x = riter->x;
							goal.y = riter->y;
							if (found) path.pop_back();
							else found = true;
							//path.pop_back();
							double dist = sqrt(pow(poscell.x-goal.x,2) + pow(poscell.y-goal.y,2));
							if ( dist > 1.5/omap->getResolution()) break;
						}
						else break;
					}
				}
				if(!found){
					completedPath = true;  // algo falla ninguna celda de la ruta es visible
//					printf("[FSAPlanner] [robot %d] no visible cells\n",rbase->getNumber()); 
				}
				//if (showPlanner) cvReleaseImage(&im);

			}
//			printf("[FSAPlanner] [robot %d] next goal: (%d,%d) - current pos (%d,%d)\n",rbase->getNumber(),goal.x, goal.y, poscell.x,poscell.y);						
		}
	
		delete omap;
		delete ppmap;

		//	modificar los enables/goal de la parte reactiva
		if (exploring){
//			printf("[FSAPlanner] ************************************** [robot %d] Reactive\n",number);
		
			reac->enableAvoidObstacles();
			reac->enableAvoidOtherRobots();
			reac->enableGoToFrontier();

			reac->disableGoToUnexploredZones();
			reac->disableGoToGoal();
			reac->disableGoToPrecisePoses();
		}
		else if (returning){
//			printf("[FSAPlanner] ************************************** [robot %d] Returning\n",number);

			reac->enableAvoidObstacles();
			reac->enableGoToPrecisePoses();

			reac->disableAvoidOtherRobots();			
			reac->disableGoToFrontier();
			reac->disableGoToUnexploredZones();
			reac->disableGoToGoal();
		}
		else if (escapingFromLocalMinimum || returningEscape){
//			printf("[FSAPlanner] ************************************** [robot %d] Escaping from local minimum\n",number);

			reac->setGoal(goal);

			reac->enableAvoidObstacles();
			reac->enableGoToGoal();

			reac->disableAvoidOtherRobots();
			reac->disableGoToFrontier();
			reac->disableGoToUnexploredZones();
			reac->disableGoToPrecisePoses();			
		}

		// write logs
		if (logstr){
			fprintf(plannerlog,"%d, %d, %d, %d, %d, %d, %d\n",nextStep, exploring, returning, escapingFromLocalMinimum, returningEscape, poscell.x, poscell.y);
		}
	}

//	if (showPlanner) cvDestroyWindow(mystr);

	//closing logs
	if (logstr){
		fclose(plannerlog);
	}

//	printf("[robot %d] stoping robot", number);
	reac->disableAll();
	reac->stop();

	closing.unlock();
	explorationFinished.step();

}
