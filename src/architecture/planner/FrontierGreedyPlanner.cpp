
#include "FrontierGreedyPlanner.h"
#include "gridMapInterface.h"
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <list>
#include <vector>
#include <opencv2/opencv.hpp>
#include "StringTokenizer.h"

#define DEBUG false

// Para registrar la clase en la factoria
namespace planners{
	planner* CreateFrontierGreedyPlanner(const ConfigFile& conf){
		return new FrontierGreedyPlanner(conf);
	};
	const int FRONTIERGREEDYPLANNER = 2 ;
	const bool registered = plannerFactory::Instance().Register(FRONTIERGREEDYPLANNER, plannerCreator(CreateFrontierGreedyPlanner));
}

using namespace std;

// Constructor
FrontierGreedyPlanner::FrontierGreedyPlanner(const ConfigFile& config):
	planner(),
	replanning_period(config.read<float>("REPLANNING_PERIOD")),
	inflate_obstacles(config.read<int>("INFLATE_OBSTACLES")),
	endPlanner(true)
{}

// Destructor
FrontierGreedyPlanner::~FrontierGreedyPlanner(){stop();}

// thread Setup
int FrontierGreedyPlanner::setup(){
	completedPath = true;
	prio = 50;
	return 0;
}

// thread on stop
void FrontierGreedyPlanner::onStop(){
	if (!endPlanner){
		endPlanner = true;
		closing.lock();
		closing.unlock();
	}
	printf("[GREEDYPLANNER] [robot %d] Stopped\n", rbase->getNumber());
}

// thread main execution
void FrontierGreedyPlanner::execute(){
	printf("[GREEDYPLANNER] \t\t\t\t\t GREEDYPLANNER (%d) running\n", rbase->getNumber());

	closing.lock();
	endPlanner = false;
	point goal;	
	int nextStep=number;

	FILE* plannerlog=0;
	if (logstr){
		char myfilestr[100];
		sprintf(myfilestr,"%splanR%d.log",logstr,number);	
		plannerlog = fopen(myfilestr,"w");
	}

//	char mywindowstr[250];
//	if (showPlanner){
//		sprintf(mywindowstr,"plan%d.jpg",number);
//		cvNamedWindow(mywindowstr,0);
//	}

	bool endExploration = false;
	int nextPlanning=0;

	while(!endPlanner && !endExploration){
		
		// esperamos a SLAM
		nextStep=mySlam->getStep()+1;
		//if (DEBUG) printf("[GREEDYPLANNER] [robot %d] waiting for step %d\n", rbase->getNumber(), nextStep);
		mySlam->waitForStep(nextStep);

		if (nextStep > 20000) endExploration = true;

		// tomamos los datos
		gridMapInterface* omap  = mySlam->getOMap();
		binMap* ppmap		= mySlam->getPreMap();
		// int numrobots = 	mySlam->getNumRobots();
		point poscell = 	mySlam->getCell(number);
		pose robotpos = 	mySlam->getPos(number);

		// celda con orientacion
//		ocell ori;
//		ori.x = poscell.x;
//		ori.y = poscell.y;
//		float ang = robotpos.th;
//		ori.th = int(floor(ang/(PI/4)+0.5));
//		if (ori.th > 7) ori.th -=8;
//		if (ori.th < 0) ori.th +=8;

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

//		IplImage* im = (showPlanner)? omap->getMapAsImage():0;
		
		// si hemos llegado al ultimo destino planificamos
		if(completedPath == true  || nextStep >= nextPlanning){
			nextPlanning = nextStep + (int)floor(replanning_period/mySlam->getSampleTime()+0.5);
			completedPath = false;

			binMap objectives;
			vector<clusterCell> clusterList;
			omap->frontiers(objectives);
			objectives.cluster(clusterList,2*inflate_obstacles);
			objectives.set(clusterList);

//			printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DIJKSTRA >>>> robot %d\n",number);
//			int lenpath = DijkstraGridWithHeading(ori, objectives,*omap,path);
			int lenpath = DijkstraGrid(poscell, objectives,*omap,path, inflate_obstacles); // ojo!!! la lista va del destino al origen!!!
//			for ( int j = 0; j < lenpath; j++) if (DEBUG) printf("(%d,%d), ",path[j].x, path[j].y);
//			if (DEBUG) printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> FIN DE LA PLANIFICACION: length: %d >>>> robot %d\n",lenpath,number);
//			if (DEBUG) printf("[GREEDYPLANNER] [robot %d] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> FIN DE LA PLANIFICACION: length: %d \n",rbase->getNumber(),lenpath);

			if (lenpath == 0){
//				if (DEBUG) printf("No path to frontiers, stopping exploration\n");
				endExploration = true;
				completedPath = true;
				char sms[100];
				sprintf(sms,"END %d", getDir()); 
				sendMessage(string(sms),-1);
			}
		}
		// Seguimos con la ruta
		else {
	
			// verificamos que el path sea transitable
			bool obstructedPath = false;
			vector<point>::reverse_iterator riter;
			if(path.size()>1) { 
				for ( riter = path.rbegin(); riter != path.rend(); riter++ )
					if (omap->isoccupied(riter->x,riter->y)) { 
//						if (DEBUG) printf("[GREEDYPLANNER] [robot %d] obstructed\n",rbase->getNumber()); 
						obstructedPath = true;
						break;
					}
			}
			// verificar distancia al destino || obstruccion para path completado
			if((sqrt(pow((float)(path.front().x-poscell.x),2) + pow((float)(path.front().y-poscell.y),2)) <= 3.0)  || obstructedPath){
//				if (DEBUG) printf("[GREEDYPLANNER] [robot %d] completed\n",rbase->getNumber()); 
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
//					if (DEBUG) printf("[GREEDYPLANNER] [robot %d] no visible cells\n",rbase->getNumber()); 
				}
				//if (showPlanner) cvReleaseImage(&im);

			}
//			if (DEBUG) printf("[GREEDYPLANNER] [robot %d] next goal: (%d,%d) - current pos (%d,%d)\n",rbase->getNumber(),goal.x, goal.y, ori.x,ori.y);						
		}


/*
		// Quitamos los puntos que hayamos recorrido
		else {
			// verificamos que el path sea transitable
			bool obstructedPath = false;
			//vector<ocell>::iterator iter;
			vector<point>::iterator iter;
			if(path.size()>1) { 
				for ( iter = path.end()-1; iter != path.begin(); iter-- )
					if (omap->isoccupied(iter->x,iter->y)) { 
						if (DEBUG) printf("[GREEDYPLANNER] [robot %d] obstructed\n",rbase->getNumber()); 
						obstructedPath = true;
						break;
					}			
			}
			// verificar distancia al destino || obstruccion para path completado
			if((sqrt(pow((float)(path.front().x-poscell.x),2) + pow((float)(path.front().y-poscell.y),2)) < 3.0)  || obstructedPath){
				if (DEBUG) printf("[GREEDYPLANNER] [robot %d] completed\n",rbase->getNumber()); 
				completedPath = true;
			}
			else {
				bool found=false;
				if(path.size()>1 && vmax>0) {
					for ( iter = path.end()-1; iter != path.begin(); iter-- ){
						bool visible = true;
						int size,p;
						point* line = omap->getLine(iter->x,iter->y,poscell.x,poscell.y,size);
						for (p = 0; p < size; p++){
							if (omap->isoccupied(line[p].x,line[p].y,2)){ visible = false; break;}
						}
						delete [] line;
						if (visible) { // is visible
							goal.x = iter->x;
							goal.y = iter->y;
							found = true;
							int dist = fabs(poscell.x-goal.x) + fabs(poscell.y-goal.y);
							if ( dist < 1.0/omap->getResolution()) {
								path.erase(iter);
								iter--;
							}
							if ( dist > 10.0/omap->getResolution()) break;
						}
						else break;
					}
				}
				if(!found){
					completedPath = true;  // algo falla ninguna celda de la ruta es visible
					if (DEBUG) printf("[GREEDYPLANNER] [robot %d] no visible cells\n",rbase->getNumber()); 
				}
			}
			if (DEBUG) printf("[GREEDYPLANNER] [robot %d] next goal: (%d,%d)\n",rbase->getNumber(),goal.x, goal.y);						

		} */
		// visualizacion
//		if (showPlanner){
//			cvCircle(im,cvPoint(poscell.x, omap->getHeight()-1-poscell.y),0,CV_RGB(255,0,0),-1); //posicion rojo	
//			cvCircle(im,cvPoint(poscell.x+avancex[ori.th], omap->getHeight()-1-poscell.y-avancey[ori.th]),0,CV_RGB(0,0,255),-1); // orientacion azul
//			for (unsigned int i = 0;i < path.size(); i++)
//				cvCircle(im,cvPoint(path[i].x, omap->getHeight()-1-path[i].y),0,CV_RGB(255,0,255),-1); // path magenta
//			cvShowImage(mywindowstr,im);
//			cvWaitKey(5);
//			cvReleaseImage(&im);
//		}

		if (omap) delete omap;
		if (ppmap) delete ppmap;

		//	modificar los enables/goal de la parte reactiva
		if (completedPath)	reac->disableAll();				// paramos el robot hasta planificar de nuevo
		else{
//			if (DEBUG) printf("[GREEDYPLANNER] [robot %d] **************************************  Following Path\n",rbase->getNumber());

			reac->setGoal(goal);
			reac->enableGoToGoal();
			reac->enableAvoidObstacles();
			reac->disableAvoidOtherRobots();
			reac->disableGoToFrontier();
			reac->disableGoToUnexploredZones();
			reac->disableGoToPrecisePoses();	
		}

		// write logs
		if (logstr)	fprintf(plannerlog,"%d, %d, %d, %d\n", nextStep, completedPath, poscell.x, poscell.y);
	}

//	if (showPlanner) cvDestroyWindow(mywindowstr);

	//closing logs
	if (logstr)	fclose(plannerlog);

//	if (DEBUG) printf("[GREEDYPLANNER] [robot %d] stoping robot...\n", rbase->getNumber());
	reac->disableAll();
	reac->stop();
	closing.unlock();
	explorationFinished.step();
//	if (DEBUG) printf("[GREEDYPLANNER] \t\t\t\t\t\t\t [robot %d] robot planner stopped\n", rbase->getNumber());
}

