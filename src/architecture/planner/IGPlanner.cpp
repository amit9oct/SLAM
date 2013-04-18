
#include "IGPlanner.h"
#include "gridMapInterface.h"
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <list>
#include <vector>
#include <opencv2/opencv.hpp>
#include "StringTokenizer.h"

// Para registrar la clase en la factoria
namespace planners{
	planner* CreateInformationGainPlanner(const ConfigFile& conf){
		return new InformationGainPlanner(conf);
	};
	const int IGPLANNER = 3 ;
	const bool registered = plannerFactory::Instance().Register(IGPLANNER, plannerCreator(CreateInformationGainPlanner));
}


using namespace std;

// Constructor
InformationGainPlanner::InformationGainPlanner(const ConfigFile& config):
	planner(),
	endPlanner(true),
	replanning_period(config.read<float>("REPLANNING_PERIOD")),
	inflate_obstacles(config.read<int>("INFLATE_OBSTACLES")),
	utility_radius(config.read<float>("UTILITY_RADIUS")),
	utility_weight(config.read<float>("UTILITY_WEIGHT")),
	cost_weight(config.read<float>("COST_WEIGHT"))
{}

// Destructor
InformationGainPlanner::~InformationGainPlanner(){stop();}

// thread Setup
int InformationGainPlanner::setup(){

//	printf("arranco planificador\n");
	completedPath = true;
	prio = 50;
	return 0;
}

// thread on stop
void InformationGainPlanner::onStop(){
	if (!endPlanner){
		endPlanner = true;
		closing.lock();
		closing.unlock();
	}
	printf("[CU-PLANNER] [robot %d] Stopped\n", rbase->getNumber());
}

// thread main execution
void InformationGainPlanner::execute(){
	printf("[CU-PLANNER] \t\t\t\t\t\t COST-UTILITY PLANNER (%d) running\n", rbase->getNumber());
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
//	
	bool endExploration = false;
	int nextPlanning=0;
	while(!endPlanner && !endExploration){
		
		// esperamos a SLAM
		nextStep=mySlam->getStep()+1;
		//printf("[CU-PLANNER] [robot %d planner] waiting for step %d\n", number, nextStep);
		mySlam->waitForStep(nextStep);

		if (nextStep > 20000) endExploration = true;

		// tomamos los datos
		gridMapInterface* omap  	= mySlam->getOMap();
		binMap* ppmap			= mySlam->getPreMap();
		//int numrobots			= mySlam->getNumRobots();
		point poscell			= mySlam->getCell(number);
		pose robotpos			= mySlam->getPos(number);
		
		// celda con orientacion
//		ocell ori;
//		ori.x = poscell.x;
//		ori.y = poscell.y;
//		float ang = robotpos.th;
//		ori.th = int(floor(ang/(PI/4)+0.5));
//		if (ori.th > 7) ori.th -=8;
//		if (ori.th < 0) ori.th +=8;

		IplImage* im = (showPlanner)? omap->getMapAsImage():0;

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

		bool obstructedPath;

		// si hemos llegado al ultimo destino planificamos
		if(completedPath == true  || nextStep == nextPlanning){
			nextPlanning = nextStep + (int)floor(replanning_period/mySlam->getSampleTime()+0.5);
			completedPath = false;
//			printf("[IGPLANNER] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PLANNING >>>> robot %d\n",number);
			// mapa de fronteras
			binMap objectives;
			vector<clusterCell> clusterList;
			omap->frontiers(objectives);
			objectives.cluster(clusterList,2*inflate_obstacles);
			objectives.set(clusterList);
			//objectives.showMap("FRONTERAS");
			
			// mapa de costes
			vector<point> targets;
			vector<point>::iterator targetIter;
//			printf("[IGPLANNER] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Creating COST MAP !!!!!!!!!!!\n");
			costMapSimple* cm = createCostMap(poscell,objectives,*omap,targets,inflate_obstacles);
//			printf("[IGPLANNER] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> COSTMAP Created !!!!!!!!!!!!!\n");
			
//			printf("[IGPLANNER] Número de objetivos: %d\n",(int)targets.size());
			
			// elegir destino
			point dest = poscell;
			float maxVal =  -9999999999999.9f;
			int util_cells = ((int)floor(utility_radius/omap->getResolution()+0.5));
			float maxutility =  util_cells*util_cells*PI;
			float maxcost = cm->getMaxCost();
			for (targetIter = targets.begin(); targetIter!= targets.end(); targetIter++){
				if (targetIter->x!=poscell.x && targetIter->y != poscell.y){
					binMap esz;
					omap->esz(targetIter->x,targetIter->y,esz,0,util_cells);
					float utilvalue = utility(*omap,esz);
					float cost = cm->getCost(targetIter->x,targetIter->y);
					float value = utility_weight*utilvalue/maxutility - cost_weight*cost/maxcost;
					//printf("(%d,%d) value = %f, utility = %f, cost= %f \n",targetIter->x,targetIter->y,value,utilvalue, cost);
					if (value > maxVal){
						maxVal = value;
						dest = *targetIter;
					}
				}
			}
			
			// recuperar el path
			//printf("origen  (%d,%d,%d)\n",ori.x,ori.y,ori.th);
			//printf("destino (%d,%d,%d)\n",dest.x,dest.y,dest.th);
			cm->getPath(dest,path);
			delete cm;
			
			if (targets.size()==0){
//				printf("[IGPLANNER] No path to frontiers, stopping exploration\n");
				endExploration = true;
				completedPath = true;
				char sms[100];
				sprintf(sms,"END %d", getDir()); 
				sendMessage(string(sms),-1);
			}
//			printf("[IGPLANNER] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PLANNING END >>>> robot %d\n",number);
		}

		// Seguimos con la ruta
		else {
	
			// verificamos que el path sea transitable
			bool obstructedPath = false;
			vector<point>::reverse_iterator riter;
			if(path.size()>1) { 
				for ( riter = path.rbegin(); riter != path.rend(); riter++ )
					if (omap->isoccupied(riter->x,riter->y)) { 
//						printf("[IGPLANNER] [robot %d] obstructed\n",rbase->getNumber()); 
						obstructedPath = true;
						break;
					}
			}
			// verificar distancia al destino || obstruccion para path completado
			if((sqrt(pow((float)(path.front().x-poscell.x),2) + pow((float)(path.front().y-poscell.y),2)) <= 3.0)  || obstructedPath){
//				printf("[IGPLANNER] [robot %d] completed\n",rbase->getNumber()); 
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
//					printf("[IGPLANNER] [robot %d] no visible cells\n",rbase->getNumber()); 
				}
				//if (showPlanner) cvReleaseImage(&im);

			}
//			printf("[IGPLANNER] [robot %d] next goal: (%d,%d) - current pos (%d,%d)\n",rbase->getNumber(),goal.x, goal.y, ori.x,ori.y);						
		}


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
//			printf("************************************** [robot %d] Following Path\n",number);

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

//	printf("[robot %d] stoping robot", number);
	reac->disableAll();
	reac->stop();

	closing.unlock();
	explorationFinished.step();
}


int InformationGainPlanner::utility(const gridMapInterface& omap, const binMap& esz){
	int value=0;	
	for (int i = esz.getRoi().x ; i < esz.getRoi().x + esz.getRoi().width ; i++){
		for (int j = esz.getRoi().y ; j < esz.getRoi().y + esz.getRoi().height ; j++){   // for each cell
			if (esz.get(i,j) && omap.isunknown(i,j)){
				value++;
			}
		}
	}
	return value;
}
