
#include "IntegratedIGPlanner.h"
#include "gridMapInterface.h"
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <list>
#include <vector>
#include <opencv2/opencv.hpp>
#include "StringTokenizer.h"
#include "matFuns.h"
#include "robotTypes.h"

// Para registrar la clase en la factoria
namespace planners{
	planner* CreateIntegratedIGPlanner(const ConfigFile& conf){
		return new IntegratedIGPlanner(conf);
	};
	const int INTEGRATEDIGPLANNER =	6 ;
	const bool registered = plannerFactory::Instance().Register(INTEGRATEDIGPLANNER, plannerCreator(CreateIntegratedIGPlanner));
}

using namespace std;

// Constructor
IntegratedIGPlanner::IntegratedIGPlanner(const ConfigFile& config):
	planner(),
	//policy(0),
	endPlanner(true),
	replanning_period(config.read<float>("REPLANNING_PERIOD")),
	inflate_obstacles(config.read<int>("INFLATE_OBSTACLES")),
	utility_radius(config.read<float>("UTILITY_RADIUS")),
	utility_weight(config.read<float>("UTILITY_WEIGHT")),
	cost_weight(config.read<float>("COST_WEIGHT")),
	localization_weight(config.read<float>("LOCALIZATION_WEIGHT")),
	camrange(config.read<float>("CAMERA_RANGE"))
{}

// Destructor
IntegratedIGPlanner::~IntegratedIGPlanner(){stop();}

// thread Setup
int IntegratedIGPlanner::setup(){

//	printf("arranco planificador\n");
	completedPath = true;
	prio = 50;
	return 0;
}

// thread on stop
void IntegratedIGPlanner::onStop(){
	if (!endPlanner){
		endPlanner = true;
		closing.lock();
		closing.unlock();
	}
	printf("[INTEGRATED_PLANNER] [robot %d] Stopped\n", rbase->getNumber());
}

// thread main execution
void IntegratedIGPlanner::execute(){
	printf("[INTEGRATED_PLANNER] \t\t\t\t\t INTEGRATED_PLANNER (%d) running\n", rbase->getNumber());
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
//		printf("[INTEGRATEDIGPLANNER] [robot %d planner] waiting for step %d\n", number, nextStep);
		mySlam->waitForStep(nextStep);

		if (nextStep > 20000) endExploration = true;

		// tomamos los datos
		gridMapInterface* omap  	= mySlam->getOMap();
		binMap* ppmap			= mySlam->getPreMap();
		//int numrobots			= mySlam->getNumRobots();
		point poscell			= mySlam->getCell(number);
		pose robotpos			= mySlam->getPos(number);
		visualMap* vmap			= mySlam->getVMap();

		// celda con orientacion
//		ocell ori;
//		ori.x = poscell.x;
//		ori.y = poscell.y;
//		float ang = robotpos.th;
//		ori.th = int(floor(ang/(PI/4)+0.5));
//		if (ori.th > 7) ori.th -=8;
//		if (ori.th < 0) ori.th +=8;

		IplImage* im = (showPlanner && rbase->getNumber()==0)? omap->getMapAsImage():0;

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

		// si hemos llegado al ultimo destino planificamos
		if(completedPath == true  || nextStep == nextPlanning){
			nextPlanning = nextStep + (int)floor(replanning_period/mySlam->getSampleTime()+0.5);

			completedPath = false;
//			printf("[INTEGRATEDIGPLANNER] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PLANNING >>>> robot %d\n",number);
			// mapa de fronteras
			binMap objectives;
			vector<clusterCell> clusterList;
			omap->frontiers(objectives);
			objectives.cluster(clusterList,2*inflate_obstacles);
			objectives.set(clusterList);			//objectives.showMap("FRONTERAS");
			
			// mapa de costes
			vector<point> targets;
			vector<point>::iterator targetIter;
			costMapSimple* cm = createCostMap(poscell,objectives,*omap,targets,inflate_obstacles);
			
//			printf("[INTEGRATEDIGPLANNER] Numero de objetivos: %d\n",(int)targets.size());
			
			// elegir destino
			point dest = poscell;
			float maxVal =  -9999999999999.9f;
			int util_cells = ((int)floor(utility_radius/omap->getResolution()+0.5));
			float maxutility =  util_cells*util_cells*PI;
			float maxcost = cm->getMaxCost();
			float det = mySlam->getCovariance(number).det();
			float maxlocutil = -10000.0f;
			if (det > 0) maxlocutil= -0.5*log( pow(PIx2*exp(1.0f),3) *  det);

//			printf("[INTEGRATEDIGPLANNER] decidiendo el destino....\n");
			for (targetIter = targets.begin(); targetIter!= targets.end(); targetIter++){
//				if (fabs(targetIter->x-ori.x) + fabs(targetIter->y - ori.y) > 15){

					binMap esz;
					omap->esz(targetIter->x,targetIter->y,esz,0,util_cells);
					float utilvalue = IGutility(*omap,esz);
					float cost = cm->getCost(targetIter->x,targetIter->y);
					esz.dilate(2);
					float locvalue = LOCutility(*targetIter,esz,*vmap,mySlam->getGlobalCovariance());
//					printf("loc= %f(%f), cost= %f(%f), util= %f(%f)\n",locvalue,maxlocutil,cost,maxcost,utilvalue,maxutility);
					float value = utility_weight*utilvalue/maxutility - cost_weight*cost/maxcost + localization_weight*locvalue/maxlocutil;
					//printf("(%d,%d) IGutility = %f, localizability = %f, cost= %f \n",targetIter->x,targetIter->y,utilvalue,locvalue, cost);
					if (value > maxVal){
						maxVal = value;
						dest = *targetIter;
					}
//				}
			}
//			printf("[INTEGRATEDIGPLANNER] decidido\n");

			cm->getPath(dest,path);	

//			printf("[INTEGRATEDIGPLANNER] length of the path = %d\n",path.size());
			delete cm;
			
			if (targets.size()==0){
//				printf("[INTEGRATEDIGPLANNER] No path to frontiers, stopping exploration\n");
				endPlanner = true;
				completedPath = true;
				char sms[100];
				sprintf(sms,"END %d", getDir()); 
				sendMessage(string(sms),-1);
			}
//			printf("[INTEGRATEDIGPLANNER] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PLANNING END >>>> robot %d\n",number);
		}

		// Seguimos con la ruta
		else {
	
			// verificamos que el path sea transitable
			bool obstructedPath = false;
			vector<point>::reverse_iterator riter;
			if(path.size()>1) { 
				for ( riter = path.rbegin(); riter != path.rend(); riter++ )
					if (omap->isoccupied(riter->x,riter->y)) { 
//						printf("[INTEGRATEDIGPLANNER] [robot %d] obstructed\n",rbase->getNumber()); 
						obstructedPath = true;
						break;
					}
			}
			// verificar distancia al destino || obstruccion para path completado
			if((sqrt(pow((float)(path.front().x-poscell.x),2) + pow((float)(path.front().y-poscell.y),2)) <= 3.0)  || obstructedPath){
//				printf("[INTEGRATEDIGPLANNER] [robot %d] completed\n",rbase->getNumber()); 
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
//					printf("[INTEGRATEDIGPLANNER] [robot %d] no visible cells\n",rbase->getNumber()); 
				}
				//if (showPlanner) cvReleaseImage(&im);

			}
//			printf("[INTEGRATEDIGPLANNER] [robot %d] next goal: (%d,%d) - current pos (%d,%d)\n",rbase->getNumber(),goal.x, goal.y, ori.x,ori.y);						
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


		delete omap;
		delete ppmap;
		delete vmap;

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



float IntegratedIGPlanner::IGutility(const gridMapInterface& omap, const binMap& esz){
	float entropy=0;
	for (int i = esz.getRoi().x ; i < esz.getRoi().x + esz.getRoi().width ; i++){
		for (int j = esz.getRoi().y ; j < esz.getRoi().y + esz.getRoi().height ; j++){   // for each cell
			if (esz.get(i,j)){
				float pocc = omap.getValue(i,j)/100.0f;
				float pemp = 1-pocc;
				if (pocc > 0 && pemp > 0)
					entropy += pocc*log(pocc)+pemp*log(pemp);
			}
		}
	}
	return -entropy;
}

float IntegratedIGPlanner::LOCutility(const point& oc, const binMap esz, visualMap& vmap, const Ematrix& P){
	float camrange2 = camrange*camrange;
	int nbots = mySlam->getNumRobots();

	pose pos;
	pos.x = mySlam->getXOrigin() + (oc.x+0.5)*mySlam->getResolution();
	pos.y = mySlam->getYOrigin() + (oc.y+0.5)*mySlam->getResolution();
	pos.th = 0;  //oc.th*PI/4.0f;

	Ematrix Hm(0,0,3*vmap.getNLandmarks(),3*vmap.getNLandmarks());
	Ematrix Hv(0,3,3*vmap.getNLandmarks(),3);

	int visiblemarks=0;
	int* marksnumbers = new int[vmap.getNLandmarks()];

	Ematrix Hmi		= jacobianHm(pos);

	for (int l = 0; l < vmap.getNLandmarks(); l++){
		landmark* mark = vmap.getLandmarkById(l);
		float dist2 = pow(mark->pos.getX()-pos.x,2)+pow(mark->pos.getY()-pos.y,2);
		if(dist2 < camrange2){
			Hm.extend(3,3);
			Hm.set(3*visiblemarks,3*visiblemarks,Hmi);
			Hv.extend(3,0);
			Hv.set(3*visiblemarks,0,jacobianHv(pos,mark->pos));
			marksnumbers[visiblemarks] = l;
			visiblemarks++;
		}
		delete mark;
	}	
	
	Ematrix Hmtrans = Hm.transpose();
	Ematrix Hvtrans = Hv.transpose();
	
	Ematrix Pom(3*visiblemarks,3*visiblemarks);
	for (int l1 = 0; l1 < visiblemarks; l1++){
		for (int l2 = 0; l2 < visiblemarks; l2++){
			Pom.set(l1*3,l2*3,P.subMat(3*nbots+3*marksnumbers[l1],3*nbots+3*marksnumbers[l1]+3,
									   3*nbots+3*marksnumbers[l2],3*nbots+3*marksnumbers[l2]+3));
		}
	}

	delete[] marksnumbers;
	//printf("visiblemarks=%d\n",visiblemarks);
	if (visiblemarks > 0){
		Ematrix Pvvinf = (Hvtrans*((Hm*(Pom*Hmtrans)).inverse()*Hv)).inverse();
		//P.print("P=");
		//Pom.print("Pom=");
		//Hm.print("Hm=");
		//Hv.print("Hv=");
		//Pvvinf.print("Pvvinf=");
		float det = Pvvinf.det();
		float locutil = -10000.0f;
		if (det > 0) locutil= -0.5*log( pow(PIx2*exp(1.0f),3) *  det);
		//printf("det= %e, locutil = %f\n",det,locutil);
		return locutil;
	}
	else return 0.0f;
}

// jacobian H
matrix IntegratedIGPlanner::jacobianHv(const pose& pos, const pos3d& mark){
	matrix H(3,3);
	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);
	float auxx = mark.getX() - pos.x;
	float auxy = mark.getY() - pos.y;
	H.set(0,0,-costh);
	H.set(0,1,-sinth);
	H.set(0,2,-auxx*sinth + auxy*costh );
	H.set(1,0, sinth);
	H.set(1,1,-costh);
	H.set(1,2,-auxx*costh - auxy*sinth );
	return H;
}

matrix IntegratedIGPlanner::jacobianHm(const pose& pos){
	matrix H(3,3);
	float costh = (float)cos(pos.th);
	float sinth = (float)sin(pos.th);
	H.set(0,0,costh);
	H.set(0,1,sinth);
	H.set(1,0,-sinth);
	H.set(1,1,costh);
	H.set(2,2,1.0f);
	return H;
}
