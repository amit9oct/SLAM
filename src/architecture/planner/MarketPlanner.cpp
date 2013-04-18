
#include "MarketPlanner.h"
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
	planner* CreateMarketPlanner(const ConfigFile& conf){
		return new MarketPlanner(conf);
	};
	const int MARKETPLANNER	= 5 ;
	const bool registered = plannerFactory::Instance().Register(MARKETPLANNER, plannerCreator(CreateMarketPlanner));
}

using namespace std;

// Constructor
MarketPlanner::MarketPlanner(const ConfigFile& config):
	planner(),
	endPlanner(true),
	replanning_period(config.read<float>("REPLANNING_PERIOD")),
	inflate_obstacles(config.read<int>("INFLATE_OBSTACLES")),
	utility_radius(config.read<float>("UTILITY_RADIUS")),
	utility_weight(config.read<float>("UTILITY_WEIGHT")),
	cost_weight(config.read<float>("COST_WEIGHT"))
{}

// Destructor
MarketPlanner::~MarketPlanner(){stop();}

// thread Setup
int MarketPlanner::setup(){

//	printf("arranco planificador\n");
	completedPath = true;
	prio = 50;
	return 0;
}

// thread on stop
void MarketPlanner::onStop(){
	if (!endPlanner){
		endPlanner = true;
		closing.lock();
		closing.unlock();
	}
	printf("[MARKETPLANNER] [robot %d] Stopped\n", rbase->getNumber());
}

// thread main execution
void MarketPlanner::execute(){
	printf("[MARKETPLANNER] \t\t\t\t\t MARKETPLANNER (%d) running\n", rbase->getNumber());
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
//	if (showPlanner && rbase->getNumber()==0){
//		sprintf(mywindowstr,"plan%d.jpg",number);
//		cvNamedWindow(mywindowstr,0);
//	}
	
	list<target>::iterator targetListIter;
	list<target>::reverse_iterator targetListIterRev;
	
	bool endExploration = false;
	int nextPlanning=0;
	while(!endPlanner && !endExploration){

		//printID("MARKET BASED PLANNER");
		//Sleep(50);

		// esperamos a SLAM
		nextStep=mySlam->getStep()+1;
//		printf("[MARKETPLANNER] [robot %d] waiting for step %d\n", number, nextStep);
		mySlam->waitForStep(nextStep);
		
		if (nextStep > 20000) endExploration = true;
		
		// tomamos los datos
		gridMapInterface* omap  	= mySlam->getOMap();
		binMap* ppmap			= mySlam->getPreMap();
		//int numrobots			= mySlam->getNumRobots();
		point poscell			= mySlam->getCell(number);
		pose robotpos			= mySlam->getPos(number);

		// celda del robot con orientacion
//		ocell ori;
//		ori.x = poscell.x;
//		ori.y = poscell.y;
//		float ang = robotpos.th;
//		ori.th = int(floor(ang/(PI/4)+0.5));
//		if (ori.th > 7) ori.th -=8;
//		if (ori.th < 0) ori.th +=8;
		
		//IplImage* im = (showPlanner && rbase->getNumber()==0)? omap->getMapAsImage():0;
		
		list<auct> auctionList;
		list<auct>::iterator aucListIt;
		bool newTargets = false;
		
		//******************************************************************************************** >>>>> leer mensajes
		string msg;
		int msglen;
		char sms[100];		
		
		bool listEmptied = false;

		auctionList.clear();			
		while((msglen=getMessage(msg))){
//			printf("[MARKETPLANNER] [robot %d] --> RECV %s",number,&msg[0]);
			StringTokenizer st(&msg[0]);
			string messagetype(st.nextToken());
			int robotid = atoi(st.nextToken());
			if (messagetype == "AUCTION"){										// -------->>> NEW AUCTION
				// Procesar mensaje
				auct aucdest;
				aucdest.cell.x  = atoi(st.nextToken());
				aucdest.cell.y  = atoi(st.nextToken());
				aucdest.auctioner = robotid;
				
				auctionList.push_back(aucdest);									// >>> add to the auction list
				//printf("[MARKETPLANNER] [robot %d] AUCTION LIST SIZE = %d\n",number,auctionList.size());
				newTargets = true;
			}
			else if (messagetype == "BID"){										// -------->>> BID RECEIVED
				// Procesar mensaje
				point aucdest;
				aucdest.x  = atoi(st.nextToken());
				aucdest.y  = atoi(st.nextToken());
				float aucprofit = atof(st.nextToken());
				
				// mirar si es mejor que las anteriores
				for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); targetListIter++){
					if (targetListIter->cell.x == aucdest.x && 
						targetListIter->cell.y == aucdest.y && 
						targetListIter->bestOfferProfit < aucprofit)
					{
						targetListIter->bestOfferID = robotid;
						targetListIter->bestOfferProfit = aucprofit;						// >>> Check if is better
					}
				}
			}
			else if (messagetype == "WINNER"){									// -------->>> AUCTION WON
				// Procesar mensaje
				target t;
				int winner = atoi(st.nextToken());
				t.cell.x  = atoi(st.nextToken());
				t.cell.y  = atoi(st.nextToken());
				t.profit = atof(st.nextToken());
				t.auctionEnds = 0;

				if (winner == getDir()){ // ganador, añadir target a la lista si no estaba
					bool found = false;
					for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); targetListIter++){	// si se a reasignado a otro lo borramos de la lista
						if (targetListIter->cell.x == t.cell.x && targetListIter->cell.y == t.cell.y){
							found = true;
							break;
						}
					}
					if (!found){
						targetList.push_back(t);										// >>> add to target list
						newTargets=true;
					}
				}
				else{
					if (targetList.size()){					
						for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); targetListIter++){	// si se a reasignado a otro lo borramos de la lista
							if (targetListIter->cell.x == t.cell.x && targetListIter->cell.y == t.cell.y){
								targetListIter = targetList.erase(targetListIter); 
								targetListIter--;
							}
						}
						if (targetList.size() ==0) listEmptied = true;			
					}
				}
			}	
			else if (messagetype == "END"){										// -------->>> EXPLORATION FINISHED
				endExploration = true;		
				completedPath = true;
			}
		}

		// **************************************************************************************************    si el tiempo del auction expira...
																// >>> Enviamos ganador
		if (targetList.size()){		
			for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); ){
				bool erased = false;
				if (targetListIter->auctionEnds==1){ // si hay mejores ofertas enviar WINNER
//					printf("[MARKETPLANNER] [robot %d] Auction cerrada\n",number);
			
					if (targetListIter->bestOfferID != getDir()){
						sprintf(sms,"WINNER %d %d %d %d %f\n", getDir(), targetListIter->bestOfferID, targetListIter->cell.x, targetListIter->cell.y, targetListIter->bestOfferProfit); 
//						printf("[MARKETPLANNER] [robot %d] --> SEND TO --> [robotID %d] %s",number,-1,&sms[0]);
						sendMessage(string(sms),-1);
						// borrar de la lista
						targetListIter = targetList.erase(targetListIter);
						erased = true;
					}
				}
				if (!erased && targetListIter->auctionEnds>0) targetListIter->auctionEnds--;
				if (!erased) ++targetListIter;
			}
			if (targetList.size() ==0) listEmptied = true;
		}
		// ************************************************************************************************************     si hemos llegado al ultimo destino planificamos

		if(completedPath || newTargets){  
		//if(completedPath == true  || newTargets || nextStep == nextPlanning){ /// OJO CON ESTO!!!
			nextPlanning = nextStep + (int)floor(replanning_period/mySlam->getSampleTime()+0.5);

//			printf("[MARKETPLANNER] [robot %d] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PLANNING >>>>\n",number);
			
			///////////////////////////////////// REUNIMOS OBJETIVOS Y CALCULAMOS COSTES ////////////////////////////////////
			
			binMap objectives(omap->getWidth(),omap->getHeight(),omap->getResolution(),omap->getXOrigin(),omap->getYOrigin()); // mapa de objetivos
			
			// cuando hemos completado un destino buscamos más fronteras para añadir
			vector<clusterCell> clusterList;
			vector<clusterCell>::iterator clusterListIt;
			
			if (targetList.size()==0 || completedPath){
				int nfront = omap->frontiers(objectives); // mapa de fronteras
				objectives.cluster(clusterList,2*inflate_obstacles);
				objectives.set(clusterList);										// fronteras en cluster a objectives list
//				printf("[MARKETPLANNER] [robot %d] Frontiers = %d\n",number,clusterList.size());
			}
			
			// los targets de la lista son objetivos tambien al replanificar
			for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); targetListIter++){
				objectives.set(targetListIter->cell.x,targetListIter->cell.y,true);					// añadimos los targets actuales a objectives list
			}
			
			// las auctions recibidas tambien son objetivos que hay que analizar
			for (aucListIt = auctionList.begin(); aucListIt!= auctionList.end(); aucListIt++){
				objectives.set(aucListIt->cell.x,aucListIt->cell.y,true);								// añadimos los auctions a objectives list
			}
			
			// Evaluamos el mapa de costes
			std::vector<point> targets;
			vector<point>::iterator targetIter;
			costMapSimple* cm = createCostMap(poscell,objectives,*omap,targets,inflate_obstacles);						// crear el costmap costmap
//			printf("[MARKETPLANNER] [robot %d] Numero de objetivos distintos: %d\n", number, (int)targets.size());
			
			///////////////////////////////////// EVALUAMOS EL BENEFICIO //////////////////////////////////////////
			
			std::list<target> reachableList;
			std::list<target>::iterator reachableListIt;
			
			// evaluamos el beneficio de cada target
			point dest = poscell;
			int util_cells = ((int)floor(utility_radius/omap->getResolution()+0.5));
			float maxutility =  util_cells*util_cells*PI;			
			float maxcost = (float) cm->getMaxCost();
			for (targetIter = targets.begin(); targetIter!= targets.end(); targetIter++){
				if (targetIter->x != poscell.x && targetIter->y != poscell.y){
					binMap esz;
					omap->esz(targetIter->x,targetIter->y,esz,0,util_cells);
					
					target t;
					t.utility = utility_weight*utility(*omap,esz)/maxutility;
					t.cost = cost_weight*cm->getCost(targetIter->x,targetIter->y)/maxcost;
					if (t.cost > 0.0f)
						t.profit = t.utility - t.cost;
					else
						t.profit = -1;
					//t.profit = 1 - t.cost;
					t.cell = *targetIter;
					reachableList.push_back(t);									// add evaluated objectives to valid target list
				}
			}
			
//			printf("[MARKETPLANNER] [robot %d] Targets validos distintos: %d\n",number, (int)reachableList.size());
//			printf("[MARKETPLANNER] [robot %d] Targets en auction list: %d\n",number, (int)auctionList.size());
//			printf("[MARKETPLANNER] [robot %d] Targets en target list: %d\n",number, (int)targetList.size());
//			printf("[MARKETPLANNER] [robot %d] Targets en new frontier cluster list: %d\n",number, (int)clusterList.size());
//			
			if (targetList.size()){			
				for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); targetListIter++){				// borramos los no alcanzables
					bool found = false;
					for (reachableListIt = reachableList.begin(); reachableListIt!= reachableList.end(); reachableListIt++)
						if (targetListIter->cell.x == reachableListIt->cell.x && targetListIter->cell.y == reachableListIt->cell.y) found = true;
					if (!found){
						targetList.erase(targetListIter);
						targetListIter--;
					}
				}
				if (targetList.size() ==0) listEmptied = true;
			}
			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//reachableList.sort();												// sort by profit
			bool listwasempty = (targetList.size()==0);
			for (reachableListIt = reachableList.begin(); reachableListIt!= reachableList.end(); reachableListIt++){	// For each target
				// pujamos por cada target de la lista de subastas
				for (aucListIt = auctionList.begin(); aucListIt!= auctionList.end(); aucListIt++){				// >> if it comes from the auction list
					if (aucListIt->cell.x == reachableListIt->cell.x && 
						aucListIt->cell.y == reachableListIt->cell.y)
					{
						sprintf(sms,"BID %d %d %d %f\n", getDir(), aucListIt->cell.x, aucListIt->cell.y, reachableListIt->profit); 
//						printf("[MARKETPLANNER] [robot %d] --> SEND %s",number,&sms[0]);
						sendMessage(string(sms),aucListIt->auctioner); 							// reply the BID
					}
				}
				// actualizamos los de la lista
				if (targetList.size()>0){
					for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); ){			// >> if it comes from the target list	
						bool erased = false;
						if (targetListIter->cell.x == reachableListIt->cell.x &&
							targetListIter->cell.y == reachableListIt->cell.y)
						{
							if (reachableListIt->utility>0.0011){									// update the target list
							//if (omap->isfrontier(reachableListIt->cell.x,reachableListIt->cell.y))
								targetListIter->profit = reachableListIt->profit;
								targetListIter->utility = reachableListIt->utility;
								targetListIter->cost = reachableListIt->cost;
							}
							else{													// remove from list if profit < 0
								//printf("[MARKETPLANNER] [robot %d] low utility, removing target\n", number);								
								targetListIter = targetList.erase(targetListIter); 
								erased = true;
							}
						}
						if (!erased) ++targetListIter;
					}
					if (targetList.size() ==0){
						listEmptied = true;	 // HE CAMBIADO ESTO			
						//printf("[MARKETPLANNER] [robot %d] list is empty\n", number);	
					}
				}
				// sacamos a subasta las nuevas fronteras									// >> if it comes from the frontier list
				for (clusterListIt = clusterList.begin(); clusterListIt!= clusterList.end(); clusterListIt++){
					if (clusterListIt->x == reachableListIt->cell.x && clusterListIt->y == reachableListIt->cell.y 
						&& reachableListIt->utility>0.0011)
					{
						if (!listwasempty){										// Broadcast Auction
							sprintf(sms,"AUCTION %d %d %d %f\n", getDir(), reachableListIt->cell.x, reachableListIt->cell.y,  reachableListIt->profit); 
//							printf("[MARKETPLANNER] [robot %d] --> SEND %s",number,&sms[0]);
							sendMessage(string(sms),-1);
							bool found = false;							
							for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); targetListIter++){	// si se a reasignado a otro lo borramos de la lista
								if (targetListIter->cell.x == reachableListIt->cell.x && targetListIter->cell.y == reachableListIt->cell.y){
									found = true;
									break;
								}
							}
							if (!found){
								reachableListIt->auctionEnds = 5;
								reachableListIt->bestOfferID=getDir();
								reachableListIt->bestOfferProfit=reachableListIt->profit;
								targetList.push_back(*reachableListIt);								// lo guardamos en el target list
							}
							else{
								targetListIter->auctionEnds = 5;
								targetListIter->bestOfferID=getDir();
								targetListIter->bestOfferProfit=reachableListIt->profit;								
							}
						}
						else{
							reachableListIt->auctionEnds = 0;							
							targetList.push_back(*reachableListIt);
						}
						break;
					}
				}
			}
			
//			printf("[MARKETPLANNER] [robot %d] targetList size after replanning = %d\n",number, targetList.size());
			
			// ordenamos los targets de la lista y elegimos el mejor
			if (targetList.size()){
				targetList.sort();
				//for (targetListIterRev = targetList.rbegin(); targetListIterRev != targetList.rend(); targetListIterRev++ ){
					//if (targetListIterRev->auctionEnds==0){
						//dest = targetListIterRev->cell;
						//break;
					//}
				//}
				dest = targetList.back().cell;
				cm->getPath(dest,path);			
			}
			else if(!listEmptied){	// HE CAMBIADO ESTO
//				printf("[MARKETPLANNER] [robot %d] No path to frontiers, stopping exploration\n",number);
				endExploration = true;
				completedPath = true;
				char sms[100];
				sprintf(sms,"END %d", getDir()); 
				sendMessage(string(sms),-1);
			}
			delete cm;
			completedPath = false;
			newTargets    = false;
//			printf("[MARKETPLANNER] [robot %d] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PLANNING END >>>>\n",number);
//			printf("[MARKETPLANNER] [robot %d] destination: %d, %d, path ini (%d,%d) \n",number, dest.x,dest.y, path.back().x, path.back().y);
		}

		// Seguimos con la ruta
		else {
			// quitamos el punto de la lista si hemos llegado
			for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); targetListIter++){
				if (abs(targetListIter->cell.x-poscell.x) <= 1 && abs(targetListIter->cell.y-poscell.y) <=1){
					targetListIter = targetList.erase(targetListIter); 	
					break;
				}
			}
	
			// verificamos que el path sea transitable
			bool obstructedPath = false;
			vector<point>::reverse_iterator riter;
			if(path.size()>1) { 
				for ( riter = path.rbegin(); riter != path.rend(); riter++ )
					if (omap->isoccupied(riter->x,riter->y)) { 
//						printf("[MARKETPLANNER] [robot %d] obstructed\n",rbase->getNumber()); 
						obstructedPath = true;
						break;
					}
			}
			// verificar distancia al destino || obstruccion para path completado
			if((sqrt(pow((float)(path.front().x-poscell.x),2) + pow((float)(path.front().y-poscell.y),2)) <= 3.0)  || obstructedPath){
//				printf("[MARKETPLANNER] [robot %d] completed\n",rbase->getNumber()); 
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
//					printf("[MARKETPLANNER] [robot %d] no visible cells\n",rbase->getNumber()); 
				}
				//if (showPlanner) cvReleaseImage(&im);

			}
//			printf("[MARKETPLANNER] [robot %d] next goal: (%d,%d) - current pos (%d,%d)\n",rbase->getNumber(),goal.x, goal.y, ori.x,ori.y);						
		}
		
		// visualizacion
//		if (showPlanner && rbase->getNumber()==0){
//			cvCircle(im,cvPoint(poscell.x, omap->getHeight()-1-poscell.y),0,CV_RGB(255,0,0),-1); //posicion rojo
//			cvCircle(im,cvPoint(poscell.x+avancex[ori.th], omap->getHeight()-1-poscell.y-avancey[ori.th]),0,CV_RGB(0,0,255),-1); // orientacion azul
//			for (unsigned int i = 0;i < path.size(); i++)
//				cvCircle(im,cvPoint(path[i].x, omap->getHeight()-1-path[i].y),0,CV_RGB(255,0,255),-1); // path magenta
//			if (targetList.size()>0)
//				for (targetListIter = targetList.begin(); targetListIter!= targetList.end(); targetListIter++ )
//					if (targetListIter->auctionEnds==0)
//						cvCircle(im,cvPoint(targetListIter->cell.x , omap->getHeight()-1-targetListIter->cell.y),1,CV_RGB(255,0,0),-1); // targets red
//					else
//						cvCircle(im,cvPoint(targetListIter->cell.x , omap->getHeight()-1-targetListIter->cell.y),1,CV_RGB(0,255,0),-1); // targets red
//			cvShowImage(mywindowstr,im);
//			cvWaitKey(5);
//			cvReleaseImage(&im);
//		}

		delete omap;
		delete ppmap;
		
		//	modificar los enables/goal de la parte reactiva
		if (completedPath)	reac->disableAll();				// paramos el robot hasta planificar de nuevo
		else{
//			printf("\t\t\t\t\t[robot %d] **************************************  Following Path\n",number);

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

//	if (showPlanner && rbase->getNumber()==0) cvDestroyWindow(mywindowstr);

	//closing logs
	if (logstr)	fclose(plannerlog);

//	printf("[robot %d] stoping robot", number);
	reac->disableAll();
	reac->stop();

	closing.unlock();
	explorationFinished.step();
}

int MarketPlanner::utility(const gridMapInterface& omap, const binMap& esz){
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
