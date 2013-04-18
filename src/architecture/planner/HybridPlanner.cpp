
#include "HybridPlanner.h"
#include "gridMapInterface.h"
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <list>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "StringTokenizer.h"

#define	ST_EXPLORE	0
#define ST_ACT_LOC	1
#define ST_PLANNED	2
#define ST_LOOP_CLOSE	3

// Para registrar la clase en la factoria
namespace planners{
	planner* CreateHybridPlanner(const ConfigFile& conf){
		return new HybridPlanner(conf);
	};
	const int HYBRIDPLANNER	= 0 ;
	const bool registered = plannerFactory::Instance().Register(HYBRIDPLANNER, plannerCreator(CreateHybridPlanner));
}

using namespace std;

// constructor
HybridPlanner::HybridPlanner(const ConfigFile& config):
	planner(),
	badlocalized(false),
	state(ST_EXPLORE),
	endPlanner(true),
	actionradius		(config.read<float>("TREERADIUS")),
	eszdilationradius	(config.read<int>("TREEDILATIONRADIUS")),
	min_frontier_length	(config.read<int>("MIN_FRONTIER_LENGTH")),
	min_gateway_length	(config.read<int>("MIN_GATEWAY_LENGTH")),
	utility_radius		(config.read<float>("UTILITY_RADIUS"))
{}

// destructor
HybridPlanner::~HybridPlanner() {stop();}

// thread setup
int HybridPlanner::setup(){
//	printf("arranco planificador\n");
	prio = 50;
	state = ST_EXPLORE;
	badlocalized = false;
	return 0;
}

// thread on stop
void HybridPlanner::onStop(){
//	printf("[HYBRID] Stopping...\n");
	if (!endPlanner){
		endPlanner = true;
//		printf("endplanner=true, locking...\n");
		closing.lock();
//		printf("unlocking...\n");
		closing.unlock();
	}
	printf("[HYBRID_PLANNER] [robot %d] Stopped\n", rbase->getNumber());
}

// thread main code
void HybridPlanner::execute(){
	printf("[HYBRID_PLANNER] \t\t\t\t\t HYBRID_PLANNER (%d) running\n", rbase->getNumber());

	closing.lock();
	endPlanner = false;	
	int n,r;
	point goal;
	badlocalized = false;
	int nextStep=number;
	
//  init logs
	FILE* plannerlog=0;
//	avifile=0;
//	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>save avi file= %d\n",saveAviFile);
	if (logstr){
		char myfilestr[100];
		sprintf(myfilestr,"%splanR%d.log",logstr,number);
		plannerlog = fopen(myfilestr,"w");
//		if (saveAviFile){
//			sprintf(myfilestr,"outfiles/%sr%dvideoplan.avi",logstr,number);	
//			avifile =  cvCreateVideoWriter( myfilestr, -1, 1, cvSize(mySlam->getGMWidth(),mySlam->getGMHeight()),1);
////			printf("avifile created\n");
//		}
	}
	

//	char mystr[250];
//	if (showPlanner){
//		sprintf(mystr,"plan%d.jpg",number);
//		cvNamedWindow("PLAN",0);
//	}

	bool endExploration = false;
	//printf("empieza el bucle\n");

	int planfailed = 0;

	while(!endPlanner && !endExploration) {
		
		// esperamos al slam
		nextStep = mySlam->getStep()+15;
//		printf("[HYBRID PLANNER] [robot %d planner] waiting for step %d\n", number, nextStep);
		mySlam->waitForStep(nextStep);
//		printf("[HYBRID PLANNER] [robot %d planner] processing step %d\n", number, nextStep);

		if (nextStep > 20000) endExploration= true;

		// cogemos los mapas y poses
		gridMapInterface* omap	= mySlam->getOMap();
		binMap* ppmap			= mySlam->getPreMap();
		binMap* ipmap			= mySlam->getImpMap();
		point poscell			= mySlam->getCell(number);
		int numrobots			= mySlam->getNumRobots();
		point* robotcells		= new point[numrobots]();
		for(r=0; r<numrobots; r++)
			robotcells[r] = mySlam->getCell(r);

		// leer mensajes
		string msg;
		int msglen;
		while((msglen=getMessage(msg))){
			//cout << "Message: >> " << msg << endl;
			StringTokenizer st(&msg[0]);
			string messagetype(st.nextToken()); 
			if (messagetype == "END"){
				endPlanner = true;
			}
		}

		// antes de crear el arbol necesitamos saber si hay que contar fronteras o celdas precisas
		//printf("[HYBRID PLANNER] [robot %d] Dispersion: %f\n", number, mySlam->getDisp(number));
		badlocalized = mySlam->getLoc(number);

		// Creamos el arbol de exploracion
		//printf("[HYBRID PLANNER] [robot %d] creando arbol...\n", number);	
		treeNode* tree = createExplorationTree(numrobots, robotcells, poscell, *omap, *ppmap, *ipmap);	
		//printf("[HYBRID PLANNER] [robot %d] Ok - Nº de ramas principales: %d\n",number,tree->getNChildren());	
		
		// Evaluamos el arbol
		float val,max=-99999999999999.9f;
		evalTree(*tree);
		//printf("evaluado\n");
		max = tree->getValue();
		
		// Añadimos los nodos de primer nivel a la lista
		std::vector<treeNode*> nodeList;
		for (n = 0; n < tree->getNChildren(); n++){
			val = tree->getChildren(n).getValue();
			if (tree->getChildren(n).isLeaf()) {
				tree->getChildren(n).setValue(val*1.0f);		// factor de correccion para nodos hoja
			}
			nodeList.push_back(&tree->getChildren(n));
		}
		
		// Correccion de valores si hay mas robots en la zona
		float d2;
		for(r=0; r < numrobots; r++){
			if (r != number){
				if(tree->isRobot(r)){
					for (n = 0; n < tree->getNChildren(); n++){
						d2 = pow((float)(robotcells[r].x-tree->getChildren(n).getX()),2)+pow((float)(robotcells[r].y-tree->getChildren(n).getY()),2);
						tree->getChildren(n).setValue(tree->getChildren(n).getValue()*d2);
					}
				}
			}
		}
		
		// decidimos reactivo o planificado 
		if (tree->getNChildren() > 0){
			//printf("ordenar\n");		
			std::sort(nodeList.begin(), nodeList.end(), treeNode::orderByValue);
			//printf("ok\n");
			if (nodeList.front()->getNodeType()==GATEWAY_NODE){		// planificado si el nodo de mas valor no es un nodo hoja
				state = ST_PLANNED;
				goal.x = nodeList.front()->getX();
				goal.y = nodeList.front()->getY();
			}
			else if (nodeList.front()->getNodeType()==LEAF_TYPE_FRONTIER){
				state = ST_EXPLORE;
			}
			else if (nodeList.front()->getNodeType()==LEAF_TYPE_PRECISE_POSE){
				state = ST_ACT_LOC;
			}
			if (max==0){
			 	if (planfailed >=2){
					endExploration = true;	// si el valor es cero acabamos
				}
				else{
					planfailed++;
				}
			}
			else{
				planfailed=0;
			}
		}

		// Activamos el estado correspondiente en la capa reactiva
		if (state == ST_EXPLORE){
			//printf("************************************** [robot %d] EXPLORE ESZ\n",number);
	
			reac->enableAvoidObstacles();
			reac->enableAvoidOtherRobots();
			reac->enableGoToFrontier();
			reac->enableGoToUnexploredZones();

			reac->disableGoToGoal();
			reac->disableGoToPrecisePoses();
		}
		
		if (state == ST_ACT_LOC){
			//printf("************************************** [robot %d] ACTIVE LOCALIZATION \n",number);
			reac->enableAvoidObstacles();
			reac->enableGoToPrecisePoses();

			reac->disableAvoidOtherRobots();
			reac->disableGoToFrontier();
			reac->disableGoToUnexploredZones();
			reac->disableGoToGoal();
		}

		if (state == ST_PLANNED){
			//printf("************************************** [robot %d] PLANNED \n",number);
			reac->setGoal(goal);
			reac->enableAvoidObstacles();
			reac->enableGoToGoal();

			reac->disableAvoidOtherRobots();
			reac->disableGoToFrontier();
			reac->disableGoToUnexploredZones();
			reac->disableGoToPrecisePoses();			
		}

		delete tree;
		delete omap;
		delete ppmap;
		delete[] robotcells;
		
		// write logs
		if (logstr){
			fprintf(plannerlog,"%d, %d, %d, %d, %d\n",nextStep, state, badlocalized, poscell.x, poscell.y);
		}

	}

	char sms[100];
	sprintf(sms,"END %d", getDir()); 
	sendMessage(string(sms),-1);

//	if (showPlanner) cvDestroyWindow("PLAN");

	
	//closing logs
	if (logstr)	fclose(plannerlog);

//	printf("[robot %d] stoping robot\n", number);
	reac->disableAll();
	reac->stop();

	closing.unlock();
	explorationFinished.step();
	
}

treeNode* HybridPlanner::createExplorationTree(int numrobots, const point* robotcells, const point& pos, const gridMapInterface& omap, const binMap& precisemap, const binMap& imprecisemap){
//	printf("here 1\n");

	// nodo raiz
	treeNode* root_node = new treeNode(GATEWAY_NODE, numrobots);
	root_node->setCell(pos);
	root_node->setCost(0);
//	printf("here 2\n");

	// zona procesada en blanco
	binMap processed(mySlam->getGMWidth(), mySlam->getGMHeight(), mySlam->getResolution(), mySlam->getXOrigin(), mySlam->getYOrigin());
//	printf("here 3\n");

	// lista de nodos por procesar
	std::vector<treeNode*> nodeStorage;

	// puntero al siguiente nodo, empezamos por el raiz
	treeNode* node = root_node;
//	printf("here 4\n");

//	if (showPlanner) im = omap.getMapAsImage();

	int numnode=0;

	int actionradius_cells = ((int)floor(actionradius/omap.getResolution()+0.5));
	int utilityradius_cells = ((int)floor(utility_radius/omap.getResolution()+0.5));
//	printf("here 5\n");
	do{

		numnode++;		
		// buscamos la ESZ del nodo
		binMap newsafezone;
//		printf("voy a hacer el esz\n");
		if(node->isLeaf()){	
			omap.esz(node->getX(), node->getY(), newsafezone, 0,utilityradius_cells);
		}
//		printf ("[%d] esz ok\n",numnode);
//		if (showPlanner){
//			if (node == root_node){
//				for (int ii=newsafezone.getRoi().x-10; ii<newsafezone.getRoi().x + newsafezone.getRoi().width+10; ii++){
//					for (int jj=newsafezone.getRoi().y-10; jj<newsafezone.getRoi().y + newsafezone.getRoi().height+10; jj++){
//						if (newsafezone.get(ii,jj))
//							cvCircle(im,cvPoint(ii,newsafezone.getHeight()-1-jj),0,CV_RGB(255,255,0),-1);
//					}
//				}
//				//show ROI
//				//cvRectangle(im, 
//				//	cvPoint(newsafezone.getRoi().x, newsafezone.getHeight()-1- newsafezone.getRoi().y),
//				//	cvPoint(newsafezone.getRoi().x + newsafezone.getRoi().width, newsafezone.getHeight()-1- (newsafezone.getRoi().y + newsafezone.getRoi().height)),
//				//	CV_RGB(0,0,255),1);
//			}
//		}
//			printf("[%d] Door - Cost: %d\n",numnode,node->getCost());

		else{											// nodo puerta
			omap.esz(node->getX(), node->getY(), newsafezone, eszdilationradius, actionradius_cells);
			
			// quitamos lo ya procesado
			// newsafezone.sub(processed);
			
			binMap filter(newsafezone);
			filter.sub(processed);
			filter.removeUnconnected(node->getX(), node->getY());
			filter.set(node->getX(), node->getY(), true);
			// buscamos nodos hijo
			seekChildren(numrobots, robotcells, newsafezone, filter, *node, omap, precisemap, imprecisemap);
			for(int i = 0; i< node->getNChildren(); i++)
				nodeStorage.push_back(&node->getChildren(i));	// los añadimos a la lista
			
			// añadimos la nueva zona a la ya procesado
			processed.add(filter);
			
//			if (showPlanner){
//				cvCircle(im,cvPoint(node->getX(), omap.getHeight()-1-node->getY()),1,CV_RGB(0,255,255),-1);
//				if (node != root_node)
//					cvLine(im,cvPoint(node->getX(),omap.getHeight()-1-node->getY()),cvPoint(node->getParent().getX(),omap.getHeight()-1-node->getParent().getY()),CV_RGB(0,0,255));
//			}	
		}
//		if(node->isLeaf()){													// nodo hoja
//			if (showPlanner){
//				if (node != root_node)
//					cvLine(im,cvPoint(node->getX(),omap.getHeight()-1-node->getY()),cvPoint(node->getParent().getX(),omap.getHeight()-1-node->getParent().getY()),CV_RGB(0,0,255));
//				cvCircle(im,cvPoint(node->getX(),omap.getHeight()-1-node->getY()),1,CV_RGB(0,255,0),-1);
//			}
////			printf("[%d] Leaf - Cost: %d\n",numnode,node->getCost());
//		}
//		printf("[%d] eval\n",numnode);
		// evaluamos la zona
		evalZone(numrobots, robotcells, newsafezone, *node, omap, precisemap, imprecisemap);
//		printf("[%d] eval finished\n",numnode);
	
		//printf("Storage size  = %d\n", nodeStorage.size());

		// ordenamos de mas a menos coste y tomamos de la lista el de menor coste que quede
		if (nodeStorage.size()>0){
			//printf("sorting...\n");
			std::sort(nodeStorage.begin(), nodeStorage.end(), treeNode::orderByCost);
			node = nodeStorage.back();
			nodeStorage.pop_back();
		}
		else
			node = 0;

//		printf("[%d] next node\n",numnode);

	}while(node);

//	if (showPlanner){
//		cvShowImage("PLAN",im);
//		cvWaitKey(5);

////		printf("voy a hacer el frame grabe %d, %d\n",saveAviFile,avifile);
//		if(saveAviFile && avifile){
//			int res = cvWriteFrame(avifile, im);
//			printf("frame grabber, result %d\n",res); 
//		}
////		printf("acabo\n");

//		cvReleaseImage(&im);
//	}
	return root_node;
}


void HybridPlanner::seekChildren(int numrobots, const point* robotcells, const binMap& safezone, const binMap& filter, treeNode& node, const gridMapInterface& omap, const binMap& precisemap, const binMap& imprecisemap){
	//printf("[robot %d] seeking\n", number);

	// buscamos puertas
	binMap gateways;
	omap.gateways(safezone, gateways);
	gateways.times(filter);

//	if (showPlanner){
//		for (int ii=gateways.getRoi().x-10; ii<gateways.getRoi().x + gateways.getRoi().width+10; ii++){
//			for (int jj=gateways.getRoi().y-10; jj<gateways.getRoi().y + gateways.getRoi().height+10; jj++){
//				if (gateways.get(ii,jj))
//					cvCircle(im,cvPoint(ii,gateways.getHeight()-1-jj),0,CV_RGB(255,0,0),-1);
//			}
//		}
//	}
	cluster(numrobots, robotcells, gateways, node, GATEWAY_NODE); // añadir los nodos puerta

	// buscamos nodos hoja
	if (!badlocalized){		// buscamos fronteras
		binMap frontiers;
		int nfront = omap.frontiersIn(filter, frontiers);
		if (nfront>0) cluster(numrobots, robotcells, frontiers, node, LEAF_TYPE_FRONTIER); // añadir los nodos frontera
	//	binMap impreciseCells(filter);
	//	impreciseCells.times(imprecisemap);
	//	cluster(numrobots, robotcells, impreciseCells, node, LEAF_TYPE_IMPRECISE_POSE); // añadir los nodos de celda precisa
	}
	else{				// buscamos celdas precisas
		binMap preciseCells(filter);
		preciseCells.times(precisemap);
		cluster(numrobots, robotcells, preciseCells, node, LEAF_TYPE_PRECISE_POSE); // añadir los nodos de celda precisa
	}
}

int HybridPlanner::cluster(int numrobots, const point* robotcells, const binMap& map, treeNode& parent, int nodetype){
	int i,j;
	//printf("[robot %d] cluster\n", number);
	//mapa en blanco de celdas analizadas
	binMap aux(map.getWidth(), map.getHeight(), map.getResolution(), map.getXOrigin(), map.getYOrigin());

	door cl;
	int count = 0;	// numero de clusters encontrado
	float d;

	for (i = map.getRoi().x ; i < map.getRoi().x + map.getRoi().width ; i++){
		for (j = map.getRoi().y ; j < map.getRoi().y + map.getRoi().height ; j++){   // for each cell
			
			std::list<point> seq;
			clustering(i,j,aux,map,cl,seq,true);
			
			if (cl.scale >= min_gateway_length || (cl.scale >= min_frontier_length && (nodetype > LEAF_TYPE_FRONTIER))){ // cluster de al menos 3 celdas

				std::list<point>::iterator it;
				int k;
				for( k = 0, it = seq.begin(); it != seq.end() && k <= cl.scale/2; it++ , k++);
				
				it--;
				k--;

				cl.x = it->x;
				cl.y = it->y;

			//	cl.x = cl.x/cl.scale;
			//	cl.y = cl.y/cl.scale;

				count++;
				
			//	printf("[robot %d] new node (%d, %d - length: %d, k: %d)\n",number, (int)cl.x, (int)cl.y, (int)cl.scale, k);
				treeNode* newNode = new treeNode(nodetype, numrobots); // new node

				newNode->setCell(point((int)cl.x,(int)cl.y));	// position
				//newNode->setCell(point(i,j));	// position

				d = EPS+sqrt(pow(((float)parent.getX())-cl.x,2)+pow(((float)parent.getY())-cl.y,2));
				//d = EPS+sqrt(pow(((float)parent.getX())-i,2)+pow(((float)parent.getY())-j,2));
				newNode->setCost(parent.getCost() + (int)d); 						// cost
				//printf("Cost: %f\n",parent->getCost() + (int)d);

				parent.addChildren(*newNode);
				
			}
			cl.x=0;
			cl.y=0;
			cl.scale=0;
		}
	}

	return count;
}

bool HybridPlanner::clustering(const int &i, const int &j, binMap& aux, const binMap& map, door& cl, std::list<point> &seq, bool backfront){

	if(aux.get(i,j)==false && map.get(i,j) == true ){  // if ipoint or door
		point p(i,j);

		if (backfront)
			seq.push_back(p);
		else
			seq.push_front(p);

		cl.x += i;
		cl.y += j;
		cl.scale++;
		aux.set(i,j,true);
		int x, y;
		for (x = i-1; x<=i+1; x++){
			for (y = j-1; y<=j+1; y++){
				if (clustering(x,y,aux,map,cl,seq,backfront)){
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

// busca robots, celdas precisas y celdas sin explorar en la zona
void HybridPlanner::evalZone(int numrobots, const point* robotcells, const binMap& map, treeNode& node, const gridMapInterface& omap, const binMap& precisemap, const binMap& imprecisemap){
	// buscar celdas de interes en la nueva zona de vision
	int nIPoints=0;
	int nbots=0;
	
	int i,j,r;

//	printf("[robot %d] %d,%d,%d,%d\n", number, map.roi.x, map.roi.y, map.roi.width, map.roi.height);

	node.clearRobots();

	for (i = map.getRoi().x ; i < map.getRoi().x + map.getRoi().width ; i++){
		for (j = map.getRoi().y ; j < map.getRoi().y + map.getRoi().height ; j++){   // for each cell
			if (map.get(i,j)){
				if (node.isLeaf()){
					if (badlocalized){
						if(precisemap.get(i,j)){
							nIPoints++;
						}
					}
					else{
						if(omap.isunknown(i,j)){

							nIPoints++;
						}
					}
				}
				for (r = 0; r<numrobots; r++){
					if (r!=number && robotcells[r].x == i && robotcells[r].y ==j ){
						nbots++;
						node.setRobot(r);
					}
				}

				if (node.getNodeType() == LEAF_TYPE_PRECISE_POSE){
					if(precisemap.get(i,j)){
						nIPoints++;
					}
				}
				else if (node.getNodeType() == LEAF_TYPE_FRONTIER){
					if(omap.isunknown(i,j)){
						nIPoints++;
					}
				}
				else if (node.getNodeType() == GATEWAY_NODE) {
					for (r = 0; r<numrobots; r++){
						if (r!=number && robotcells[r].x == i && robotcells[r].y ==j ){
							nbots++;
							node.setRobot(r);
						}
					}
				}
			}
		}
	}
	node.setiPoints(nIPoints);
	node.setnBots(nbots);
}

void HybridPlanner::evalTree(treeNode& tree){
	if (tree.isLeaf()){
		tree.setValue(((float)tree.getiPoints()) / pow((float)tree.getCost(),2));
	}
	else{
		float val,max=0;
		for (int n = 0; n < tree.getNChildren(); n++){
			evalTree(tree.getChildren(n));
			val = tree.getChildren(n).getValue();
			if (val>=max) max = val;
		}
		tree.setValue(max/(tree.getnBots()+1.0f));
	}
}



