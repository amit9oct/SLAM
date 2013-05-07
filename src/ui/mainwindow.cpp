
#include <stdio.h>
#include "mainwindow.h"
#include "team.h"
#include "simulatedModel.h"
#include "slamInterface.h"
#include "ui_RobotSettingsDialog.h"
#include "ui_scenePropertiesDialog.h"
#include "ui_slamOptionsDialog.h"
#include "ui_strategyOptionsDialog.h"
#include "ui_About.h"
#include "ui_appConfigDialog.h"
#include "StringTokenizer.h"
#include <QFileDialog>
#include <QDir>
#include <QTime>
#include <QGraphicsItemGroup>
#include <QMessageBox>

void waitEnding::run(){
	t->waitForTaskFinished();
	emit simulationFinished(true);
}

//
// Constructor
//
MainWindow::MainWindow(QWidget *parent) : 
	QMainWindow(parent), 
	numrobots(1),
	expConf(0),
	scenarioConf(0),
	scene(0), 
	myteam(0),
	slam(0),
	mapLoaded(false),
	strategiesGroup(0),
	view(&qscene),
	scale(20),
	scaleInitialized(false),
	waitTh(0)
{
	
	setupUi(this);
	show();
	
	// cargar config files
	appConf = new ConfigFile(  ( QDir::homePath()+="/.mrxt/config/app.config").toStdString().c_str() );
	slamConf = new ConfigFile( ( QDir::homePath()+="/.mrxt/config/slam.config").toStdString().c_str() );

	qRegisterMetaType<rlines>("rlines");
	qRegisterMetaType<rposes>("rposes");
	
	connect( actionOpen_Scenario, SIGNAL(triggered(bool)), this, SLOT(openScenario(bool)));
	connect( actionSaveOmap, SIGNAL(triggered(bool)), this, SLOT(saveOmap(bool)));
	connect( actionSaveVmap, SIGNAL(triggered(bool)), this, SLOT(saveVmap(bool)));
	connect( actionSaveSLAMlog, SIGNAL(triggered(bool)), this, SLOT(saveSLAMlog(bool)));
	connect( actionSaveGTlog, SIGNAL(triggered(bool)), this, SLOT(saveGTlog(bool)));
	connect( actionQuit, SIGNAL(triggered(bool)), this, SLOT(close()));
	connect( spinBox, SIGNAL(valueChanged(int)), this, SLOT(changeNumRobots(int)));
	connect( randomPosesButton, SIGNAL(clicked(bool)), this, SLOT(randomPoses(bool)));
	connect( actionPlay, SIGNAL(triggered(bool)), this, SLOT(startSimulation(bool)));
	connect( actionStop, SIGNAL(triggered(bool)), this, SLOT(stopSimulation(bool)));
	connect( actionSceneProperties, SIGNAL(triggered(bool)), this, SLOT(openScenePropDiag(bool)));
	connect( actionRobotSettings, SIGNAL(triggered(bool)), this, SLOT(openRobotSetDiag(bool)));
	connect( actionStrategy_options, SIGNAL(triggered(bool)), this, SLOT(openStrategyOptDiag(bool)));
	connect( actionSLAM_options, SIGNAL(triggered(bool)), this, SLOT(openSLAMOptSetDiag(bool)));
	connect( actionAbout, SIGNAL(triggered(bool)), this, SLOT(openAboutDiag(bool)));
	connect( actionConfiguration, SIGNAL(triggered(bool)), this, SLOT(openAppConfigDiag(bool)));
	connect( zoomSlider, SIGNAL(valueChanged (int)), this, SLOT(sceneZoom(int)));
	
    	QActionGroup* strategiesGroup = new QActionGroup(this);
	strategiesGroup->addAction(actionNearest_Frontier);
	strategiesGroup->addAction(actionCost_Utility);
	strategiesGroup->addAction(actionMarket_Based);
	strategiesGroup->addAction(actionBehaviour_Based);
	strategiesGroup->addAction(actionHybrid);
	strategiesGroup->addAction(actionCoordinated);
	strategiesGroup->addAction(actionIntegrated);
	strategiesGroup->setExclusive(true);
       	actionNearest_Frontier->setChecked(true);
	changeStrategy();
	
    	QActionGroup* slamTechniquesGroup = new QActionGroup(this);
	slamTechniquesGroup->addAction(actionRBPF);
	slamTechniquesGroup->addAction(actionEKF);
	slamTechniquesGroup->addAction(actionEKF2);
	slamTechniquesGroup->setExclusive(true);
       	if (slamConf->read<int>("SLAM")==0) actionRBPF->setChecked(true);
       	else if (slamConf->read<int>("SLAM")==1) actionEKF->setChecked(true);
       	else if (slamConf->read<int>("SLAM")==2) actionEKF2->setChecked(true);
	actionSaveVmap->setEnabled(false);
	actionSaveOmap->setEnabled(false);
	actionSaveSLAMlog->setEnabled(false);
	actionSaveGTlog->setEnabled(false);
	actionStop->setEnabled(false);
	
	connect( actionNearest_Frontier, SIGNAL(triggered(bool)), this, SLOT(changeStrategy(bool)));
	connect( actionCost_Utility, SIGNAL(triggered(bool)), this, SLOT(changeStrategy(bool)));
	connect( actionMarket_Based, SIGNAL(triggered(bool)), this, SLOT(changeStrategy(bool)));
	connect( actionBehaviour_Based, SIGNAL(triggered(bool)), this, SLOT(changeStrategy(bool)));
	connect( actionHybrid, SIGNAL(triggered(bool)), this, SLOT(changeStrategy(bool)));
	connect( actionCoordinated, SIGNAL(triggered(bool)), this, SLOT(changeStrategy(bool)));
	connect( actionIntegrated, SIGNAL(triggered(bool)), this, SLOT(changeStrategy(bool)));
	connect( actionEKF, SIGNAL(triggered(bool)), this, SLOT(changeSlam(bool)));
	connect( actionEKF2, SIGNAL(triggered(bool)), this, SLOT(changeSlam(bool)));
	connect( actionRBPF, SIGNAL(triggered(bool)), this, SLOT(changeSlam(bool)));

	playButton->setDefaultAction(actionPlay);
	stopButton->setDefaultAction(actionStop);
	stopButton->setDisabled(true);

	actionSceneProperties->setDisabled(true);
	actionRobotSettings->setDisabled(true);

}

//
// Destroyer
//
MainWindow::~MainWindow(){
	if (scene) delete scene;
	if (expConf) delete expConf;
	if (myteam) delete myteam;
	if (scenarioConf) delete scenarioConf;
}


///////////////////////////////////////////////////////////////////            DIALOGS                ///////////////////////////////////////////////////

//
// Open Scenario Dialog
//
void MainWindow::openScenario(bool checked){

	QString fileName;
	fileName = QFileDialog::getOpenFileName(this, tr("Open Map File"), QDir::homePath()+="/.mrxt/maps", tr("map files (*.map)"));

	try{
		if (fileName.size() > 0){
			scenarioConf = new ConfigFile(fileName.toStdString().c_str());
			scene = new simulatedModel(numrobots,*scenarioConf, appConf->read<double>("SAMPLE_TIME"));

			connect( scene, SIGNAL(changedPositions(rposes)), this, SLOT(updatePoses(rposes)));

			// shows the scenario
			// testing the drwaing capabilities
			scrollArea->setWidget(&view);
			drawScene();
			drawRobots();

			actionSceneProperties->setEnabled(true);
			actionRobotSettings->setEnabled(true);
			mapLoaded = true;
		}
	}
	catch ( ... ){
		//printf("error loading map\n");
		mapLoaded = false;
	}
}


void MainWindow::saveOmap(bool checked){
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Occupancy Grid Map"), QDir::homePath()+="/.mrxt/outfiles", tr("Images (*.jpg)"));
	try{
		QFile::copy(QDir::homePath()+="/.mrxt/outfiles/tempLogomap.jpg", fileName);
	}
	catch( ... ){
		printf("Error saving file");
	}
}

void MainWindow::saveVmap(bool checked){
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Landmarks Map"),QDir::homePath()+="/.mrxt/outfiles", tr("matlab m-file (*.m)"));
	try{
		QFile::copy(QDir::homePath()+="/.mrxt/outfiles/tempLogvmap.m",fileName);
	}
	catch( ... ){
		printf("Error saving file");
	}
}

void MainWindow::saveSLAMlog(bool checked){
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save SLAM log"), QDir::homePath()+="/.mrxt/outfiles", tr("matlab m-file (*.m)"));
	try{
		QFile::copy(QDir::homePath()+="/.mrxt/outfiles/tempLogslam.m",fileName);
	}
	catch( ... ){
		printf("Error saving file");
	}
}
void MainWindow::saveGTlog(bool checked){
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Ground Truth log"), QDir::homePath()+="/.mrxt/outfiles", tr("matlab m-file (*.m)"));
	try{
		QFile::copy(QDir::homePath()+="/.mrxt/outfiles/tempLogGT.m",fileName);
	}
	catch( ... ){
		printf("Error saving file");
	}
}

//
// Strategy Options Dialog
//
void MainWindow::openStrategyOptDiag(bool){
	if (expConf) {
		Ui::StrategyOptionsDialog stratDiag;
		QDialog diag;
		stratDiag.setupUi(&diag);
		stratDiag.tabWidget->clear();

		if (actionHybrid->isChecked()){
			stratDiag.tabWidget->insertTab(0, stratDiag.HTab,"Hybrid");
			stratDiag.HFSSpinBox->setValue(expConf->read<double>("SIGMAGOFRO"));
			stratDiag.HUSSpinBox->setValue(expConf->read<double>("SIGMAGOUZ"));
			stratDiag.HOSSpinBox->setValue(expConf->read<double>("SIGMAAVOBS"));
			stratDiag.HRSSpinBox->setValue(expConf->read<double>("SIGMAAVROB"));
			stratDiag.HPSSpinBox->setValue(expConf->read<double>("SIGMAGOPRE"));
			stratDiag.HGSSpinBox->setValue(expConf->read<double>("SIGMAGOGOAL"));
			stratDiag.HFASpinBox->setValue(expConf->read<double>("WEIGHTGOFRO"));
			stratDiag.HUASpinBox->setValue(expConf->read<double>("WEIGHTGOUZ"));
			stratDiag.HOASpinBox->setValue(expConf->read<double>("WEIGHTAVOBS"));
			stratDiag.HRASpinBox->setValue(expConf->read<double>("WEIGHTAVROB"));
			stratDiag.HPASpinBox->setValue(expConf->read<double>("WEIGHTGOPRE"));
			stratDiag.HGASpinBox->setValue(expConf->read<double>("WEIGHTGOGOAL"));
			stratDiag.HVMAXSpinBox->setValue(expConf->read<double>("VMAX"));
			stratDiag.HWMAXSpinBox->setValue(expConf->read<double>("WMAX"));
			stratDiag.HK1SpinBox->setValue(expConf->read<double>("K1"));
			stratDiag.HK2SpinBox->setValue(expConf->read<double>("K2"));
			stratDiag.HLPWSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALWIDTH"));
			stratDiag.HLPHSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALHEIGHT"));
			if (expConf->read<bool>("USEESZ")) stratDiag.HUseESZCB->setChecked(true);
			else stratDiag.HUseESZCB->setChecked(false);
			stratDiag.HESZRadiusSpinBox->setValue(expConf->read<double>("ESZRADIUS"));
			stratDiag.HESZDilationSpinBox->setValue(expConf->read<int>("ESZDILATIONRADIUS"));

			stratDiag.HTreeRadiusSpinBox->setValue(expConf->read<double>("TREERADIUS"));
			stratDiag.HTreeDilationSpinBox->setValue(expConf->read<int>("TREEDILATIONRADIUS"));
			stratDiag.HMFLSpinBox->setValue(expConf->read<int>("MIN_FRONTIER_LENGTH"));
			stratDiag.HMGLSpinBox->setValue(expConf->read<int>("MIN_GATEWAY_LENGTH"));
			stratDiag.HURSpinBox->setValue(expConf->read<double>("UTILITY_RADIUS"));

		}
		else  if (actionBehaviour_Based->isChecked()){
			stratDiag.tabWidget->insertTab(0, stratDiag.BBTab, "BehaviourBased");
			stratDiag.BBFSSpinBox->setValue(expConf->read<double>("SIGMAGOFRO"));
			stratDiag.BBUSSpinBox->setValue(expConf->read<double>("SIGMAGOUZ"));
			stratDiag.BBOSSpinBox->setValue(expConf->read<double>("SIGMAAVOBS"));
			stratDiag.BBRSSpinBox->setValue(expConf->read<double>("SIGMAAVROB"));
			stratDiag.BBPSSpinBox->setValue(expConf->read<double>("SIGMAGOPRE"));
			stratDiag.BBGSSpinBox->setValue(expConf->read<double>("SIGMAGOGOAL"));
			stratDiag.BBFASpinBox->setValue(expConf->read<double>("WEIGHTGOFRO"));
			stratDiag.BBUASpinBox->setValue(expConf->read<double>("WEIGHTGOUZ"));
			stratDiag.BBOASpinBox->setValue(expConf->read<double>("WEIGHTAVOBS"));
			stratDiag.BBRASpinBox->setValue(expConf->read<double>("WEIGHTAVROB"));
			stratDiag.BBPASpinBox->setValue(expConf->read<double>("WEIGHTGOPRE"));
			stratDiag.BBGASpinBox->setValue(expConf->read<double>("WEIGHTGOGOAL"));
			stratDiag.BBVMAXSpinBox->setValue(expConf->read<double>("VMAX"));
			stratDiag.BBWMAXSpinBox->setValue(expConf->read<double>("WMAX"));
			stratDiag.BBK1SpinBox->setValue(expConf->read<double>("K1"));
			stratDiag.BBK2SpinBox->setValue(expConf->read<double>("K2"));
			stratDiag.BBLPWSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALWIDTH"));
			stratDiag.BBLPHSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALHEIGHT"));
			if (expConf->read<bool>("USEESZ")) stratDiag.BBUseESZCB->setChecked(true);
			else stratDiag.BBUseESZCB->setChecked(false);
			stratDiag.BBESZRadiusSpinBox->setValue(expConf->read<double>("ESZRADIUS"));
			stratDiag.BBESZDilationSpinBox->setValue(expConf->read<int>("ESZDILATIONRADIUS"));
				
			if (expConf->read<bool>("INTEGRATE_SLAM")) stratDiag.BBCBIntegrated->setChecked(true);
			else stratDiag.BBCBIntegrated->setChecked(false);
			if (expConf->read<bool>("ESCAPE_FROM_LOCAL_MINIMA")) stratDiag.BBCBEscapeActivated->setChecked(true);
			else stratDiag.BBCBEscapeActivated->setChecked(false);
			stratDiag.BBIOSpinBox->setValue(expConf->read<int>("INFLATE_OBSTACLES"));
		}
		else  if (actionNearest_Frontier->isChecked()){
			stratDiag.tabWidget->insertTab(0, stratDiag.NFTab, "NearestFrontier");
			stratDiag.NFOSSpinBox->setValue(expConf->read<double>("SIGMAAVOBS"));
			stratDiag.NFGSSpinBox->setValue(expConf->read<double>("SIGMAGOGOAL"));
			stratDiag.NFOASpinBox->setValue(expConf->read<double>("WEIGHTAVOBS"));
			stratDiag.NFGASpinBox->setValue(expConf->read<double>("WEIGHTGOGOAL"));
			stratDiag.NFVMAXSpinBox->setValue(expConf->read<double>("VMAX"));
			stratDiag.NFWMAXSpinBox->setValue(expConf->read<double>("WMAX"));
			stratDiag.NFK1SpinBox->setValue(expConf->read<double>("K1"));
			stratDiag.NFK2SpinBox->setValue(expConf->read<double>("K2"));
			stratDiag.NFLPWSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALWIDTH"));
			stratDiag.NFLPHSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALHEIGHT"));

			stratDiag.NFRPSpinBox->setValue(expConf->read<double>("REPLANNING_PERIOD"));
			stratDiag.NFIOSpinBox->setValue(expConf->read<int>("INFLATE_OBSTACLES"));

			
		}
		else  if (actionCost_Utility->isChecked()){
			stratDiag.tabWidget->insertTab(0, stratDiag.CUTab, "Cost-Utility");
			stratDiag.CUOSSpinBox->setValue(expConf->read<double>("SIGMAAVOBS"));
			stratDiag.CUGSSpinBox->setValue(expConf->read<double>("SIGMAGOGOAL"));
			stratDiag.CUOASpinBox->setValue(expConf->read<double>("WEIGHTAVOBS"));
			stratDiag.CUGASpinBox->setValue(expConf->read<double>("WEIGHTGOGOAL"));
			stratDiag.CUVMAXSpinBox->setValue(expConf->read<double>("VMAX"));
			stratDiag.CUWMAXSpinBox->setValue(expConf->read<double>("WMAX"));
			stratDiag.CUK1SpinBox->setValue(expConf->read<double>("K1"));
			stratDiag.CUK2SpinBox->setValue(expConf->read<double>("K2"));
			stratDiag.CULPWSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALWIDTH"));
			stratDiag.CULPHSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALHEIGHT"));

			stratDiag.CURPSpinBox->setValue(expConf->read<double>("REPLANNING_PERIOD"));
			stratDiag.CUIOSpinBox->setValue(expConf->read<int>("INFLATE_OBSTACLES"));
			stratDiag.CUURSpinBox->setValue(expConf->read<double>("UTILITY_RADIUS"));
			stratDiag.CUUWSpinBox->setValue(expConf->read<double>("UTILITY_WEIGHT"));
			stratDiag.CUCWSpinBox->setValue(expConf->read<double>("COST_WEIGHT"));

		}	
		else  if (actionCoordinated->isChecked()){
			stratDiag.tabWidget->insertTab(0, stratDiag.CTab, "Coordinated");
			stratDiag.COSSpinBox->setValue(expConf->read<double>("SIGMAAVOBS"));
			stratDiag.CGSSpinBox->setValue(expConf->read<double>("SIGMAGOGOAL"));
			stratDiag.COASpinBox->setValue(expConf->read<double>("WEIGHTAVOBS"));
			stratDiag.CGASpinBox->setValue(expConf->read<double>("WEIGHTGOGOAL"));
			stratDiag.CVMAXSpinBox->setValue(expConf->read<double>("VMAX"));
			stratDiag.CWMAXSpinBox->setValue(expConf->read<double>("WMAX"));
			stratDiag.CK1SpinBox->setValue(expConf->read<double>("K1"));
			stratDiag.CK2SpinBox->setValue(expConf->read<double>("K2"));
			stratDiag.CLPWSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALWIDTH"));
			stratDiag.CLPHSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALHEIGHT"));

			stratDiag.CRPSpinBox->setValue(expConf->read<double>("REPLANNING_PERIOD"));
			stratDiag.CIOSpinBox->setValue(expConf->read<int>("INFLATE_OBSTACLES"));
			stratDiag.CIRSpinBox->setValue(expConf->read<double>("INFLUENCE_RADIUS"));

		}	
		else  if (actionMarket_Based->isChecked()){
			stratDiag.tabWidget->insertTab(0, stratDiag.MBTab, "Market Based");
			stratDiag.MBOSSpinBox->setValue(expConf->read<double>("SIGMAAVOBS"));
			stratDiag.MBGSSpinBox->setValue(expConf->read<double>("SIGMAGOGOAL"));
			stratDiag.MBOASpinBox->setValue(expConf->read<double>("WEIGHTAVOBS"));
			stratDiag.MBGASpinBox->setValue(expConf->read<double>("WEIGHTGOGOAL"));
			stratDiag.MBVMAXSpinBox->setValue(expConf->read<double>("VMAX"));
			stratDiag.MBWMAXSpinBox->setValue(expConf->read<double>("WMAX"));
			stratDiag.MBK1SpinBox->setValue(expConf->read<double>("K1"));
			stratDiag.MBK2SpinBox->setValue(expConf->read<double>("K2"));
			stratDiag.MBLPWSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALWIDTH"));
			stratDiag.MBLPHSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALHEIGHT"));

			stratDiag.MBRPSpinBox->setValue(expConf->read<double>("REPLANNING_PERIOD"));
			stratDiag.MBIOSpinBox->setValue(expConf->read<int>("INFLATE_OBSTACLES"));
			stratDiag.MBURSpinBox->setValue(expConf->read<double>("UTILITY_RADIUS"));
			stratDiag.MBUWSpinBox->setValue(expConf->read<double>("UTILITY_WEIGHT"));
			stratDiag.MBCWSpinBox->setValue(expConf->read<double>("COST_WEIGHT"));


		}
		else  if (actionIntegrated->isChecked()){
			stratDiag.tabWidget->insertTab(0, stratDiag.ITab,"Integrated");
			stratDiag.IOSSpinBox->setValue(expConf->read<double>("SIGMAAVOBS"));
			stratDiag.IGSSpinBox->setValue(expConf->read<double>("SIGMAGOGOAL"));
			stratDiag.IOASpinBox->setValue(expConf->read<double>("WEIGHTAVOBS"));
			stratDiag.IGASpinBox->setValue(expConf->read<double>("WEIGHTGOGOAL"));
			stratDiag.IVMAXSpinBox->setValue(expConf->read<double>("VMAX"));
			stratDiag.IWMAXSpinBox->setValue(expConf->read<double>("WMAX"));
			stratDiag.IK1SpinBox->setValue(expConf->read<double>("K1"));
			stratDiag.IK2SpinBox->setValue(expConf->read<double>("K2"));
			stratDiag.ILPWSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALWIDTH"));
			stratDiag.ILPHSpinBox->setValue(expConf->read<double>("LOCALPOTENTIALHEIGHT"));

			stratDiag.IRPSpinBox->setValue(expConf->read<double>("REPLANNING_PERIOD"));
			stratDiag.IIOSpinBox->setValue(expConf->read<int>("INFLATE_OBSTACLES"));
			stratDiag.IURSpinBox->setValue(expConf->read<double>("UTILITY_RADIUS"));
			stratDiag.IUWSpinBox->setValue(expConf->read<double>("UTILITY_WEIGHT"));
			stratDiag.ICWSpinBox->setValue(expConf->read<double>("COST_WEIGHT"));
			stratDiag.ILWSpinBox->setValue(expConf->read<double>("LOCALIZATION_WEIGHT"));
			stratDiag.ICRSpinBox->setValue(expConf->read<double>("CAMERA_RANGE"));

		}

		diag.exec();
		
		if (diag.result()==QDialog::Accepted){

			if (actionHybrid->isChecked()){
				expConf->add<double>("SIGMAGOFRO", stratDiag.HFSSpinBox->value());
				expConf->add<double>("SIGMAGOUZ", stratDiag.HUSSpinBox->value());
				expConf->add<double>("SIGMAAVOBS", stratDiag.HOSSpinBox->value());
				expConf->add<double>("SIGMAAVROB", stratDiag.HRSSpinBox->value());
				expConf->add<double>("SIGMAGOPRE", stratDiag.HPSSpinBox->value());
				expConf->add<double>("SIGMAGOGOAL", stratDiag.HGSSpinBox->value());
				expConf->add<double>("WEIGHTGOFRO", stratDiag.HFASpinBox->value());
				expConf->add<double>("WEIGHTGOUZ", stratDiag.HUASpinBox->value());
				expConf->add<double>("WEIGHTAVOBS", stratDiag.HOASpinBox->value());
				expConf->add<double>("WEIGHTAVROB", stratDiag.HRASpinBox->value());
				expConf->add<double>("WEIGHTGOPRE", stratDiag.HPASpinBox->value());
				expConf->add<double>("WEIGHTGOGOAL", stratDiag.HGASpinBox->value());
				expConf->add<double>("VMAX", stratDiag.HVMAXSpinBox->value());
				expConf->add<double>("WMAX", stratDiag.HWMAXSpinBox->value());
				expConf->add<double>("K1", stratDiag.HK1SpinBox->value());
				expConf->add<double>("K2", stratDiag.HK2SpinBox->value());
				expConf->add<double>("LOCALPOTENTIALWIDTH", stratDiag.HLPWSpinBox->value());
				expConf->add<double>("LOCALPOTENTIALHEIGHT", stratDiag.HLPHSpinBox->value());
				if (stratDiag.HUseESZCB->isChecked()) expConf->add<bool>("USEESZ",true);
				else expConf->add<bool>("USEESZ",false);
				expConf->add<double>("ESZRADIUS", stratDiag.HESZRadiusSpinBox->value());
				expConf->add<int>("ESZDILATIONRADIUS", stratDiag.HESZDilationSpinBox->value());

				expConf->add<double>("TREERADIUS", stratDiag.HTreeRadiusSpinBox->value());
				expConf->add<int>("TREEDILATIONRADIUS", stratDiag.HTreeDilationSpinBox->value());
				expConf->add<int>("MIN_FRONTIER_LENGTH", stratDiag.HMFLSpinBox->value());
				expConf->add<int>("MIN_GATEWAY_LENGTH", stratDiag.HMGLSpinBox->value());
				expConf->add<double>("UTILITY_RADIUS", stratDiag.HURSpinBox->value());

			}
			else  if (actionBehaviour_Based->isChecked()){
				expConf->add<double>("SIGMAGOFRO", stratDiag.BBFSSpinBox->value());
				expConf->add<double>("SIGMAGOUZ", stratDiag.BBUSSpinBox->value());
				expConf->add<double>("SIGMAAVOBS", stratDiag.BBOSSpinBox->value());
				expConf->add<double>("SIGMAAVROB", stratDiag.BBRSSpinBox->value());
				expConf->add<double>("SIGMAGOPRE", stratDiag.BBPSSpinBox->value());
				expConf->add<double>("SIGMAGOGOAL", stratDiag.BBGSSpinBox->value());
				expConf->add<double>("WEIGHTGOFRO", stratDiag.BBFASpinBox->value());
				expConf->add<double>("WEIGHTGOUZ", stratDiag.BBUASpinBox->value());
				expConf->add<double>("WEIGHTAVOBS", stratDiag.BBOASpinBox->value());
				expConf->add<double>("WEIGHTAVROB", stratDiag.BBRASpinBox->value());
				expConf->add<double>("WEIGHTGOPRE", stratDiag.BBPASpinBox->value());
				expConf->add<double>("WEIGHTGOGOAL", stratDiag.BBGASpinBox->value());
				expConf->add<double>("VMAX", stratDiag.BBVMAXSpinBox->value());
				expConf->add<double>("WMAX", stratDiag.BBWMAXSpinBox->value());
				expConf->add<double>("K1", stratDiag.BBK1SpinBox->value());
				expConf->add<double>("K2", stratDiag.BBK2SpinBox->value());
				expConf->add<double>("LOCALPOTENTIALWIDTH", stratDiag.BBLPWSpinBox->value());
				expConf->add<double>("LOCALPOTENTIALHEIGHT", stratDiag.BBLPHSpinBox->value());
				if (stratDiag.BBUseESZCB->isChecked()) expConf->add<bool>("USEESZ",true);
				else expConf->add<bool>("USEESZ",false);
				expConf->add<double>("ESZRADIUS", stratDiag.BBESZRadiusSpinBox->value());
				expConf->add<int>("ESZDILATIONRADIUS", stratDiag.BBESZDilationSpinBox->value());

				if (stratDiag.BBCBEscapeActivated->isChecked()) expConf->add<bool>("ESCAPE_FROM_LOCAL_MINIMA",true);
				else expConf->add<bool>("ESCAPE_FROM_LOCAL_MINIMA",false);
				if (stratDiag.BBCBIntegrated->isChecked()) expConf->add<bool>("INTEGRATE_SLAM",true);
				else expConf->add<bool>("INTEGRATE_SLAM",false);
				expConf->add<int>("INFLATE_OBSTACLES", stratDiag.BBIOSpinBox->value());

			}	
			else  if (actionNearest_Frontier->isChecked()){
				expConf->add<double>("SIGMAAVOBS", stratDiag.NFOSSpinBox->value());
				expConf->add<double>("SIGMAGOGOAL", stratDiag.NFGSSpinBox->value());
				expConf->add<double>("WEIGHTAVOBS", stratDiag.NFOASpinBox->value());
				expConf->add<double>("WEIGHTGOGOAL", stratDiag.NFGASpinBox->value());
				expConf->add<double>("VMAX", stratDiag.NFVMAXSpinBox->value());
				expConf->add<double>("WMAX", stratDiag.NFWMAXSpinBox->value());
				expConf->add<double>("K1", stratDiag.NFK1SpinBox->value());
				expConf->add<double>("K2", stratDiag.NFK2SpinBox->value());
				expConf->add<double>("LOCALPOTENTIALWIDTH", stratDiag.NFLPWSpinBox->value());
				expConf->add<double>("LOCALPOTENTIALHEIGHT", stratDiag.NFLPHSpinBox->value());

				expConf->add<double>("REPLANNING_PERIOD", stratDiag.NFRPSpinBox->value());
				expConf->add<int>("INFLATE_OBSTACLES", stratDiag.NFIOSpinBox->value());

			}	
			else  if (actionCost_Utility->isChecked()){
				expConf->add<double>("SIGMAAVOBS", stratDiag.CUOSSpinBox->value());
				expConf->add<double>("SIGMAGOGOAL", stratDiag.CUGSSpinBox->value());
				expConf->add<double>("WEIGHTAVOBS", stratDiag.CUOASpinBox->value());
				expConf->add<double>("WEIGHTGOGOAL", stratDiag.CUGASpinBox->value());
				expConf->add<double>("VMAX", stratDiag.CUVMAXSpinBox->value());
				expConf->add<double>("WMAX", stratDiag.CUWMAXSpinBox->value());
				expConf->add<double>("K1", stratDiag.CUK1SpinBox->value());
				expConf->add<double>("K2", stratDiag.CUK2SpinBox->value());
				expConf->add<double>("LOCALPOTENTIALWIDTH", stratDiag.CULPWSpinBox->value());
				expConf->add<double>("LOCALPOTENTIALHEIGHT", stratDiag.CULPHSpinBox->value());

				expConf->add<double>("REPLANNING_PERIOD", stratDiag.CURPSpinBox->value());
				expConf->add<int>("INFLATE_OBSTACLES", stratDiag.CUIOSpinBox->value());
				expConf->add<double>("UTILITY_RADIUS", stratDiag.CUURSpinBox->value());
				expConf->add<double>("UTILITY_WEIGHT", stratDiag.CUUWSpinBox->value());
				expConf->add<double>("COST_WEIGHT", stratDiag.CUCWSpinBox->value());

			}	

			else  if (actionCoordinated->isChecked()){
				expConf->add<double>("SIGMAAVOBS", stratDiag.COSSpinBox->value());
				expConf->add<double>("SIGMAGOGOAL", stratDiag.CGSSpinBox->value());
				expConf->add<double>("WEIGHTAVOBS", stratDiag.COASpinBox->value());
				expConf->add<double>("WEIGHTGOGOAL", stratDiag.CGASpinBox->value());
				expConf->add<double>("VMAX", stratDiag.CVMAXSpinBox->value());
				expConf->add<double>("WMAX", stratDiag.CWMAXSpinBox->value());
				expConf->add<double>("K1", stratDiag.CK1SpinBox->value());
				expConf->add<double>("K2", stratDiag.CK2SpinBox->value());
				expConf->add<double>("LOCALPOTENTIALWIDTH", stratDiag.CLPWSpinBox->value());
				expConf->add<double>("LOCALPOTENTIALHEIGHT", stratDiag.CLPHSpinBox->value());

				expConf->add<double>("REPLANNING_PERIOD", stratDiag.CRPSpinBox->value());
				expConf->add<int>("INFLATE_OBSTACLES", stratDiag.CIOSpinBox->value());
				expConf->add<double>("INFLUENCE_RADIUS", stratDiag.CIRSpinBox->value());
			}	
			else  if (actionMarket_Based->isChecked()){
				expConf->add<double>("SIGMAAVOBS", stratDiag.MBOSSpinBox->value());
				expConf->add<double>("SIGMAGOGOAL", stratDiag.MBGSSpinBox->value());
				expConf->add<double>("WEIGHTAVOBS", stratDiag.MBOASpinBox->value());
				expConf->add<double>("WEIGHTGOGOAL", stratDiag.MBGASpinBox->value());
				expConf->add<double>("VMAX", stratDiag.MBVMAXSpinBox->value());
				expConf->add<double>("WMAX", stratDiag.MBWMAXSpinBox->value());
				expConf->add<double>("K1", stratDiag.MBK1SpinBox->value());
				expConf->add<double>("K2", stratDiag.MBK2SpinBox->value());
				expConf->add<double>("LOCALPOTENTIALWIDTH", stratDiag.MBLPWSpinBox->value());
				expConf->add<double>("LOCALPOTENTIALHEIGHT", stratDiag.MBLPHSpinBox->value());

				expConf->add<double>("REPLANNING_PERIOD", stratDiag.MBRPSpinBox->value());
				expConf->add<int>("INFLATE_OBSTACLES", stratDiag.MBIOSpinBox->value());
				expConf->add<double>("UTILITY_RADIUS", stratDiag.MBURSpinBox->value());
				expConf->add<double>("UTILITY_WEIGHT", stratDiag.MBUWSpinBox->value());
				expConf->add<double>("COST_WEIGHT", stratDiag.MBCWSpinBox->value());
			}

			else  if (actionIntegrated->isChecked()){
				expConf->add<double>("SIGMAAVOBS", stratDiag.IOSSpinBox->value());
				expConf->add<double>("SIGMAGOGOAL", stratDiag.IGSSpinBox->value());
				expConf->add<double>("WEIGHTAVOBS", stratDiag.IOASpinBox->value());
				expConf->add<double>("WEIGHTGOGOAL", stratDiag.IGASpinBox->value());
				expConf->add<double>("VMAX", stratDiag.IVMAXSpinBox->value());
				expConf->add<double>("WMAX", stratDiag.IWMAXSpinBox->value());
				expConf->add<double>("K1", stratDiag.IK1SpinBox->value());
				expConf->add<double>("K2", stratDiag.IK2SpinBox->value());
				expConf->add<double>("LOCALPOTENTIALWIDTH", stratDiag.ILPWSpinBox->value());
				expConf->add<double>("LOCALPOTENTIALHEIGHT", stratDiag.ILPHSpinBox->value());

				expConf->add<double>("REPLANNING_PERIOD", stratDiag.IRPSpinBox->value());
				expConf->add<int>("INFLATE_OBSTACLES", stratDiag.IIOSpinBox->value());
				expConf->add<double>("UTILITY_RADIUS", stratDiag.IURSpinBox->value());
				expConf->add<double>("UTILITY_WEIGHT", stratDiag.IUWSpinBox->value());
				expConf->add<double>("COST_WEIGHT", stratDiag.ICWSpinBox->value());
				expConf->add<double>("LOCALIZATION_WEIGHT", stratDiag.ILWSpinBox->value());
				expConf->add<double>("CAMERA_RANGE", stratDiag.ICRSpinBox->value());
			}

		}
	}
}

//
// About Dialog
//
void MainWindow::openAboutDiag(bool){
	Ui::AboutDialog aboutDiag;
	QDialog diag;
	aboutDiag.setupUi(&diag);
	diag.exec();
}

//
// Application Configuration Dialog
//
void MainWindow::openAppConfigDiag(bool){
	if (appConf){
		Ui::AppConfigDialog appDiag;
		QDialog diag;
		appDiag.setupUi(&diag);

		appDiag.sampleTimeSpinBox->setValue(appConf->read<double>("SAMPLE_TIME"));
		appDiag.groupPosesCheckBox->setChecked(appConf->read<bool>("GROUPINITIALPOSES"));	
		appDiag.maxIniPoseDistSpinBox->setValue(appConf->read<double>("MAXDISTINIPOSE"));
		appDiag.minIniPoseDistSpinBox->setValue(appConf->read<double>("MINDISTINIPOSE"));

		if(!appConf->read<bool>("GROUPINITIALPOSES")) appDiag.maxIniPoseDistSpinBox->setEnabled(false);

		diag.exec();

		if (diag.result()==QDialog::Accepted){
			appConf->add<double>("SAMPLE_TIME", appDiag.sampleTimeSpinBox->value());
			appConf->add<bool>("GROUPINITIALPOSES", appDiag.groupPosesCheckBox->isChecked());
			appConf->add<double>("MAXDISTINIPOSE", appDiag.maxIniPoseDistSpinBox->value());
			appConf->add<double>("MINDISTINIPOSE", appDiag.minIniPoseDistSpinBox->value());
		}
	}
}

//
// SLAM Options Dialog
//
void MainWindow::openSLAMOptSetDiag(bool){

	if (slamConf){
		Ui::SlamOptionsDialog slamDiag;
		QDialog diag;
		slamDiag.setupUi(&diag);
	
		slamDiag.resolutionSpinBox->setValue(slamConf->read<double>("RESOLUTION"));
		slamDiag.xOriginSpinBox->setValue(slamConf->read<double>("XORIGIN"));
		slamDiag.yOriginSpinBox->setValue(slamConf->read<double>("YORIGIN"));
		slamDiag.widthSpinBox->setValue(slamConf->read<double>("SCENE_WIDTH"));
		slamDiag.heightSpinBox->setValue(slamConf->read<double>("SCENE_HEIGHT"));
		slamDiag.RBPFRadioButton->setChecked((slamConf->read<int>("SLAM")==0));
		slamDiag.EKFRadioButton->setChecked((slamConf->read<int>("SLAM")==1));
		slamDiag.EKF2RadioButton->setChecked((slamConf->read<int>("SLAM")==2));
		slamDiag.particlesSpinBox->setValue(slamConf->read<int>("PARTICLESPERROBOT"));
		slamDiag.MultiplyPerRobotsCheckBox->setChecked(slamConf->read<bool>("MULTIPLYPARTPERROBOTS"));
		slamDiag.MahalanobisThSpinBox->setValue(slamConf->read<double>("MAHALANOBISTH"));
		slamDiag.DescriptorThSpinBox->setValue(slamConf->read<double>("DESCRIPTORTH"));
		slamDiag.lowThSpinBox->setValue(slamConf->read<double>("THRESHOLD_LOW"));
		slamDiag.highThSpinBox->setValue(slamConf->read<double>("THRESHOLD_HIGH"));
		slamDiag.MatchByDescriptorRadioButton->setChecked((slamConf->read<int>("NEARESTNEIGHBOURBY")==0));
		slamDiag.MatchByDistanceRadioButton->setChecked((slamConf->read<int>("NEARESTNEIGHBOURBY")==1));

		diag.exec();

		if (diag.result()==QDialog::Accepted){
			slamConf->add<double>("RESOLUTION", slamDiag.resolutionSpinBox->value());
			slamConf->add<double>("XORIGIN", slamDiag.xOriginSpinBox->value());
			slamConf->add<double>("YORIGIN", slamDiag.yOriginSpinBox->value());
			slamConf->add<double>("SCENE_WIDTH", slamDiag.widthSpinBox->value());
			slamConf->add<double>("SCENE_HEIGHT", slamDiag.heightSpinBox->value());
			if (slamDiag.RBPFRadioButton->isChecked()){
				slamConf->add<int>("SLAM", 0);
				actionRBPF->setChecked(true);
			}
			else if (slamDiag.EKFRadioButton->isChecked()){
				slamConf->add<int>("SLAM", 1);
				actionEKF->setChecked(true);
			}
			else if (slamDiag.EKF2RadioButton->isChecked()){
				slamConf->add<int>("SLAM", 2);
				actionEKF2->setChecked(true);
			}
			slamConf->add<int>("PARTICLESPERROBOT", slamDiag.particlesSpinBox->value());
			slamConf->add<bool>("MULTIPLYPARTPERROBOTS", slamDiag.MultiplyPerRobotsCheckBox->isChecked());
			slamConf->add<double>("MAHALANOBISTH", slamDiag.MahalanobisThSpinBox->value());
			slamConf->add<double>("DESCRIPTORTH", slamDiag.DescriptorThSpinBox->value());
			slamConf->add<double>("THRESHOLD_LOW", slamDiag.lowThSpinBox->value());
			slamConf->add<double>("THRESHOLD_HIGH", slamDiag.highThSpinBox->value());
			if (slamDiag.MatchByDescriptorRadioButton->isChecked())
				slamConf->add<int>("NEARESTNEIGHBOURBY", 0);
			if (slamDiag.MatchByDistanceRadioButton->isChecked())
				slamConf->add<int>("NEARESTNEIGHBOURBY", 1);
		}
	}
}

//
// Scene Properties Dialog
//
void MainWindow::openScenePropDiag(bool){

	if (mapLoaded){
		Ui::ScenePropertiesDialog sceneDiag;
		QDialog diag;
		sceneDiag.setupUi(&diag);
		sceneDiag.wallsLab->setNum(scenarioConf->read<int>("NUMWALLS"));
		sceneDiag.landmarksLab->setNum(scenarioConf->read<int>("NUMFEATURES"));
		sceneDiag.descLengthLab->setNum(scenarioConf->read<int>("DESCRIPTORLENGTH"));		

		diag.exec();

	}
}

//
// Robots Settings Dialog
//
void MainWindow::openRobotSetDiag(bool){

	if (mapLoaded){
		Ui::RobotSettingsDialog robotSetDiag;
		QDialog diag;
		robotSetDiag.setupUi(&diag);

		robotSetDiag.fSpinBox->setValue(scenarioConf->read<float>("f"));
		robotSetDiag.bSpinBox->setValue(scenarioConf->read<float>("I"));
		robotSetDiag.MaxDistSpinBox->setValue(scenarioConf->read<float>("distMAX"));
		robotSetDiag.MinDistSpinBox->setValue(scenarioConf->read<float>("distMIN"));
		robotSetDiag.widthSpinBox->setValue(scenarioConf->read<float>("WIDTH"));
		robotSetDiag.heightSpinBox->setValue(scenarioConf->read<float>("HEIGHT"));
		robotSetDiag.sigmacSpinBox->setValue(scenarioConf->read<float>("sigmac"));
		robotSetDiag.sigmarSpinBox->setValue(scenarioConf->read<float>("sigmar"));
		robotSetDiag.sigmadSpinBox->setValue(scenarioConf->read<float>("sigmad"));
		robotSetDiag.cameraxSpinBox->setValue(scenarioConf->read<float>("camx"));
		robotSetDiag.cameraySpinBox->setValue(scenarioConf->read<float>("camy"));
		robotSetDiag.camerazSpinBox->setValue(scenarioConf->read<float>("camz"));

		robotSetDiag.laserPointsSpinBox->setValue(scenarioConf->read<float>("NUMPOINTS"));
		robotSetDiag.laserMinAngleSpinBox->setValue(scenarioConf->read<float>("LASERMINANGLE"));
		robotSetDiag.laserMaxAngleSpinBox->setValue(scenarioConf->read<float>("LASERMAXANGLE"));
		robotSetDiag.laserMaxDistSpinBox->setValue(scenarioConf->read<float>("LASERMAXDIST"));
		robotSetDiag.laserMinDistSpinBox->setValue(scenarioConf->read<float>("LASERMINDIST"));
		robotSetDiag.laserSigmaSpinBox->setValue(scenarioConf->read<float>("LASERSIGMA"));
		robotSetDiag.laserxSpinBox->setValue(scenarioConf->read<float>("LASERX"));
		robotSetDiag.laserySpinBox->setValue(scenarioConf->read<float>("LASERY"));
		robotSetDiag.laserzSpinBox->setValue(scenarioConf->read<float>("LASERZ"));

		robotSetDiag.alfa1SpinBox->setValue(scenarioConf->read<float>("alfa1"));
		robotSetDiag.alfa2SpinBox->setValue(scenarioConf->read<float>("alfa2"));
		robotSetDiag.alfa3SpinBox->setValue(scenarioConf->read<float>("alfa3"));
		robotSetDiag.alfa4SpinBox->setValue(scenarioConf->read<float>("alfa4"));
		robotSetDiag.vmaxSpinBox->setValue(scenarioConf->read<float>("VABSMAX"));
		robotSetDiag.wmaxSpinBox->setValue(scenarioConf->read<float>("WABSMAX"));

		diag.exec();

		if (diag.result()==QDialog::Accepted){
			scenarioConf->add("f", robotSetDiag.fSpinBox->value());
			scenarioConf->add("I", robotSetDiag.bSpinBox->value());
			scenarioConf->add("distMAX", robotSetDiag.MaxDistSpinBox->value());
			scenarioConf->add("distMIN", robotSetDiag.MinDistSpinBox->value());
			scenarioConf->add("WIDTH", robotSetDiag.widthSpinBox->value());
			scenarioConf->add("HEIGHT", robotSetDiag.heightSpinBox->value());
			scenarioConf->add("sigmac", robotSetDiag.sigmacSpinBox->value());
			scenarioConf->add("sigmar", robotSetDiag.sigmarSpinBox->value());
			scenarioConf->add("sigmad", robotSetDiag.sigmadSpinBox->value());
			scenarioConf->add("camx", robotSetDiag.cameraxSpinBox->value());
			scenarioConf->add("camy", robotSetDiag.cameraySpinBox->value());
			scenarioConf->add("camz", robotSetDiag.camerazSpinBox->value());
			scenarioConf->add("NUMPOINTS", robotSetDiag.laserPointsSpinBox->value());
			scenarioConf->add("LASERMINANGLE", robotSetDiag.laserMinAngleSpinBox->value());
			scenarioConf->add("LASERMAXANGLE", robotSetDiag.laserMaxAngleSpinBox->value());
			scenarioConf->add("LASERMAXDIST", robotSetDiag.laserMaxDistSpinBox->value());
			scenarioConf->add("LASERMINDIST", robotSetDiag.laserMinDistSpinBox->value());
			scenarioConf->add("LASERSIGMA", robotSetDiag.laserSigmaSpinBox->value());
			scenarioConf->add("LASERX", robotSetDiag.laserxSpinBox->value());
			scenarioConf->add("LASERY", robotSetDiag.laserySpinBox->value());
			scenarioConf->add("LASERZ", robotSetDiag.laserzSpinBox->value());
			scenarioConf->add("alfa1", robotSetDiag.alfa1SpinBox->value());
			scenarioConf->add("alfa2", robotSetDiag.alfa2SpinBox->value());
			scenarioConf->add("alfa3", robotSetDiag.alfa3SpinBox->value());
			scenarioConf->add("alfa4", robotSetDiag.alfa4SpinBox->value());
			scenarioConf->add("VABSMAX", robotSetDiag.vmaxSpinBox->value());
			scenarioConf->add("wABSMAX", robotSetDiag.wmaxSpinBox->value());

			scene->initialize(numrobots,*scenarioConf, appConf->read<double>("SAMPLE_TIME"));
		}
	}
}

//////////////////////////////////////////////////////////////             END DIALOGS             ///////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////            OTHER OPTIONS            ////////////////////////////////////////////////

void MainWindow::changeNumRobots(int n){
	numrobots = n;
	if (mapLoaded){
		scene->setNumRobots(n);
		drawScene();
		drawRobots();
	}
}

void MainWindow::randomPoses(bool){
	if (mapLoaded){
		scene->reset();
		scene->randomPoses(appConf->read<bool>("GROUPINITIALPOSES") ,appConf->read<double>("MAXDISTINIPOSE"),  appConf->read<double>("MINDISTINIPOSE"));
		drawScene();
		drawRobots();
		// TODO: change the config with the new poses when saving
	}
}

void MainWindow::changeSlam(bool){
	if (actionRBPF->isChecked()){
		slamConf->add<int>("SLAM", 0);
	}	
	else if (actionEKF->isChecked()){
		slamConf->add<int>("SLAM", 1);
	}
	else if (actionEKF2->isChecked()){
		slamConf->add<int>("SLAM", 2);
	}
}


void MainWindow::changeStrategy(bool){
	if (expConf) delete expConf;
	if (actionHybrid->isChecked()){
		expConf = new ConfigFile((QDir::homePath() += "/.mrxt/config/tec0.config").toStdString().c_str());
	}
	else  if (actionBehaviour_Based->isChecked()){
		expConf = new ConfigFile((QDir::homePath() += "/.mrxt/config/tec1.config").toStdString().c_str());	
	}	
	else  if (actionNearest_Frontier->isChecked()){
		expConf = new ConfigFile((QDir::homePath() += "/.mrxt/config/tec2.config").toStdString().c_str());	
	}	
	else  if (actionCost_Utility->isChecked()){
		expConf = new ConfigFile((QDir::homePath() += "/.mrxt/config/tec3.config").toStdString().c_str());	
	}	
	else  if (actionCoordinated->isChecked()){
		expConf = new ConfigFile((QDir::homePath() += "/.mrxt/config/tec4.config").toStdString().c_str());	
	}	
	else  if (actionMarket_Based->isChecked()){
		expConf = new ConfigFile((QDir::homePath() += "/.mrxt/config/tec5.config").toStdString().c_str());	
	}	
	else  if (actionIntegrated->isChecked()){
		expConf = new ConfigFile((QDir::homePath() += "/.mrxt/config/tec6.config").toStdString().c_str());	
	}
	else expConf = 0;
}

/////////////////////////////////////////////////////////               END OTHER OPTIONS              //////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////             SIMULATION CONTROL             ///////////////////////////////////////////////////////////////

//
// Start Simulation
//
void MainWindow::startSimulation(bool){
	if (mapLoaded){
		actionPlay->setDisabled(true);
		randomPosesButton->setDisabled(true);
		actionStop->setEnabled(true);
		actionSaveVmap->setEnabled(false);
		actionSaveOmap->setEnabled(false);
		actionSaveSLAMlog->setEnabled(false);
		actionSaveGTlog->setEnabled(false);
		spinBox->setDisabled(true);
		myteam = new team(numrobots, *scene, *expConf, *slamConf, robots::EXPLORER, "explorers");
		QString tempLogpath = QDir::homePath() += "/.mrxt/outfiles/tempLog";
		myteam->setLogName(tempLogpath.toStdString().c_str());
		slam = myteam->getGlobalSlam();
		connect( slam, SIGNAL(slamUpdated(void)), this, SLOT(updateSlam(void)));
		QString tempLogGTpath = QDir::homePath() += "/.mrxt/outfiles/tempLogGT";
		scene->setLogName(tempLogGTpath.toStdString().c_str());
		scene->run();
		myteam->start();
		if (waitTh) delete waitTh;
		waitTh = new waitEnding(myteam);
		connect( waitTh, SIGNAL(simulationFinished(bool)), this, SLOT(stopSimulation(bool)));
		waitTh->start();
	}
}

//
// Stop Simulation
//
void MainWindow::stopSimulation(bool){
	disconnect( waitTh, SIGNAL(simulationFinished(bool)), this, SLOT(stopSimulation(bool)));
	printf("[GUI] Stop Simulation Requested\n");
	disconnect( slam, SIGNAL(slamUpdated(void)));
	visualMap* vm = slam->getVMap();
	slam = 0;
	myteam->stop();
	scene->stop();
	if (myteam){ delete myteam; myteam=0;}
	printf("[GUI] Simulation Stopped\n");
	scene->reset();
	actionStop->setDisabled(true);
	actionPlay->setEnabled(true);
	actionSaveVmap->setEnabled(true);
	actionSaveOmap->setEnabled(true);
	actionSaveSLAMlog->setEnabled(true);
	actionSaveGTlog->setEnabled(true);
	randomPosesButton->setEnabled(true);
	spinBox->setEnabled(true);

	double error = scene->evaluateVMap(*vm);
	double time = scene->getTime();
	delete vm;
	
	QString qstr = "- Exploration Time: " + QString::number(time) + " secs\n- Error in landmark based map: " + QString::number(error) + " m";
	QMessageBox::information(this, "Simulation Results", qstr);
}

//
// Update poses SLOT
//
// This slot is trigered by the simulation scene thread each step with the new real poses of the robots
//
void MainWindow::updatePoses(rposes poses){

	for (int r = 0; r< numrobots; r++){
		robotShapes[r]->setX(poses[r].x);
		robotShapes[r]->setY(-poses[r].y);
		//robotShapes[r]->setTransformOriginPoint(poses[r].x,poses[r].y);
		robotShapes[r]->setRotation(-poses[r].th*180.0/PI);
	}

	int msecs = (int)(1000.0f*scene->getTime());
	QTime ref;
	QTime qtime = ref.addMSecs(msecs);
	timeDisplay->display(qtime.toString(Qt::TextDate));
}

//
// Update SLAM SLOT
//
// This slot is trigered by the simulation SLAM thread each step 
//
void MainWindow::updateSlam(){
	if (slam){
		QPixmap* mapfig = slam->getPixmap(); 
		slamMapLabel->setPixmap(*mapfig);
		delete mapfig;
	}
}

//////////////////////////////////////////////////////////           END SIMULATION CONTROL             ///////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////               VISUALIZATION                  ///////////////////////////////////////////////////////////////

//
// Refresh Poses
//
// This method updates the visualization of the simulated scene
//
void MainWindow::drawRobots(){

//	QPixmap* mapfig = scene->getPixmap();
//	mapLabel->setPixmap(*mapfig);
//	delete mapfig;

	// get poses
	rposes poses = scene->getPoses();
//	char entrada[50];
//	for (int r = 0; r< numrobots; r++){
//		sprintf(entrada,"R%i",r+1);
//		string robotpos = scenarioConf->read<string>(entrada);
//		StringTokenizer st(&robotpos[0]);

//		float x = atof(st.nextToken());
//		float y = atof(st.nextToken());
//		float th = atof(st.nextToken());
//		pose pos(x,y,th);

//		//pose pos = scene->getPos(r); //ESTO NO EXISTE!
//		poses.push_back(pos);
//	}

	// get footprint
	std::vector<double> robot_footprint;
	string robotpos = scenarioConf->read<string>("FOOTPRINT");
	StringTokenizer st(&robotpos[0]);
	robot_footprint.clear();
	for (int t = 0; t < st.countTokens(); t++){
		float pt = atof(st.nextToken());
		robot_footprint.push_back(pt);
	}	
	int total_lines2 = robot_footprint.size();

	// create robot shape
	rlines robots_lines;
	std::vector<line> newline;
	robotShapes.clear();
	QGraphicsItemGroup* newgroup;
	for (int r = 0; r< numrobots; r++){
		robots_lines.push_back(newline);
		robotShapes.push_back(newgroup);
		QList<QGraphicsItem *> list;
		QPolygonF polygon;
		for (int l = 0; l < total_lines2; l+=2){
			float x1 = robot_footprint[l];
			float y1 = robot_footprint[l+1];	
			float x2 = robot_footprint[(l+2)%total_lines2];
			float y2 = robot_footprint[(l+3)%total_lines2];
			line lin(	x1*cos(poses[r].th)-y1*sin(poses[r].th),
					-(x1*sin(poses[r].th)+y1*cos(poses[r].th)),
					x2*cos(poses[r].th)-y2*sin(poses[r].th),
					-(x2*sin(poses[r].th)+y2*cos(poses[r].th)));
			robots_lines[r].push_back(lin);
			polygon << QPointF(x1,y1);
			//QGraphicsLineItem* rl = qscene.addLine(lin.x1, lin.y1, lin.x2, lin.y2,QPen(QColor(255,0,0)));
			//list.push_back(rl);
		}
		QGraphicsItem* pol = qscene.addPolygon(polygon,QPen(QColor(0,0,0)),QBrush(QColor(255,0,0),Qt::SolidPattern));
		list.push_back(pol);
		QGraphicsLineItem* rlori = qscene.addLine(0, 0, 0.4 , 0,QPen(QColor(0,0,0)));
		list.push_back(rlori);
		robotShapes[r] = qscene.createItemGroup(list);
	}
	updatePoses(poses);
}

void MainWindow::sceneZoom(int zoomSlider){
	double newscale = zoomSlider;
	double zoomval = newscale/scale; 
	view.scale(zoomval,zoomval);
	scale = newscale;
}

void MainWindow::drawScene(){

	while (qscene.items().size() > 0){
		qscene.removeItem(qscene.items().at(0));
	}

	char entrada[20];
	
	// walls / obstacles
	int numwalls = scenarioConf->read<int>("NUMWALLS");
	for (int i = 0; i< numwalls; i++){
		sprintf(entrada,"WALL%i",i+1);
		string wallstr = scenarioConf->read<string>(entrada);
		StringTokenizer st(&(wallstr[0]));
		float x1 = atof(st.nextToken());
		float y1 = -atof(st.nextToken());
		float x2 = atof(st.nextToken());
		float y2 = -atof(st.nextToken());
		qscene.addLine( x1, y1, x2, y2 );
	}

	// walls marks
	int nummarks = scenarioConf->read<int>("NUMFEATURES");
	for (int i = 0; i< nummarks; i++){
		sprintf(entrada,"FEAT%i",i+1);
		string featstr = scenarioConf->read<string>(entrada);
		StringTokenizer st(&(featstr[0]));
		float x = atof(st.nextToken());
		float y = -atof(st.nextToken());
		float z = atof(st.nextToken());
		float desc = atof(st.nextToken());
		qscene.addRect(x-0.05,y-0.05,0.1,0.1,QPen(QColor(0,100,0)),QBrush(QColor(0,100,0),Qt::SolidPattern));
	}

	if (!scaleInitialized){	
		view.scale(scale,scale);
		scaleInitialized = true;
	}
}

//////////////////////////////////////////////////////////             END VISUALIZATION                ///////////////////////////////////////////////////////////////





