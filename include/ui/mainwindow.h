
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainWindow.h"
#include "ConfigFile.h"
#include "simulatedModel.h"
#include "team.h"
#include <QPixmap>
#include <QActionGroup>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QThread>


/**
* @brief Class used to attend the stop button
*
*/
class waitEnding : public QThread{
	Q_OBJECT;
public:
	team* t;
	waitEnding(team* theteam): t(theteam){};
	void run();

signals:
	void simulationFinished(bool);

};


/**
* @brief QT main window. Implements the slots for the events.
*
*/
class MainWindow : public QMainWindow, public Ui::MainWindow
{
    	Q_OBJECT
private:

	int numrobots;
	ConfigFile* expConf;
	ConfigFile* scenarioConf;
	ConfigFile* slamConf;
	ConfigFile* appConf;
	simulatedModel* scene;
	team* myteam;
	slamInterface* slam;
	bool mapLoaded;
	QActionGroup* strategiesGroup;
	QGraphicsScene qscene;
	QGraphicsView view;
	double scale;
	bool scaleInitialized;
	std::vector<QGraphicsItemGroup*> robotShapes;
	waitEnding* waitTh;

	void drawRobots();
	void drawScene();


public:

    	MainWindow(QWidget *parent = 0);

	~MainWindow();

public slots:

	void randomPoses(bool checked);
	void openScenario(bool checked);
	void saveOmap(bool checked);
	void saveVmap(bool checked);
	void saveSLAMlog(bool checked);
	void saveGTlog(bool checked);
	void changeNumRobots(int n);
	void updatePoses(rposes poses);
	void updateSlam();
	void startSimulation(bool);
	void stopSimulation(bool);
	void openScenePropDiag(bool=true);
	void openRobotSetDiag(bool=true);
	void openStrategyOptDiag(bool=true);
	void openSLAMOptSetDiag(bool=true);
	void openAboutDiag(bool);
	void openAppConfigDiag(bool);
	void changeStrategy(bool=true);
	void changeSlam(bool=true);
	void sceneZoom(int newscale);

};
#endif
