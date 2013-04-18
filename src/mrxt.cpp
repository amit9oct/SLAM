
#include "team.h"
#include "simulatedModel.h"
#include "time.h"
#include <opencv2/opencv.hpp>
#include <QApplication>
//#include "ui_mainWindow.h"
#include "mainwindow.h"
#include <QFile>
#include <QDir>

using namespace std;


/*! \mainpage Multi-Robot eXploration Tool Documentation
 *
 * This is source code documentation.
 *
 * Visit http://arvc.umh.es/~mjulia/mrxt for a general description of the application.
 *
 */
int main(int argc, char **argv)
{

	if(!QFile::exists( QDir::homePath()+="/.mrxt")){
		printf("initializing the config folder...\n");
		QDir::home().mkdir(".mrxt");
		QDir currentfolder(QDir::homePath());
		currentfolder.cd(".mrxt");
		currentfolder.mkdir("maps");
		currentfolder.mkdir("config");		
		currentfolder.mkdir("outfiles");

		QFile::copy( "/usr/share/mrxt/maps/scene1.map", QDir::homePath()+="/.mrxt/maps/scene1.map");
		QFile::copy( "/usr/share/mrxt/maps/scene1.omap", QDir::homePath()+="/.mrxt/maps/scene1.omap");
		QFile::copy( "/usr/share/mrxt/maps/scene1.jpg", QDir::homePath()+="/.mrxt/maps/scene1.jpg");
		QFile::copy( "/usr/share/mrxt/maps/scene2.map", QDir::homePath()+="/.mrxt/maps/scene2.map");
		QFile::copy( "/usr/share/mrxt/maps/scene2.omap", QDir::homePath()+="/.mrxt/maps/scene2.omap");
		QFile::copy( "/usr/share/mrxt/maps/scene2.jpg", QDir::homePath()+="/.mrxt/maps/scene2.jpg");
		QFile::copy( "/usr/share/mrxt/config/app.config", QDir::homePath()+="/.mrxt/config/app.config");
		QFile::copy( "/usr/share/mrxt/config/slam.config", QDir::homePath()+="/.mrxt/config/slam.config");
		QFile::copy( "/usr/share/mrxt/config/tec0.config", QDir::homePath()+="/.mrxt/config/tec0.config");
		QFile::copy( "/usr/share/mrxt/config/tec1.config", QDir::homePath()+="/.mrxt/config/tec1.config");
		QFile::copy( "/usr/share/mrxt/config/tec2.config", QDir::homePath()+="/.mrxt/config/tec2.config");
		QFile::copy( "/usr/share/mrxt/config/tec3.config", QDir::homePath()+="/.mrxt/config/tec3.config");
		QFile::copy( "/usr/share/mrxt/config/tec4.config", QDir::homePath()+="/.mrxt/config/tec4.config");
		QFile::copy( "/usr/share/mrxt/config/tec5.config", QDir::homePath()+="/.mrxt/config/tec5.config");
		QFile::copy( "/usr/share/mrxt/config/tec6.config", QDir::homePath()+="/.mrxt/config/tec6.config");
		//exit(0);
	}

	QApplication app( argc, argv );

	MainWindow mywin;

	app.exec();


	return (0);
} 

