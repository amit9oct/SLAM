#include "reactive.h"
#include "math.h"

reactive::reactive(const ConfigFile& config):
	vmax				(config.read<float>("VMAX")),
	wmax				(config.read<float>("WMAX")),
	k1				(config.read<float>("K1")),
	k2				(config.read<float>("K2")),
	resolution			(0),
	avoidObstaclesEnabled		(true),
	goToFrontierEnabled		(true),
	goToUnexploredZonesEnabled	(true),
	avoidOtherRobotsEnabled		(true),
	goToPrecisePosesEnabled		(false),
	goToGoalEnabled			(false),
	avoidObstaclesWeight		(config.read<float>("WEIGHTAVOBS")),
	goToFrontierWeight		(config.read<float>("WEIGHTGOFRO")),
	goToUnexploredZonesWeight	(config.read<float>("WEIGHTGOUZ")),
	avoidOtherRobotsWeight		(config.read<float>("WEIGHTAVROB")),
	goToPrecisePosesWeight		(config.read<float>("WEIGHTGOPRE")),
	goToGoalWeight			(config.read<float>("WEIGHTGOGOAL")),
	avoidObstaclesWidth		(config.read<float>("SIGMAAVOBS")),
	goToFrontierWidth		(config.read<float>("SIGMAGOFRO")),
	goToUnexploredZonesWidth	(config.read<float>("SIGMAGOUZ")),
	avoidOtherRobotsWidth		(config.read<float>("SIGMAAVROB")),
	goToPrecisePosesWidth		(config.read<float>("SIGMAGOPRE")),
	goToGoalWidth			(config.read<float>("SIGMAGOGOAL")),
	localMinimumDetected		(false)
{
}


speed reactive::regulator(const pose& pos, pointf& f){
	speed velo;

	if (f.x!=0 || f.y!=0){
		float ang = (atan2(f.y, f.x) - pos.th );
		if (ang>=PI) ang += (float)(-2*PI);
		else if (ang<-PI) ang += (float)(2*PI);

		velo.w = k1*ang;
		if (velo.w>=wmax) velo.w = wmax;
		else if (velo.w<-wmax)	velo.w = -wmax;

		velo.v = vmax/(k2*fabs(ang)+1);

	}
	else{
		velo.v=0;
		velo.w=0;
	}
	return velo;
}

void reactive::disableAll(){
	goToUnexploredZonesEnabled=false;
	goToFrontierEnabled=false;
	avoidOtherRobotsEnabled=false;
	avoidObstaclesEnabled=false;
	goToPrecisePosesEnabled=false;
	goToGoalEnabled=false;
}

pointf reactive::getResponse(localPotentialField& lpf){
	
	pointf ctrlAction;
	int i,j;
	float vali=0;
	float valj=0;
			
	float min=lpf.get(0,0);
	float max=lpf.get(0,0);
	int mini=0, minj=0;

	pose currentPos = mySlam->getPos(number);
	point currentCell = mySlam->getCell(number);

	float incX = (currentPos.x - ((currentCell.x+0.5)*mySlam->getResolution() + mySlam->getXOrigin()))/mySlam->getResolution();
	float incY = (currentPos.y - ((currentCell.y+0.5)*mySlam->getResolution() + mySlam->getYOrigin()))/mySlam->getResolution();

	//printf("x: %f y: %f - cellx: %d, celly: %d, incX: %f, incY: %f\n",currentPos.x, currentPos.y, currentCell.x, currentCell.y,incX,incY);

	float sum=0;

	for(i=0;i< lpf.getWidth(); i++){
		for(j=0;j< lpf.getHeight(); j++){
			if (min > lpf.get(i,j)){
				min = lpf.get(i,j);
				mini=i;
				minj=j;
			}
			if (max < lpf.get(i,j))	max = lpf.get(i,j);
		}
	}
	
	int imin,imax,jmin,jmax;

	imin = (mini-1>=0)? mini-1:0;
	jmin = (minj-1>=0)? minj-1:0;
	imax = (mini+1<lpf.getWidth())? mini+1 : lpf.getWidth()-1;
	jmax = (minj+1<lpf.getHeight())? minj+1 : lpf.getHeight()-1;

	//printf("%d,%d,%d,%d\n", imin,imax,jmin,jmax);
	for(i=imin;i<=imax; i++){
		for(j=jmin;j<=jmax; j++){
			float aux = ((max-min)-(lpf.get(i,j)-min));
			vali += aux*i;
			valj += aux*j;
			sum += aux;
		}
	}

	if (sum==0){
		ctrlAction.x = 0;
		ctrlAction.y = 0;
	}
	else{
		ctrlAction.x = ((vali-incX)/sum)-(lpf.getWidth()-1)/2;
		ctrlAction.y = ((valj-incY)/sum)-(lpf.getHeight()-1)/2;
	}
	//printf("fx: %f, fy: %f\n",ctrlAction.x, ctrlAction.y);

	if ((ctrlAction.x*ctrlAction.x + ctrlAction.y*ctrlAction.y) < 1.5){
		localMinimumDetected = true;
		//printf("LOCAL MINIMUM DETECTED!!!!!!\n");
	}
	else
		localMinimumDetected = false;

	return ctrlAction;
}

