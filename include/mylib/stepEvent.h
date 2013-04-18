#pragma once
#ifndef __STEP_EVENT__
#define __STEP_EVENT__

#include "ClMutex.h"
#include <vector>
#ifndef WIN32
//#include <semaphore.h>
#endif

/**
* @brief Thread synchronization auxiliary class for waiting thread queues
*
*/
class waitingThread{
	private:
		#ifdef WIN32
		HANDLE sem;
		#else
		pthread_cond_t cond;
		#endif
		int stepToUnclock;
		bool posted;
	public:
		waitingThread();
		virtual ~waitingThread();
		void wait();
		void post();
		void setStep(int step);
		int getStep();
		bool isPosted();
};

/**
* @brief Thread synchronization using a waif for step model
*
*/
class stepEventManager
{
private:


	ClMutex mut;
	int currentStep;
	std::vector<waitingThread*> waitingList;
public:
	stepEventManager();
	void waitForStep(int step); 
	void step();
	int getStep(){return currentStep;};
	void resetCounter(){currentStep=0;};
};


/**
* @brief Thread synchronization with a producer/consumer model
*
*/
class production{
private:
	int itemProd;
	int totalSize;
	int prodWaiting;
	int consWaiting;

#ifndef WIN32
	pthread_mutex_t mutex;
	pthread_cond_t producerWaiting;
	pthread_cond_t consumerWaiting;
#else
	HANDLE mutex;
	HANDLE producerWaiting;
	HANDLE consumerWaiting;
#endif

public:
	production(int size=1);
	virtual ~production();
	
	void beginProduction();
	void endProduction();

	void beginConsumition();
	void endConsumition();
		
};


#endif

