
#include "stepEvent.h"
#include <errno.h>
#include <time.h>

using namespace std;

stepEventManager::stepEventManager(){
	currentStep = 0;
}

void stepEventManager::waitForStep(int step){
	//printf("Waiting for step %d, currenstep = %d\n", step, currentStep);
	if (step<=currentStep){
		return;
	}
	else{
		//printf("new waiting thread   <<<<--------------------\n");
		waitingThread* wth = new waitingThread();
		//printf("setstep\n");
		wth->setStep(step);
		//printf("lock\n");
		mut.lock();
		//printf("push_back\n");
		waitingList.push_back(wth);
		//printf("unlock\n");
		mut.unlock();

		//printf("waiting...\n");
		wth->wait();
		//printf("ok\n");

		mut.lock();
		vector<waitingThread*>::iterator wListIter;
		for(wListIter = waitingList.begin(); wListIter != waitingList.end(); wListIter++){
			if ((*wListIter)==wth) {
				waitingList.erase(wListIter);
				break;
			}
		}
		mut.unlock();
		delete(wth);
	}
}

void stepEventManager::step(){
	currentStep++;
	vector<waitingThread*>::iterator wListIter;
	//printf("----------------  SEM STEP %d  ------------------\n",currentStep);
	mut.lock();
	for(wListIter = waitingList.begin(); wListIter != waitingList.end(); wListIter++){
		//printf("STEP - a thread is waiting for step %d, in step %d\n",(*wListIter)->getStep(), currentStep);
		if ((*wListIter)->getStep() <= currentStep) {
			(*wListIter)->post();
		}
	}
	mut.unlock();
}


// ----------------------------------------------------------------------------------------------------


waitingThread::waitingThread(){	

	int value;

	// initialize a private semaphore 
	value = 0;	

#ifdef WIN32
	sem = CreateSemaphore(NULL,value, 1, NULL);
#else
	pthread_cond_init(&cond,NULL);
#endif
}

waitingThread::~waitingThread(){
#ifdef WIN32
	CloseHandle(sem);
#else
	pthread_cond_destroy(&cond);
#endif
}

void waitingThread::wait(){
#ifdef WIN32
	WaitForSingleObject(sem,INFINITE);
#else
	//printf("Waiting Thread::wait step %d - cond: %p...\n", stepToUnclock, &cond);

	pthread_mutex_t mutex;
	pthread_mutex_init(&mutex,NULL);
	pthread_mutex_lock(&mutex);
	pthread_cond_wait(&cond, &mutex);
	pthread_mutex_unlock(&mutex);
	pthread_mutex_destroy(&mutex);
#endif
	posted = false;
}

void waitingThread::post(){
#ifdef WIN32
	ReleaseSemaphore(sem, 1, NULL);
#else
	pthread_cond_signal(&cond);
#endif
	posted = true;
}

void waitingThread::setStep(int step){
	stepToUnclock = step;
}

int waitingThread::getStep(){
	return stepToUnclock;
}

bool waitingThread::isPosted(){
	return posted;
}

production::production(int size):
itemProd(0),
totalSize(size),
prodWaiting(0),
consWaiting(0)
{
#ifndef WIN32
	pthread_mutex_init(&mutex,NULL);
	pthread_cond_init(&producerWaiting,NULL);
	pthread_cond_init(&consumerWaiting,NULL);
#else
	mutex = CreateMutex(NULL,FALSE,NULL);
	producerWaiting = CreateSemaphore(NULL,0, 1, NULL);
	consumerWaiting = CreateSemaphore(NULL,0, 1, NULL);
#endif
}

production::~production(){
#ifndef WIN32
	pthread_cond_destroy(&consumerWaiting);
	pthread_cond_destroy(&producerWaiting);
	pthread_mutex_destroy(&mutex);
#else
	CloseHandle(mutex);
	CloseHandle(producerWaiting);
	CloseHandle(consumerWaiting);
#endif
}

void production::beginProduction(){
#ifndef WIN32
	pthread_mutex_lock(&mutex);
//	printf("prod locks\n");
	while (itemProd >= totalSize){
//		printf("prod wait...\n");
		prodWaiting++;
		pthread_cond_signal(&consumerWaiting);
		pthread_cond_wait(&producerWaiting,&mutex);
//		printf("producer signaled\n");
		prodWaiting--;
	}
	pthread_mutex_unlock(&mutex);
#else
//	printf("prod locks\n");
	WaitForSingleObject(mutex,INFINITE);
	while (itemProd >= totalSize){
//		printf("prod wait...\n");
		prodWaiting++;
		ReleaseSemaphore(consumerWaiting, 1, NULL);
		SignalObjectAndWait(mutex,producerWaiting,INFINITE,false);
//		printf("producer signaled\n");
		prodWaiting--;
	}
	ReleaseMutex(mutex);
#endif
}

void production::endProduction(){
#ifndef WIN32
	pthread_mutex_lock(&mutex);
	itemProd++;
//	printf("items = %d\n",itemProd);
//	printf("end prod\n");
	if (consWaiting>0){
		pthread_cond_signal(&consumerWaiting);
	}
//	printf("prod unlocks\n");
	pthread_mutex_unlock(&mutex);
#else
	WaitForSingleObject(mutex,INFINITE);
	itemProd++;
//	printf("items = %d\n",itemProd);
//	printf("end prod\n");
	if (consWaiting>0){
		ReleaseSemaphore(consumerWaiting, 1, NULL);
	}
//	printf("prod unlocks\n");
	ReleaseMutex(mutex);
#endif
}

void production::beginConsumition(){
#ifndef WIN32
	pthread_mutex_lock(&mutex);
//	printf("cons locks\n");
	while (itemProd == 0){
//		printf("cons wait...\n");
		consWaiting++;
		pthread_cond_signal(&producerWaiting);
		pthread_cond_wait(&consumerWaiting,&mutex);
//		printf("consumer signaled\n");
		consWaiting--;
	}
	pthread_mutex_unlock(&mutex);
#else
//	printf("\t\t\tcons locks\n");
	WaitForSingleObject(mutex,INFINITE);
	while (itemProd == 0){
//		printf("\t\t\tcons wait...\n");
		consWaiting++;
		ReleaseSemaphore(producerWaiting, 1, NULL);
		SignalObjectAndWait(mutex,consumerWaiting,INFINITE,false);
//		printf("\t\t\tconsumer signaled\n");
		consWaiting--;
	}
	ReleaseMutex(mutex);
#endif
}

void production::endConsumition(){
#ifndef WIN32
	pthread_mutex_lock(&mutex);
	itemProd--;
//	printf("items = %d\n",itemProd);
//	printf("end cons\n");
	if (prodWaiting>0){
		pthread_cond_signal(&producerWaiting);
	}
//	printf("cons unlocks\n");
	pthread_mutex_unlock(&mutex);
#else
	WaitForSingleObject(mutex,INFINITE);
	itemProd--;
//	printf("\t\t\titems = %d\n",itemProd);
//	printf("\t\t\tend cons\n");
	if (prodWaiting>0){
		ReleaseSemaphore(producerWaiting, 1, NULL);
	}
//	printf("\t\t\tcons unlocks\n");
	ReleaseMutex(mutex);
#endif
}




