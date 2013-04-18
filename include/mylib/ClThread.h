/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class ClThread
*
*/
#pragma once
#ifndef __CLASS__THREAD__
#define __CLASS__THREAD__

#ifndef WIN32
#include <pthread.h>
#else
#include <windows.h>
#endif

#include <stdio.h>

/**
* @brief Implements a platform-independent threaded superclass 
*
* Derive a class from this one and implement the vitual methods:
*
* - setup 
* - onStop 
* - execute 
*/
class ClThread{

protected:

	int prio;
	size_t stacksize;

public:
	/// Constructor
	ClThread();
	/// Destructor
	virtual ~ClThread();
	/// runs the setup method and starts the thread
	int run();
	/// calls the onStop method and stops the thread
	void stop();
	/// Prints the Thread ID
	void printID(char* str) { printf("THREAD ID %d: %s\n",(int)thread_id, str); };

private:

	#ifndef WIN32
	pthread_t m_thread;
	pthread_t thread_id;
	pthread_attr_t tattr;
	#else
	HANDLE m_thread;
	DWORD  thread_id;
	#endif

	bool running;

	/// Thread entry point function
	#ifndef WIN32 
	static void* EntryPoint(void * pthis);
	#else
	static DWORD WINAPI EntryPoint(void * pthis);
	#endif 

	/// this virtual method will be executed when trying to start the thead, must return 0 if ok
	virtual int setup(){return 0;};

	/// this virtual method will be executed when the stop method is called
	virtual void onStop(){};

	/// virtual method for the thread main process. 
	virtual void execute(){};

	
};

void sleepms(int milliseconds);

#endif

