
#include "ClThread.h"
#ifndef WIN32
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <sys/time.h> // getrlimit()
#include <sys/resource.h> // getrlimit()
#else
#include <windows.h>
#endif
#include <stdio.h>


ClThread::ClThread():
	stacksize(0),
	running(false)
{
	//printf("[CLTHREAD] Th class created \n");
}

ClThread::~ClThread(){
	stop();
	//printf("[CLTHREAD] Th class destroyed \n");
}

int ClThread::run(){
#ifndef WIN32
	int ret;
	sched_param param;
#endif

	if(!setup()){ 
		
		#ifndef WIN32
		sigset_t mask, oldmask;
		sigemptyset (&mask);
		sigaddset (&mask, SIGUSR1);
		pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

		/* initialized with default attributes */
		ret = pthread_attr_init (&tattr);

		/* safe to get existing scheduling param */
		ret = pthread_attr_getschedparam (&tattr, &param);

		/* set the priority; others are unchanged */
		//param.sched_priority = prio;

		/* setting the new scheduling param */
		ret = pthread_attr_setschedparam (&tattr, &param);

		if (stacksize) pthread_attr_setstacksize (&tattr, stacksize);
		
		pthread_create(&thread_id, &tattr, ClThread::EntryPoint, (void*) this);

		#else
		DWORD m_stackSize = 0;
		m_thread = CreateThread(NULL, m_stackSize, ClThread::EntryPoint, (LPVOID) this, 0, &thread_id);
		#endif
		running = true;
		return(0);
	}
	else 
		return(-1);
}

void ClThread::stop()
// This method closes the connection
{
	if(running){
		running = false;
		onStop();
		#ifndef WIN32
		pthread_cancel(thread_id); 				// kill thread
		pthread_detach(thread_id);	
		#else
		//TerminateThread(m_thread,0);
		CloseHandle( m_thread );
		#endif
	}
}

#ifndef WIN32 
void *ClThread::EntryPoint(void * pthis)
#else
DWORD WINAPI ClThread::EntryPoint(void * pthis)
#endif 
// Thread entry point. The thread start function must be static. In order to uses non 
// static methods and properties we make use of this function.
// This static function calls the non static method execute() receiving the whole object as a
// parameter
//
{
	ClThread *th = (ClThread*)pthis;  	// cast the object
	th->execute();
#ifndef WIN32
	pthread_exit(NULL);
#else
	ExitThread(NULL);
#endif
	return 0;
}

void sleepms(int milliseconds){
#ifdef WIN32
	Sleep(milliseconds);
#else
	usleep(1000*milliseconds);
#endif

}
