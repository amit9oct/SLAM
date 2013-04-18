/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2008
* 
* Class ClMutex
*
*/

#ifndef __CLMUTEX__
#define __CLMUTEX__

#ifndef WIN32
#include <pthread.h>
#else
#include <windows.h>
#endif


/**
* @brief Implements a platform independent mutual exclusion mechanism 
*/
class ClMutex
{
private:

#ifdef WIN32
	HANDLE m_mutex;
#else
	pthread_mutex_t m_mutex;
#endif

public:

	/// constructor
	ClMutex(void);
	/// destructor
	virtual ~ClMutex();

	/// locks the mutex
	void lock();
	/// unlocks the mutex
	void unlock();


};
#endif

