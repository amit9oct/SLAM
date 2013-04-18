#include "ClMutex.h"
#include <stdio.h>

ClMutex::ClMutex(void)
{
#ifdef WIN32
   m_mutex = CreateMutex(NULL,FALSE,NULL);
#else
   pthread_mutexattr_t mattr;

   pthread_mutexattr_init( &mattr );
   pthread_mutex_init(&m_mutex,&mattr);

#endif
}

ClMutex::~ClMutex(void)
{
#ifdef WIN32
	WaitForSingleObject(m_mutex,INFINITE);
	CloseHandle(m_mutex);
#else
	pthread_mutex_lock(&m_mutex);
	pthread_mutex_unlock(&m_mutex);
	pthread_mutex_destroy(&m_mutex);
#endif
}

void
ClMutex::lock()
{
#ifdef WIN32
	WaitForSingleObject(m_mutex,INFINITE);
#else
	pthread_mutex_lock(&m_mutex);
#endif
}

void 
ClMutex::unlock()
{
#ifdef WIN32
	ReleaseMutex(m_mutex);
#else
	pthread_mutex_unlock(&m_mutex);
#endif
}

