/*
*
* Author: Miguel Julia <mjulia@umh.es> 
* 
* Date:   2009
* 
* Class ComThead
*
*/

#pragma once
#ifndef __CLASS__COMMUNICATIVE__THREAD__
#define __CLASS__COMMUNICATIVE__THREAD__

#include "ClThread.h"
#include "ClMutex.h"
#include <list>
#include <map>
#include <string>


/**
* @brief Implements communication between threads
*
* Objects of classes derived from ComThread can send (to a specific address) or broadcast (to everybody) messages to other ComThread derived objects
*
* The messages received are cummulated in a FIFO queue that can be read at any moment
*
*/
class ComThread : public ClThread{
public:
	/// constructor
	ComThread();
	/// destroyer
	virtual ~ComThread();

protected:
	/// Sends a message to a specific address (dir < 0 to broadcast)
	void sendMessage(const std::string& msg, int dir);
	/// Reads the oldest message in the queue
	int getMessage(std::string& msg);
	/// Returns the communication address of the object
	int getDir(){return mydir;};

private:

	static int nextId;
	static std::map<int,std::list<std::string>*> idmap;
	static ClMutex mut;

	std::list<std::string> messageQueue;
	int mydir;

	/// this virtual method will be executed when trying to start the thead, must return 0 if ok
	virtual int setup(){return 0;};

	/// this virtual method will be executed when the stop method is called
	virtual void onStop(){};

	/// virtual method for the thread main process. 
	virtual void execute(){};

};


#endif

