
#include "ComThread.h"

using namespace std;

int ComThread::nextId=0;
std::map<int,std::list<string>*> ComThread::idmap;
ClMutex ComThread::mut;

ComThread::ComThread():
	ClThread()
{
	mut.lock();
	mydir = ++nextId;
	idmap[mydir]= &messageQueue;
	mut.unlock();
}

ComThread::~ComThread(){
	mut.lock();
	idmap.erase(idmap.find(mydir));
	mut.unlock();
};

void ComThread::sendMessage(const string& msg, int dir){
	mut.lock();
	map<int,std::list<string>*>::iterator it;
	if (dir>=0){
		it = idmap.find(dir);
		if (it->second) it->second->push_back(msg);
	}
	else{
		for ( it=idmap.begin() ; it != idmap.end(); it++ )
			if (it->first != mydir) it->second->push_back(msg);
	}
	mut.unlock();
};

int ComThread::getMessage(string& msg){
	int res = 0;
	mut.lock();
	if (messageQueue.size()>0){
		msg = messageQueue.front();
		messageQueue.pop_front();
		res = msg.size();
	}
	mut.unlock();
	return res;
};
