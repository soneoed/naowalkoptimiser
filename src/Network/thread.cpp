#include "thread.h"
#include <iostream>

using namespace std;

Thread::Thread(string _name): name(_name), running(false){
	cout << "Thread Created: " << name << endl;
}

int Thread::start(){
	if(running) 
		return -1;

	cout << "Starting Thread: " << name << endl;

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	// Create the thread.
	const int result = pthread_create(&thread, &attr, runThread, (void*)this);

	pthread_attr_destroy(&attr);
	return result;
}

void Thread::stop(){
	running = false;
}

void* Thread::runThread(void* _this){
	reinterpret_cast<Thread*>(_this)->run();
	cout << "Exiting Thread: " << reinterpret_cast<Thread*>(_this)->name << endl; 
	pthread_exit(NULL);	
}
