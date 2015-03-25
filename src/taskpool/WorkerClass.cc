#include "WorkerClass.h"

WorkerClass::WorkerClass() :
    ThreadsafeClass(), threadRunning(false), runPeriod(0.0) {};

WorkerClass::~WorkerClass() {
    Stop();
}

void WorkerClass::Start() {
    WriteLock lock(mutex);
    if(threadRunning) { return; }
    threadRunning = true;
    workerThread = boost::thread( boost::bind(&WorkerClass::Spin, this) );
}

void WorkerClass::Stop() {
    WriteLock lock(mutex);
    if(!threadRunning) { return; }
    workerThread.interrupt();
    //threadStopRequested = true;
}

void WorkerClass::Spin() {

    try {
        while(true) {
            boost::this_thread::interruption_point();
            Process();
            if(runPeriod == 0.0) {
                boost::this_thread::yield();
            } else {
                boost::posix_time::time_duration td =
                    boost::posix_time::milliseconds(runPeriod*1E3);
                boost::this_thread::sleep(td);
            }
        }
    } catch(boost::thread_interrupted e) {}
        
    threadRunning = false;
}

void WorkerClass::Join() {
    if(!threadRunning) { return; }
    workerThread.join();
}

void WorkerClass::SetRunFrequency(float freq) {
    WriteLock lock(mutex);
    runPeriod = 1.0/freq;
}