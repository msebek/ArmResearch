#include "TaskPool.h"
#include <boost/foreach.hpp>
#include <boost/bind.hpp>

TaskPool::TaskPool(unsigned int numThreads) :
    taskCount(0), ended(false) {
    AddThreads(numThreads);
}

// HACK
TaskPool::~TaskPool() {
    threads.interrupt_all();
}

void TaskPool::AddThreads(unsigned int numThreads) {
    for(unsigned int i = 0; i < numThreads; i++) {
        threads.create_thread( boost::bind( &TaskPool::ThreadProcess, this ) );
    }
}

void TaskPool::ThreadProcess() {
    try {
        while(true) {

            taskCount.wait();

            Task::Ptr task = GetTask();
            task->Run();

        }
    } catch(boost::thread_interrupted e) {}
    
}

void TaskPool::AddTask(Task::Ptr task) {
    if(!task) {
        std::cout << "TaskPool: Received null task." << std::endl;
    }
    
    WriteLock lock(mutex);
    tasks.push(task);
    taskCount.post();
}

Task::Ptr TaskPool::GetTask() {
    WriteLock lock(mutex);
    Task::Ptr task = tasks.front();
    tasks.pop();
    return task;
}

void TaskPool::SetEnded() {
    WriteLock lock(mutex);
    ended = true;
}

bool TaskPool::GetEnded() {
    ReadLock lock(mutex);
    return ended;
}