#ifndef _TASK_POOL_H_
#define _TASK_POOL_H_

#include "TaskFactory.h"
#include "Task.h"
#include "ThreadsafeClass.h"

#include <boost/thread.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <memory>
#include <queue>

/*! \class TaskPool TaskPool.h
 * \brief provides a set of threads that execute tasks from a queue. */
class TaskPool : private ThreadsafeClass {
public:

    typedef std::shared_ptr<TaskPool> Ptr;
    
    TaskPool(unsigned int numThreads);
    ~TaskPool();

    // TODO Implement!
    void AddThreads(unsigned int numThreads);
//     void RemoveThreads(unsigned int numThreads);

    void AddTask(Task::Ptr task);
    
protected:

    boost::thread_group threads;
    boost::interprocess::interprocess_semaphore taskCount;
    std::queue<Task::Ptr> tasks;
    bool ended;
    
    Task::Ptr GetTask();
    void SetEnded();
    bool GetEnded();
    void ThreadProcess();
};

#endif