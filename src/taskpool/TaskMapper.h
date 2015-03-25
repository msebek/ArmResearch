#ifndef _TASK_MAPPER_H_
#define _TASK_MAPPER_H_

#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include "TaskPool.h"
#include "Factory.h"

/*! \class TaskMapper TaskMapper.h
 * \brief pulls tasks out of a factory and assigns them to a TaskPool. */
class TaskMapper : public Task {
public:

    typedef Factory<Task::Ptr> TaskFactory;

    struct Parameters : public Task::Parameters {
        TaskPool::Ptr taskSink;
        TaskFactory::Ptr taskSource;
        unsigned int bufferSize;
    };
    
    TaskMapper(Parameters &params);
    ~TaskMapper();

protected:

    Parameters parameters;
    boost::interprocess::interprocess_semaphore bufferCount;
    
    void ExecuteTask();
    void TaskEndCallback(Task* task);

};

#endif