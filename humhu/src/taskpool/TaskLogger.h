#ifndef _TASK_LOGGER_H_
#define _TASK_LOGGER_H_

#include "Task.h"
#include "Factory.h"
#include "ThreadsafeClass.h"
#include <iostream>
#include <fstream>

class TaskLogger : public Factory<Task::Ptr> {
public:

    typedef Factory<Task::Ptr> TaskFactory;

    TaskLogger(std::string logName, TaskFactory::Ptr src, std::string header);
    ~TaskLogger();

protected:

    TaskFactory::Ptr taskSource;
    std::ofstream logFile;
    
    // Produces wrapped tasks from source factory
    Task::Ptr ProduceObject();

    // Logs task results on completion
    void TaskEndCallback(Task* task);
    
};

#endif