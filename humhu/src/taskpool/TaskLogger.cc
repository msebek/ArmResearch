#include "TaskLogger.h"
#include "WrapperTask.h"

TaskLogger::TaskLogger(std::string logName, TaskFactory::Ptr src, std::string header) :
    logFile(logName), taskSource(src) {
    if(!logFile.is_open()) {
        std::cout << "TaskLogger: Failed to create log " << logName << std::endl;
    }
    logFile << header << std::endl;
}

TaskLogger::~TaskLogger() {}

Task::Ptr TaskLogger::ProduceObject() {

    WrapperTask::Parameters wrapperParams;
    wrapperParams.onEnd = boost::bind( &TaskLogger::TaskEndCallback, this, _1);
    wrapperParams.wrappedTask = taskSource->Produce();
    if(!wrapperParams.wrappedTask) {
        return nullptr;
    }
    
    Task::Ptr task( new WrapperTask(wrapperParams) );
    return task;
    
}

void TaskLogger::TaskEndCallback(Task* task) {
    WriteLock lock(mutex);
    task->Print(logFile);
    logFile << std::endl;
}