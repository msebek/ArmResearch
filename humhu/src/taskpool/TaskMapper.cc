#include "TaskMapper.h"
#include "WrapperTask.h"
#include <boost/bind.hpp>

TaskMapper::TaskMapper(Parameters &params) :
    Task(params), parameters(params), bufferCount(params.bufferSize) {}

TaskMapper::~TaskMapper() {
    std::cout << "TaskMapper: Waiting for all tasks to complete." << std::endl;
    for(unsigned int i = 0; i < parameters.bufferSize; i++) {
        bufferCount.wait();
    }
    std::cout << "TaskMapper: Tasks completed." << std::endl;
}

void TaskMapper::ExecuteTask() {
    
    WrapperTask::Parameters wrapperParams;
    wrapperParams.onEnd = boost::bind( &TaskMapper::TaskEndCallback, this, _1);

    if(!parameters.taskSink || !parameters.taskSource || parameters.bufferSize == 0) {
        return;
    }
    
    wrapperParams.wrappedTask = parameters.taskSource->Produce();
    while(wrapperParams.wrappedTask) {
        bufferCount.wait();
        WrapperTask::Ptr task(new WrapperTask(wrapperParams) );
        parameters.taskSink->AddTask(task);
        wrapperParams.wrappedTask = parameters.taskSource->Produce();
    }
    
}

void TaskMapper::TaskEndCallback(Task* task) {
    bufferCount.post();
}