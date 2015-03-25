#include "WrapperTask.h"

WrapperTask::WrapperTask(Parameters &params) :
    Task(params), parameters(params) {}

WrapperTask::~WrapperTask() {}

void WrapperTask::ExecuteTask() {
    if(parameters.wrappedTask) {
        parameters.wrappedTask->Run();
    }
}

void WrapperTask::Print(std::ostream& os) const {
    //os << *parameters.wrappedTask;
    parameters.wrappedTask->Print(os);
}

std::ostream& operator<<(std::ostream& os, const WrapperTask& task) {
    task.Print(os);
    return os;
}
