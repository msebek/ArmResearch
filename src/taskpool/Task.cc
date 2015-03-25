#include "Task.h"

Task::Task(Parameters &params) : parameters(params) {}

Task::~Task() {}

void Task::Run() {
    if(parameters.onStart) { parameters.onStart(this); }
    ExecuteTask();
    if(parameters.onEnd) { parameters.onEnd(this); }
}

void Task::PrintHeader(std::ostream& os) const {}

void Task::Print(std::ostream& os) const {}

std::ostream& operator<<(std::ostream& os, const Task& task) {
    task.Print(os);
    return os;
};