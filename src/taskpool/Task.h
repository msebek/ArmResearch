#ifndef _TASK_H_
#define _TASK_H_

#include <boost/function.hpp>
#include <memory>
#include <iostream>

struct TaskParameters;

// Represents a task to be performed
class Task {
public:

    typedef std::shared_ptr<Task> Ptr;
    typedef boost::function<void(Task*)> Callback;

    struct Parameters {
        Task::Callback onStart;
        Task::Callback onEnd;
    };
    
    Task(Parameters &params);
    ~Task();

    // Runs the task until completion
    void Run();

    virtual void Print(std::ostream& os) const;
    void PrintHeader(std::ostream& os) const;
    
    friend std::ostream& operator<<(std::ostream& os, const Task& task);
    
protected:

    const Parameters parameters;

    // Body of task to be implemented
    virtual void ExecuteTask() = 0;
    
};

std::ostream& operator<<(std::ostream& os, const Task& task);

#endif