#ifndef _WRAPPER_TASK_H_
#define _WRAPPER_TASK_H_

#include "Task.h"

class WrapperTask : public Task {
public:

    typedef Task::Callback Callback;

    struct Parameters : public Task::Parameters {
        Task::Ptr wrappedTask;
    };
    
    WrapperTask(Parameters &params);
    ~WrapperTask();

    virtual void Print(std::ostream& os) const;
    
protected:

    Parameters parameters;
    
    void ExecuteTask();
    
    friend std::ostream& operator<<(std::ostream& os, const WrapperTask& task);
    
};

std::ostream& operator<<(std::ostream& os, const WrapperTask& task);

#endif