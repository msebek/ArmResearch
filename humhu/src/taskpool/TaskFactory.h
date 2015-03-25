#ifndef _TASK_FACTORY_H_
#define _TASK_FACTORY_H_

#include "Task.h"
#include "ThreadsafeClass.h"
#include <memory>

template <class T>
class TaskFactory : private ThreadsafeClass {
public:

    typedef std::shared_ptr<T> OutputPtr;
    
    TaskFactory();
    ~TaskFactory();

    TaskFactory<T>::OutputPtr Produce() {
        WriteLock lock(mutex);
        outputCount++;
        return ProduceObject();
    };

    unsigned int GetCount() {
        ReadLock lock(mutex);
        return outputCount;
    };
    
protected:

    unsigned int outputCount;

    virtual TaskFactory<T>::OutputPtr ProduceObject() = 0;
    
};

#endif