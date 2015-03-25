#ifndef _FACTORY_H_
#define _FACTORY_H_

#include "ThreadsafeClass.h"
#include <memory>

template <class T>
class Factory : protected ThreadsafeClass {
public:

    typedef std::shared_ptr< Factory<T> > Ptr;
    typedef T Output;

    Factory() :
        outputCount(0) {};
    ~Factory() {};

    T Produce() {
        WriteLock lock(mutex);
        outputCount++;
        return ProduceObject();
    }
    
    unsigned int GetCount() {
        ReadLock lock(mutex);
        return outputCount;
    };
    
protected:

    virtual T ProduceObject() = 0;

private:

    unsigned int outputCount;
    
};

#endif