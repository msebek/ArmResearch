#ifndef _THREADSAFE_CLASS
#define _THREADSAFE_CLASS

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

/*! \class ThreadsafeClass ThreadsafeClass.h
 * \brief serves as a base class for thread-safe classes. It provides a
 * mutex and external locking checks. */
class ThreadsafeClass {

protected:

    typedef boost::shared_mutex Mutex;
    typedef boost::unique_lock<Mutex> WriteLock;
    typedef boost::shared_lock<Mutex> ReadLock;
    
    /*! \var Access protection to this object. */
    mutable Mutex mutex;
    
    /*! \brief Check to verify that external locking is done correctly. */
    void AssertExternalLock(WriteLock &lock) const;
    void AssertExternalLock(ReadLock &lock) const;
    
public:
    
    ThreadsafeClass();
    ~ThreadsafeClass();
    
};

#endif //_THREADSAFE_CLASS
