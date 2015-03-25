#ifndef _WORKER_CLASS_
#define _WORKER_CLASS_

#include  <boost/thread.hpp>
#include "ThreadsafeClass.h"

// TODO: Implement detatch()

/*! \class WorkerClass WorkerClass.h
 * \brief serves as a base class for classes with background
 * execution functionality. */
class WorkerClass : public virtual ThreadsafeClass {
public:

    typedef boost::thread Thread;
    
    /*! \brief Initializes the threaded class, but does not start it. */
    WorkerClass();
    
    /*! \brief Halts and waits for the thread to return. */
    ~WorkerClass();
    
    /*! \brief Begins threaded operation. */
    virtual void Start();
    
    /*! \brief Requests threaded operation to halt. */
    virtual void Stop();
    
    /*! \brief Joins the thread. */
    virtual void Join();
    
    void SetRunFrequency(float freq);
    
protected:
    /*! \var Indicates if the worker is running. */
    bool threadRunning;

    /*! \var The worker thread. */
    Thread workerThread;

    /*! \var Desired time in between spins. */
    float runPeriod;
    
    // TODO: Switch to use interrupt() instead of threadStopRequested
    /*! \brief The function called by the worker thread. This calls Process()
     * and also manages halting and flag-setting. */
    virtual void Spin();

    /*! \brief The work that the threaded class needs to do in the background.
     * This should be overridden by the derived class. */
    virtual void Process() = 0;

};

#endif //_WORKER_CLASS_
