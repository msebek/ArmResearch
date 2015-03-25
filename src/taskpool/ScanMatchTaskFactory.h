#ifndef _SCANMATCH_TASK_FACTORY_H_
#define _SCANMATCH_TASK_FACTORY_H_

#include "Factory.h"
#include "ScanMatchTask.h"
#include "ScanMatcherFactory.h"
#include "LaserScan.h"

class ScanMatchTaskFactory : public Factory<ScanMatchTask::Ptr> {
public:

    typedef Factory<LaserScan::Ptr> ScanFactory;
    typedef ScanMatchTask::CloudMatcher CloudMatcher;
    
    ScanMatchTaskFactory(ScanFactory::Ptr scanSrc, ScanMatcherFactory::Ptr matcherSrc,
                         ScanMatchTask::Parameters &params);
    ~ScanMatchTaskFactory();

protected:

    ScanFactory::Ptr scanSource;
    ScanMatcherFactory::Ptr matcherSource;
    
    ScanMatchTask::Parameters baseParameters;
    LaserScan::Ptr prevScan;
    unsigned int prevScanID;
    
    ScanMatchTask::Ptr ProduceObject();
    
};

#endif