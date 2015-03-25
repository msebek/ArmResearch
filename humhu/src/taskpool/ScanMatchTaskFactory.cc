#include "ScanMatchTaskFactory.h"

ScanMatchTaskFactory::ScanMatchTaskFactory(ScanFactory::Ptr scanSrc,
                                           ScanMatcherFactory::Ptr matcherSrc,
                                           ScanMatchTask::Parameters &params) :
    scanSource(scanSrc), matcherSource(matcherSrc), baseParameters(params),
    prevScanID(0) {
    prevScan = scanSource->Produce();
}

ScanMatchTaskFactory::~ScanMatchTaskFactory() {}

ScanMatchTask::Ptr ScanMatchTaskFactory::ProduceObject() {

    ScanMatchTask::Parameters params = baseParameters;
    params.scanMatcher = matcherSource->Produce();
    //params.fixedScan = prevScan;
    params.fixedScan = scanSource->Produce();
    params.matchScan = params.fixedScan;

    unsigned int nextScanID = prevScanID + 1;
    params.fixedScanNumber = nextScanID;
    params.matchScanNumber = nextScanID;
    
    if(!params.fixedScan || !params.matchScan || !params.scanMatcher) {
        return nullptr;
    }
    
    ScanMatchTask::Ptr task( new ScanMatchTask(params, false) );
    //prevScan = params.matchScan;
    prevScanID = nextScanID;
    return task;
    
};