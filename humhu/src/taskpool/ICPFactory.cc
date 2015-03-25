#include "ICPFactory.h"

ICPFactory::ICPFactory(Parameters &params) :
    ScanMatcherFactory(params), parameters(params) {}

ICPFactory::~ICPFactory() {}

ICPFactory::ScanMatcher::Ptr ICPFactory::ProduceScanMatcher() {

    ICP::Ptr matcher( new ICP() );
    matcher->setUseReciprocalCorrespondences(parameters.useReciprocalCorrespondences);
    return matcher;
    
}