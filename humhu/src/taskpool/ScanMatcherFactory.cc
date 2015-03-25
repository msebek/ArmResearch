#include "ScanMatcherFactory.h"

ScanMatcherFactory::ScanMatcherFactory(Parameters &params) :
    parameters(params) {}

ScanMatcherFactory::~ScanMatcherFactory() {}

ScanMatcherFactory::ScanMatcher::Ptr ScanMatcherFactory::ProduceObject() {

    ScanMatcher::Ptr matcher = ProduceScanMatcher();
    //matcher->setTransformationEstimator(parameters.transformationEstimator);
    //matcher->setCorrespondenceEstimator(parameters.correspondenceEstimator);
    //matcher->setSearchMethod(parameters.searchMethod);
    matcher->setMaximumIterations(parameters.maximumIterations);
    matcher->setRANSACIterations(parameters.RANSACIterations);
    matcher->setMaxCorrespondenceDistance(parameters.maxCorrespondenceDistance);
    matcher->setRANSACOutlierRejectionThreshold(parameters.RANSACOutlierRejectionThreshold);
    matcher->setTransformationEpsilon(parameters.transformationEpsilon);
    matcher->setEuclideanFitnessEpsilon(parameters.euclideanFitnessEpsilon);
    return matcher;
    
}