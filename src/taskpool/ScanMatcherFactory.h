#ifndef _SCANMATCHER_FACTORY_H_
#define _SCANMATCHER_FACTORY_H_

#include "Factory.h"
#include <pcl/registration/registration.h>

class ScanMatcherFactory
    : public Factory< pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, double>::Ptr > {
public:

    typedef pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, double> ScanMatcher;
    
    struct Parameters {
        // TransformationEstimator
        // CorrespondenceEstimator
        // SearchMethod
        unsigned int maximumIterations;
        unsigned int RANSACIterations;
        double RANSACOutlierRejectionThreshold;
        double maxCorrespondenceDistance;
        double transformationEpsilon;
        double euclideanFitnessEpsilon;
    };
    
    ScanMatcherFactory(Parameters &params);
    ~ScanMatcherFactory();

protected:

    Parameters parameters;
    
    ScanMatcherFactory::ScanMatcher::Ptr ProduceObject();
    
    virtual ScanMatcher::Ptr ProduceScanMatcher() = 0;
    
};

#endif