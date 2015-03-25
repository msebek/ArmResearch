#ifndef _ICP_FACTORY_H_
#define _ICP_FACTORY_H_

#include "ScanMatcherFactory.h"
#include <pcl/registration/icp.h>
#include <memory>

// TODO: Add support for parameter sweeping
class ICPFactory : public ScanMatcherFactory {
public:

    typedef ScanMatcherFactory::ScanMatcher ScanMatcher;
    typedef pcl::PointXYZ Point;
    typedef pcl::IterativeClosestPoint<Point, Point, double> ICP;
    typedef std::shared_ptr<ICPFactory> Ptr;
    
    struct Parameters : public ScanMatcherFactory::Parameters {
        bool useReciprocalCorrespondences;
    };

    ICPFactory(Parameters &params);
    ~ICPFactory();

protected:

    Parameters parameters;

    ScanMatcher::Ptr ProduceScanMatcher();
    
};

#endif