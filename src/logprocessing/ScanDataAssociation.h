#ifndef _SCAN_DATA_ASSOCIATION_H_
#define _SCAN_DATA_ASSOCIATION_H_

//#include <flirtlib/feature/Detector.h>
#include <flirtlib/feature/CurvatureDetector.h>
#include <flirtlib/feature/BetaGrid.h>
#include <flirtlib/feature/RansacFeatureSetMatcher.h>
#include <flirtlib/utils/SimpleMinMaxPeakFinder.h>
#include <flirtlib/utils/HistogramDistances.h>

#include <memory>

#include "LaserScan.h"
#include "Factory.h"

class ScanDataAssociation {
public:

    typedef Factory< LaserScan::Ptr > LaserFactory;
    typedef std::vector<InterestPoint*> Features;
    typedef std::pair<InterestPoint*, InterestPoint*> FeatureCorrespondence;
    typedef std::pair<unsigned int, unsigned int> IndexCorrespondence;
    typedef std::pair<IndexCorrespondence, OrientedPoint2D> CorrespondenceResult;
    
    ScanDataAssociation( LaserFactory::Ptr src, unsigned int minInliers );
    ~ScanDataAssociation();

    // Pull a scan from the source and check correspondences
    void ProcessNextScan();
    std::vector< CorrespondenceResult > GetResults();
    
protected:
    
    unsigned int minInliers; // Min # of inliers to count as match
    
    LaserFactory::Ptr scanSource;
    std::vector< LaserScan::Ptr > scans;
    std::vector< std::vector<InterestPoint*> > features;
    std::vector< OrientedPoint2D > poses;
    std::vector< CorrespondenceResult > correspondenceResults;
    
    // FLIRT objects
    std::shared_ptr< SimpleMinMaxPeakFinder > peakFinder;
    std::shared_ptr< Detector > featureDetector;
    std::shared_ptr< HistogramDistance<double> > featureDistance;
    std::shared_ptr< DescriptorGenerator > descriptorGenerator;
    std::shared_ptr< RansacFeatureSetMatcher > ransacMatcher;
    
    // Check if two scans correspond
    bool CheckScans(Features& features1, Features& features2, OrientedPoint2D& transform);
    
};

#endif