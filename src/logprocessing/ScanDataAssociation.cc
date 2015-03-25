#include "ScanDataAssociation.h"

#define MINPEAK (0.34)
#define MINEPAKDIST (0.001)

ScanDataAssociation::ScanDataAssociation(LaserFactory::Ptr src, unsigned int minIn) :
    scanSource(src), minInliers(minIn) {

    unsigned int scale = 5, dmst = 2, window = 3, detectorType = 0, descriptorType = 0, distanceType = 2, strategy = 0;
    double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, acceptanceSigma = 0.1, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
    bool useMaxRange = false;
        
    peakFinder.reset( new SimpleMinMaxPeakFinder(minPeak, minPeakDistance) );

    CurvatureDetector* cdet = new CurvatureDetector(peakFinder.get(), scale, baseSigma, sigmaStep, dmst);
    cdet->setUseMaxRange(useMaxRange);
    featureDetector.reset( cdet );
    
    featureDistance.reset( new SymmetricChi2Distance<double>() );

    BetaGridGenerator* bgrid = new BetaGridGenerator(0.02, 0.5, 4, 12);
    bgrid->setDistanceFunction(featureDistance.get());
    descriptorGenerator.reset(bgrid);
    
    ransacMatcher.reset( new RansacFeatureSetMatcher(acceptanceSigma*acceptanceSigma*5.99,
        success, inlier, matchingThreshold, acceptanceSigma*acceptanceSigma*3.84, false) );
    
}

ScanDataAssociation::~ScanDataAssociation() {}

std::vector< ScanDataAssociation::CorrespondenceResult > ScanDataAssociation::GetResults() {
    return correspondenceResults;
}

void ScanDataAssociation::ProcessNextScan() {

    LaserScan::Ptr nextScan = scanSource->Produce();
    if(!nextScan) { return; }

    // Convert to FLIRT data format
    LaserReading nextReading(nextScan->angles, nextScan->ranges);

    // Get new scan features
    unsigned int nextInd = features.size() + 1;
    Features nextFeatures;
    featureDetector->detect(nextReading, nextFeatures);
    for(unsigned int i = 0; i < nextFeatures.size(); i++) {
        nextFeatures[i]->setDescriptor( descriptorGenerator->describe( *nextFeatures[i], nextReading ) );
    }

    // Compare new scan to all past scans
    OrientedPoint2D transform;
    for(size_t i = 0; i < features.size(); i++) {

        // Save correspondence when found
        if(!CheckScans(features[i], nextFeatures, transform)) { continue; }
        IndexCorrespondence iCorr(i, nextInd);
        CorrespondenceResult fCorr(iCorr, transform);
        correspondenceResults.push_back(fCorr);
        
    }

    // Add the current scan to history
    features.push_back(nextFeatures);
    
}

bool ScanDataAssociation::CheckScans(Features& features1, Features& features2,
                                     OrientedPoint2D& transform) {

    std::vector< FeatureCorrespondence > correspondences;    
    
    double error = ransacMatcher->matchSets(features1, features2, transform, correspondences);
    return correspondences.size() >= minInliers;
    
}