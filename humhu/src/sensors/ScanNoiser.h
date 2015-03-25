#ifndef _SCAN_NOISER_H_
#define _SCAN_NOISER_H_

#include "LaserScan.h"
#include <Eigen/Core>
#include "MVGaussianDistribution.h"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

// TODO Make noise sources modular
// TODO Implement Noiser interface
class ScanNoiser {
public:

    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> Cloud;

    struct Noise {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::vector<double> rangeNoise;
        std::vector<double> angleNoise;
        Eigen::Vector3d transNoise;
    };
    
    ScanNoiser(Eigen::Vector3d& transMean, Eigen::Matrix3d& transCov,
        double rMean, double rVar, double scanResolution);
    ~ScanNoiser();

    ScanNoiser::Noise GenerateNoise(LaserScan::Ptr source);
    
    // Returns a transformed and range-angle jittered scan
    Cloud::Ptr PerturbScan(LaserScan::Ptr source, Noise &noise);
    
protected:

    // For now transform will be multivariate Gaussian, range will be Gaussian,
    // and angle will be uniform

    boost::mt19937 randEngine;
    MVGaussianDistribution<3> transNoiseSrc;

    boost::normal_distribution<double> rangeNoiseDist;
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > rangeNoiseSrc;

    boost::uniform_real<double> angleNoiseDist;
    boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > angleNoiseSrc;
    
};

#endif