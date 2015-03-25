#include "ScanNoiser.h"

ScanNoiser::ScanNoiser(Eigen::Vector3d& transMean, Eigen::Matrix3d& transCov,
                       double rMean, double rVar, double angleRes) :
    randEngine(), transNoiseSrc(transMean, transCov, randEngine),
    rangeNoiseDist(rMean, rVar), rangeNoiseSrc(randEngine, rangeNoiseDist),
    angleNoiseDist(-angleRes/2.0, angleRes/2.0), angleNoiseSrc(randEngine, angleNoiseDist) {}

ScanNoiser::~ScanNoiser() {}

ScanNoiser::Noise ScanNoiser::GenerateNoise(LaserScan::Ptr source) {

    unsigned int numPoints = source->ranges.size();
    Noise noise;
    noise.rangeNoise.resize(numPoints);
    noise.angleNoise.resize(numPoints);

    for(unsigned int i = 0; i < numPoints; i++) {
        noise.rangeNoise[i] = rangeNoiseSrc();
        noise.angleNoise[i] = angleNoiseSrc();
    }
    noise.transNoise = transNoiseSrc.Sample();
    return noise;
    
}

ScanNoiser::Cloud::Ptr ScanNoiser::PerturbScan(LaserScan::Ptr source, Noise &noise) {

    unsigned int numPoints = source->ranges.size();
    std::vector<double> noisedRanges;
    std::vector<double> noisedAngles;
    noisedRanges.resize(numPoints);
    noisedAngles.resize(numPoints);
    
    for(unsigned int i = 0; i < source->ranges.size(); i++) {
        noisedRanges[i] = source->ranges[i] + noise.rangeNoise[i];
        noisedAngles[i] = source->angles[i] + noise.angleNoise[i];
    }

    LaserScan scan(noisedRanges, noisedAngles);
    Cloud::Ptr cloud = scan.ToCloud();

    // These must be predeclared due to a quirk in Eigen with implicit conversions
    Eigen::Transform<double, 3, Eigen::Affine> rot, trans;

    rot = Eigen::AngleAxis<double>(noise.transNoise(2), Eigen::Vector3d::UnitZ());
    trans = Eigen::Translation<double,3>(noise.transNoise(0), noise.transNoise(1), 0.0);
    Eigen::Transform<double, 3, Eigen::Affine> perturbation = rot*trans;
    pcl::transformPointCloud<Point, double>(*cloud, *cloud, perturbation);

    return cloud;
    
}