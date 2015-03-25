#ifndef _SCANMATCH_TASK_H_
#define _SCANMATCH_TASK_H_

#include "Task.h"
#include "LaserScan.h"
#include "ScanNoiser.h"
#include <pcl/registration/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/StdVector> // Needed to support fixed-size matrices in vector
#include <iostream>

/*! \class ScanMatchTask ScanMatchTask.h
 * \brief represents a brute-force scan matching task between two scans with
 * certain perturbation parameters. */
class ScanMatchTask : public Task {
public:

    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> Cloud;
    typedef pcl::Registration<Point, Point, double> CloudMatcher;
    
    typedef Eigen::Vector3d Vec;
    typedef Eigen::Matrix3d Mat;
    
    typedef std::vector<Vec, Eigen::aligned_allocator<Vec> > VecArray;
    
    struct Parameters : public Task::Parameters {
        unsigned int fixedScanNumber;
        unsigned int matchScanNumber;
        LaserScan::Ptr fixedScan;
        LaserScan::Ptr matchScan;
        ScanMatchTask::CloudMatcher::Ptr scanMatcher;
        unsigned int iterations;
        ScanMatchTask::Vec noiseMean;
        ScanMatchTask::Mat noiseCovariance;
        double rangeMean;
        double rangeVariance;
    };
    
    ScanMatchTask(ScanMatchTask::Parameters &params, bool vOn = false);
    ~ScanMatchTask();

    ScanMatchTask::Vec GetDataMean();
    ScanMatchTask::Mat GetDataCovariance();

    virtual void Print(std::ostream& os) const;
    static void PrintHeader(std::ostream& os);
    
protected:
    
    Parameters parameters;
    ScanNoiser noiser;
    VecArray matchingResults;
    bool visOn;

    Vec dataMean;
    Mat dataCov;

    void ExecuteTask();
    void RunTrials();
    void CalculateStatistics();
    
    friend std::ostream& operator<<(std::ostream& os, const ScanMatchTask& task);
    
};

std::ostream& operator<<(std::ostream& os, const ScanMatchTask& task);

#endif