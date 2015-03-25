#ifndef _LASER_SCAN_H_
#define _LASER_SCAN_H_

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <memory>

class LaserScan {
public:

    typedef std::shared_ptr<LaserScan> Ptr;
    
    typedef std::vector<double> Ranges;
    typedef std::vector<double> Angles;

    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> Cloud;
    
    LaserScan(Ranges &r, Angles &t);
    ~LaserScan();

    Cloud::Ptr ToCloud();
    void ToCloud(Cloud &dst);
    
    const Ranges ranges;
    const Angles angles;
    
};

#endif