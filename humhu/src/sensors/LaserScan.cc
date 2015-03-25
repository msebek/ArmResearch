#include "LaserScan.h"

LaserScan::LaserScan(Ranges& r, Angles& t) :
    ranges(r), angles(t) {}

LaserScan::~LaserScan() {}

LaserScan::Cloud::Ptr LaserScan::ToCloud() {

    Cloud::Ptr cloud( new Cloud );
    ToCloud( *cloud );
    return cloud;
    
}

void LaserScan::ToCloud(Cloud& dst) {

    dst.clear();
    for(unsigned int i = 0; i < ranges.size(); i++) {
        
        double r = ranges[i];
        double a = angles[i];
        Point p(r*cos(a), r*sin(a), 0.0);
        dst.push_back(p);
    }
    
}