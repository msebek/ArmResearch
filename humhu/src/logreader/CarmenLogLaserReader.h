#ifndef _CARMENLASERREADER_H_
#define _CARMENLASERREADER_H_

#include "CarmenLogReader.h"
#include "Factory.h"
#include "LaserScan.h"
#include <memory>

struct CarmenLogLaserLine : public CarmenLogLine {
    unsigned int num_readings;
    std::vector<double> range_readings;
    float x;
    float y;
    float theta;
    float odom_x;
    float odom_y;
    float odom_theta;
};

class CarmenLogLaserReader : public CarmenLogReader<CarmenLogLaserLine>,
                             public Factory< LaserScan::Ptr > {
public:

    typedef std::shared_ptr<CarmenLogLaserReader> Ptr;
    
    CarmenLogLaserReader(std::string filename, std::string tag, double startAngle,
                         double endAngle);
    ~CarmenLogLaserReader();
    
protected:

    const std::string lineTag;
    const double startAngle;
    const double endAngle;

    LaserScan::Ptr ProduceObject();
    
    void ParseLineMiddle(TokenizerIterator &iter, CarmenLogLaserLine &logLine);
    
};

#endif