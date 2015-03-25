#include "CarmenLogLaserReader.h"

CarmenLogLaserReader::CarmenLogLaserReader(std::string filename, std::string tag,
        double sAng, double eAng) :
    CarmenLogReader(filename), lineTag(tag), startAngle(sAng), endAngle(eAng) {}

CarmenLogLaserReader::~CarmenLogLaserReader() {}

LaserScan::Ptr CarmenLogLaserReader::ProduceObject() {

    CarmenLogLaserLine logLine = GetNextLine(lineTag);
    if(!logLine.valid) { return nullptr; }

    // Have to fake the angles since logs don't contain any information
    double angle = startAngle;
    double resolution = (endAngle - startAngle)/logLine.num_readings;
    std::vector<double> angles;
    angles.resize(logLine.num_readings);
    for(unsigned int i = 0; i < logLine.num_readings; i++) {
        angles[i] = angle;
        angle += resolution;
    }
    
    LaserScan::Ptr scan( new LaserScan(logLine.range_readings, angles) );
    return scan;
    
}

void CarmenLogLaserReader::ParseLineMiddle(TokenizerIterator&iter, CarmenLogLaserLine &logLine) {

    logLine.num_readings = boost::lexical_cast<unsigned int>(*iter); iter++;
    logLine.range_readings.clear();
    logLine.range_readings.resize(logLine.num_readings);
    for(unsigned int i = 0; i < logLine.num_readings; i++) {
        logLine.range_readings[i] = boost::lexical_cast<double>(*iter); iter++;
    }
    logLine.x = boost::lexical_cast<float>(*iter); iter++;
    logLine.y = boost::lexical_cast<float>(*iter); iter++;
    logLine.theta = boost::lexical_cast<float>(*iter); iter++;
    logLine.odom_x = boost::lexical_cast<float>(*iter); iter++;
    logLine.odom_y = boost::lexical_cast<float>(*iter); iter++;
    logLine.odom_theta = boost::lexical_cast<float>(*iter); iter++;
    
}