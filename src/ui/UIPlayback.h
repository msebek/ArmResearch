#ifndef _UI_PLAYBACK_H_
#define _UI_PLAYBACK_H_

#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <memory>

#include "WorkerClass.h"
#include "Factory.h"
#include "LaserScan.h"

class UIPlayback : public WorkerClass {
public:

    typedef std::shared_ptr<UIPlayback> Ptr;
    
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> Cloud;
    typedef pcl::visualization::PointCloudColorHandlerCustom<Point> CloudColorHandler;
    typedef Factory<LaserScan::Ptr> ScanFactory;
    typedef std::shared_ptr<pcl::visualization::PCLVisualizer> VisualizerPtr;
    
    UIPlayback(std::string n, ScanFactory::Ptr scanSrc, float framerate);
    ~UIPlayback();

protected:

    int scanCounter;
    ScanFactory::Ptr scanSource;
    VisualizerPtr display;
    bool playing;

    std::pair<float, float> GetWindowSize();
    void UpdateScanCounter(int n);
    void UpdateStatusText(std::string t);
    void UpdateCloud(Cloud::Ptr cloud);

    void KeyboardCallback(const pcl::visualization::KeyboardEvent &event, void *ui);
    virtual void Process();
    
};

#endif