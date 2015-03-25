#include "UIPlayback.h"

UIPlayback::UIPlayback(std::string n, ScanFactory::Ptr scanSrc, float framerate) :
    scanCounter(0), scanSource(scanSrc), playing(true) {

    display = std::make_shared<pcl::visualization::PCLVisualizer>(n);
    display->setBackgroundColor(0,0,0);
    display->registerKeyboardCallback( &UIPlayback::KeyboardCallback, *this, display.get());
    SetRunFrequency(framerate);

    std::pair<float,float> windowSize = GetWindowSize();
    display->addText("Initializing", windowSize.first/2.0, windowSize.second - 30, 16, 1, 0, 0, "StatusText");
    display->addText("Scan -", windowSize.first/2.0, 30, 16, 0, 1, 0, "ScanCounter");

    Cloud::Ptr cloud(new Cloud);
    Point zero;
    cloud->push_back(zero);
    CloudColorHandler chandler(cloud, 1.0, 1.0, 1.0);
    display->addPointCloud<Point>(cloud, chandler, "Cloud");
    display->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");

    Point pX(0.5, 0.0, 0.0);
    Point pY(0.0, 0.5, 0.0);
    display->addLine(zero, pX, 1.0, 0.0, 0.0, "x_axis");
    display->addLine(zero, pY, 0.0, 1.0, 0.0, "y_axis");
    
}

UIPlayback::~UIPlayback() {}

std::pair<float,float> UIPlayback::GetWindowSize() {
    std::vector<pcl::visualization::Camera> cams;
    display->getCameras(cams);
    pcl::visualization::Camera cam = cams[0];
    std::pair<float,float> sizes(cam.window_size[0], cam.window_size[1]);
    return sizes;
}

void UIPlayback::UpdateScanCounter(int n) {
    std::pair<float,float> windowSize = GetWindowSize();
    std::stringstream ss;
    ss << "Scan: " << n;
    display->updateText(ss.str(), windowSize.first/2.0, 30, 16, 0, 1, 0, "ScanCounter");
}

void UIPlayback::UpdateStatusText(std::string t) {
    std::pair<float,float> windowSize = GetWindowSize();
    display->updateText(t, windowSize.first/2.0, windowSize.second - 30, 16, 1, 0, 0, "StatusText");
}

void UIPlayback::UpdateCloud(Cloud::Ptr cloud) {
    CloudColorHandler chandler(cloud, 255.0, 255.0, 255.0);
    display->updatePointCloud<Point>(cloud, chandler, "Cloud");
    display->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");
}

void UIPlayback::Process() {

    if(playing) {
        LaserScan::Ptr nextScan = scanSource->Produce();
        if(!nextScan) {
            UpdateStatusText("Reached end of log.");
            playing = false;
        } else {
            Cloud::Ptr nextCloud = nextScan->ToCloud();
            scanCounter++;
            UpdateCloud(nextCloud);
            UpdateScanCounter(scanCounter);
            UpdateStatusText("Playing");
        }
    } else {
        UpdateStatusText("Paused");
    }
    
    display->spinOnce();
}
    
void UIPlayback::KeyboardCallback(const pcl::visualization::KeyboardEvent &event,
                                        void *ui) {
    if(event.keyUp()) {
        if(event.getKeyCode() == 'f') {
            playing = !playing;
        }
    }

}