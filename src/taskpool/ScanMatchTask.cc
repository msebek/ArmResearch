#include "ScanMatchTask.h"
#include "MVGaussianDistribution.h"
#include <Eigen/Geometry>

#include <pcl/visualization/pcl_visualizer.h>

ScanMatchTask::ScanMatchTask(Parameters &params, bool vOn) :
    Task(params), parameters(params), noiser( params.noiseMean, params.noiseCovariance,
        params.rangeMean, params.rangeVariance,
        params.fixedScan->angles[1] - params.fixedScan->angles[0] ),
    visOn(vOn) {
    matchingResults.resize(parameters.iterations);
    matchingResults.clear();
};

ScanMatchTask::~ScanMatchTask() {};

void ScanMatchTask::ExecuteTask() {
    RunTrials();
    CalculateStatistics();
    std::cout << "ScanMatchTask: Finished execution of scan pair (" <<
    parameters.fixedScanNumber << ", " << parameters.matchScanNumber <<
    ") with mean: "<< std::endl;
    std::cout << dataMean << std::endl;
    std::cout << "covariance:" << std::endl;
    std::cout << dataCov << std::endl;
}

void ScanMatchTask::RunTrials() {

    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Color;
    
    // Initialize scan matcher
    Cloud::Ptr perturbedCloud(new Cloud());
    Cloud::Ptr matchedCloud(new Cloud());
    Cloud::Ptr fixedCloud = parameters.fixedScan->ToCloud();
    parameters.scanMatcher->setInputTarget(fixedCloud);
    
    // Initialize viewer
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if(visOn) {
        viewer.reset(new pcl::visualization::PCLVisualizer("TaskViewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);

        Color green(fixedCloud, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ>(fixedCloud, green, "Fixed Cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Fixed Cloud");

        Color blue(fixedCloud, 0, 0, 255);
        viewer->addPointCloud<pcl::PointXYZ>(fixedCloud, blue, "Perturbed Cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Perturbed Cloud");

        Color red(fixedCloud, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(fixedCloud, red, "Matched Cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Matched Cloud");

        viewer->initCameraParameters();
        viewer->resetCamera();
    }

    // These must be predeclared due to a quirk in Eigen with implicit conversions
    Eigen::Transform<double, 3, Eigen::Affine> rot, trans;
    
    for(unsigned int i = 0; i < parameters.iterations; i++) {

        ScanNoiser::Noise noise = noiser.GenerateNoise( parameters.matchScan );
        perturbedCloud = noiser.PerturbScan( parameters.matchScan, noise );
        
        // Match using matcher
        parameters.scanMatcher->setInputSource(perturbedCloud);
        parameters.scanMatcher->align(*matchedCloud);
        Eigen::Transform<double, 3, Eigen::Affine> matchResult =
            Eigen::Transform<double, 3, Eigen::Affine>(parameters.scanMatcher->getFinalTransformation());
        if(!parameters.scanMatcher->hasConverged()) {
            std::cout << "ScanMatchTask: Registration failed to converge!" << std::endl;
            // TODO Do something?
        }

        rot = Eigen::AngleAxis<double>(noise.transNoise(2), Eigen::Vector3d::UnitZ());
        trans = Eigen::Translation<double,3>(noise.transNoise(0), noise.transNoise(1), 0.0);
        Eigen::Transform<double, 3, Eigen::Affine> perturbation = rot*trans;
        Eigen::Transform<double, 3, Eigen::Affine> residual = matchResult*perturbation;
        
        Eigen::Rotation2D<double> resultRot(0.0);
        resultRot.fromRotationMatrix(residual.matrix().block<2,2>(0,0));
        Eigen::Translation<double, 2> resultTrans(residual.matrix().block<2,1>(0,3));
        Vec result;
        result << resultTrans.x(), resultTrans.y(), resultRot.angle();
        //std::cout << result << std::endl;
        matchingResults.push_back(result);
        
        // Visualize results
        if(visOn) {
            Color blue(matchedCloud, 0, 0, 255);
            viewer->updatePointCloud<pcl::PointXYZ>(perturbedCloud, blue, "Perturbed Cloud");
            Color red(matchedCloud, 255, 0, 0);
            viewer->updatePointCloud<pcl::PointXYZ>(matchedCloud, red, "Matched Cloud");

            viewer->spinOnce();
        }
        
    }
    
}

void ScanMatchTask::CalculateStatistics() {

    Vec accumulator = Vec::Zero();
    for(unsigned int i = 0; i < parameters.iterations; i++) {
        accumulator += matchingResults[i];
    }
    dataMean = accumulator/parameters.iterations;
    
    Eigen::Matrix<double, 3, Eigen::Dynamic> data =
        Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, parameters.iterations);
    for(unsigned int i = 0; i < parameters.iterations; i++) {
        data.col(i) = matchingResults[i] - dataMean;;
    }
    dataCov = data*data.transpose()/(parameters.iterations - 1);
    
}

ScanMatchTask::Vec ScanMatchTask::GetDataMean() {
    return dataMean;
}

ScanMatchTask::Mat ScanMatchTask::GetDataCovariance() {
    return dataCov;
}

void ScanMatchTask::PrintHeader(std::ostream& os) {

    os << "FixedScanID MatchScanID numIterations transformCov_xx transformCov_yy "
       << "transformCov_tt rangeVar errMean_x errMean_y errMean_t errCov_xx "
       << "errCov_yy errCov_tt errCov_xy errCov_xt errCov_yt ";
    
}

// TODO Figure out how to print scan matcher parameters
void ScanMatchTask::Print(std::ostream& os) const {
    Task::Print(os);
    
    os << parameters.fixedScanNumber << " " << parameters.matchScanNumber <<
        " " << parameters.iterations << " " << parameters.noiseCovariance(0,0) << " "
        << parameters.noiseCovariance(1,1) << " " << parameters.noiseCovariance(2,2)
        << " " << parameters.rangeVariance << " " << dataMean(0) << " " << dataMean(1) <<
        " " << dataMean(2) << " " << dataCov(0,0) << " " << dataCov(1,1)
        << " " << dataCov(2,2) << " " << dataCov(0,1) << " " <<
        dataCov(0,2) << " " << dataCov(1,2);
}

std::ostream& operator<<(std::ostream& os, const ScanMatchTask& task) {
    task.Print(os);
    return os;
}
