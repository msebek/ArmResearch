#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>

#include "CarmenLogLaserReader.h"
#include "ScanMatchTask.h"
#include "ScanMatchTaskFactory.h"
#include "TaskLogger.h"
#include "TaskMapper.h"
#include "TaskPool.h"
#include "ICPFactory.h"

#include <pcl/registration/icp.h>

#define THREAD_POOL_SIZE (6)
#define MAP_BUFFER_SIZE (6)

int main(int argc, char* argv[]) {

    if(argc < 3) {
        std::cout << "Please provide log paths." << std::endl;
        return -1;
    }
    
    std::string sourceFile(argv[1]);
    std::string resultsFile(argv[2]);

    std::cout << resultsFile << std::endl;

    // Create default task parameters
    CarmenLogLaserReader::Ptr reader( new CarmenLogLaserReader(sourceFile, "FLASER",
        -M_PI/2.0, M_PI/2.0 ) );

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> ICP;
    ICPFactory::Parameters icpParams;
    icpParams.maximumIterations = ICP.getMaximumIterations();
    icpParams.RANSACIterations = ICP.getRANSACIterations();
    icpParams.RANSACOutlierRejectionThreshold = ICP.getRANSACOutlierRejectionThreshold();
    icpParams.maxCorrespondenceDistance = ICP.getMaxCorrespondenceDistance();
    icpParams.transformationEpsilon = ICP.getTransformationEpsilon();
    icpParams.euclideanFitnessEpsilon = ICP.getEuclideanFitnessEpsilon();
    icpParams.useReciprocalCorrespondences = ICP.getUseReciprocalCorrespondences();
    ICPFactory::Ptr icpFactory( new ICPFactory(icpParams) );

    ScanMatchTask::Parameters parameters;
    parameters.noiseMean = ScanMatchTask::Vec::Zero();
    ScanMatchTask::Mat c = ScanMatchTask::Mat::Zero();
    c(0,0) = 0.1; c(1,1) = 0.1; c(2,2) = 0.05;
    parameters.noiseCovariance = c;
    parameters.rangeMean = 0.0;
    parameters.rangeVariance = 0.1;
    parameters.iterations = 1;
    
    ScanMatchTaskFactory::Ptr taskFactory(
        new ScanMatchTaskFactory(reader, icpFactory, parameters) );
    TaskPool::Ptr pool( new TaskPool(THREAD_POOL_SIZE) );

    std::stringstream ss;
    ss << "#Empirically determined covariances for " << sourceFile << std::endl;
    ScanMatchTask::PrintHeader(ss);
    TaskLogger::Ptr logger( new TaskLogger(resultsFile, taskFactory, ss.str()) );
    
    TaskMapper::Parameters mapParams;
    mapParams.taskSink = pool;
    mapParams.taskSource = logger;
    mapParams.bufferSize = MAP_BUFFER_SIZE;
    TaskMapper map(mapParams);

    map.Run();
    
}