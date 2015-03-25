#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>

#include "CarmenLogLaserReader.h"
#include "ScanMatchTask.h"
#include "ScanMatchTaskFactory.h"
#include "TaskLogger.h"
#include "TaskMapper.h"
#include "TaskPool.h"
#include "ICPFactory.h"
#include "ScanDataAssociation.h"
#include "UIPlayback.h"

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

    ScanDataAssociation dassc(reader, 5);
    for(unsigned int i = 0; i < 200; i++) {
        dassc.ProcessNextScan();
    }

    std::vector< ScanDataAssociation::CorrespondenceResult > results = dassc.GetResults();
    for(unsigned int i = 0; i < results.size(); i++) {
        std::pair<unsigned int, unsigned int> inds = results[i].first;
        std::cout << "Found correspondence between " << inds.first << " and " << inds.second << std::endl;
    }
    
}