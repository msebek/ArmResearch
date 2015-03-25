#include "CarmenLogLaserReader.h"
#include "UIPlayback.h"

#include <pcl/registration/icp.h>

#define THREAD_POOL_SIZE (6)
#define MAP_BUFFER_SIZE (6)

int main(int argc, char* argv[]) {

    if(argc < 2) {
        std::cout << "Please provide log path." << std::endl;
        return -1;
    }

    std::string sourceFile(argv[1]);

    // Create default task parameters
    CarmenLogLaserReader::Ptr reader( new CarmenLogLaserReader(sourceFile, "FLASER",
        -M_PI/2.0, M_PI/2.0 ) );

    UIPlayback::Ptr ui(new UIPlayback(argv[1], reader, 10.0));
    ui->Start();
    ui->Join();
    
}