include_directories(${CovReg_SOURCE_DIR}/src/)
include_directories(${CovReg_SOURCE_DIR}/src/logprocessing)
include_directories(${CovReg_SOURCE_DIR}/src/logreader)
include_directories(${CovReg_SOURCE_DIR}/src/sensors)
include_directories(${CovReg_SOURCE_DIR}/src/taskpool)
include_directories(${CovReg_SOURCE_DIR}/src/ui)

link_directories(${CovReg_BINARY_DIR}/src/)

add_executable(ProcessLog ProcessLog.cc)
target_link_libraries(ProcessLog CovReg)

add_executable(PlayLog PlayLog.cc)
target_link_libraries(PlayLog CovReg)

add_executable(FindCorrespondences FindCorrespondences.cc)
target_link_libraries(FindCorrespondences CovReg)