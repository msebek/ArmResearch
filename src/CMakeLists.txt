include_directories(${CovReg_SOURCE_DIR}/src)
include_directories(${CovReg_SOURCE_DIR}/src/logprocessing)
include_directories(${CovReg_SOURCE_DIR}/src/logreader)
include_directories(${CovReg_SOURCE_DIR}/src/sensors)
include_directories(${CovReg_SOURCE_DIR}/src/taskpool)
include_directories(${CovReg_SOURCE_DIR}/src/ui)

link_directories(${CovReg_BINARY_DIR}/src)

add_subdirectory(logprocessing)
add_subdirectory(logreader)
add_subdirectory(sensors)
add_subdirectory(taskpool)
add_subdirectory(ui)

#Set the list of source files
add_library(CovReg STATIC ${SOURCES} ${HEADERS})

target_link_libraries(CovReg ${GSL_LIBRARIES} ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}
    ${flirt_LIBRARIES})