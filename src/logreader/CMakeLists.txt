include_directories(${CovReg_SOURCE_DIR}/logreader)

set(SOURCES
    ${SOURCES}
    logreader/CarmenLogLaserReader.cc
    PARENT_SCOPE
)

set(HEADERS
    ${HEADERS}
    logreader/CarmenLogReader.h
    logreader/CarmenLogLaserReader.h
    PARENT_SCOPE
)