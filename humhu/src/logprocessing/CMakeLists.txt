include_directories(${CovReg_SOURCE_DIR}/src/logprocessing)

set(SOURCES
    ${SOURCES}
    logprocessing/ScanDataAssociation.cc
    PARENT_SCOPE
)

set(HEADERS
    ${HEADERS}
    logprocessing/ScanDataAssociation.h
)