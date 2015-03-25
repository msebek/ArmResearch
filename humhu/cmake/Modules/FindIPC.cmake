# CMake find module for IPC

find_path( IPC_INCLUDE_DIR ipc-3.9.1/include/ipc.h ipc/ipc.h include/ipc.h
          HINTS ~/Software/ ~/Code/ ~/ /usr/lib/ /usr/include/ ~/Software/ipc-3.9.1/ )

find_library( IPC_LIBRARY NAMES ipc libipc
             HINTS "${IPC_INCLUDE_DIR}/ipc-3.9.1/lib/Linux-3.2" "${IPC_INCLUDE_DIR}/ipc-3.9.1/lib/Linux-3.5")
             
set( IPC_LIBRARIES ${IPC_LIBRARY} )
set( IPC_INCLUDE_DIRS "${IPC_INCLUDE_DIR}/ipc-3.9.1/include/" "${IPC_INCLUDE_DIR}/ipc/" )

message(STATUS ${IPC_INCLUDE_DIR})
message(STATUS ${IPC_LIBRARY})

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( libipc DEFAULT_MSG
                                  IPC_LIBRARY IPC_INCLUDE_DIR )
mark_as_advanced( IPC_INCLUDE_DIR IPC_LIBRARY )
