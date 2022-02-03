cmake_minimum_required(VERSION 3.1.0)
project(robofleet_base)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(${PROJECT_NAME} SHARED
    src/decode.hpp
    src/encode.hpp
    include/message_structs.h
    include/MessageSchedulerLib.hpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<INSTALL_INTERFACE:include>    
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT ${PROJECT_NAME}-export
  FILE
    RobofleetBaseTargets.cmake
  NAMESPACE
  RobofleetBase::
  DESTINATION
    lib/cmake/RobofleetBase
)