cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0048 NEW) # Set policy
project(object_slam VERSION 1.0.0)

message(STATUS "Eigen3_DIR=${Eigen3_DIR}")
message(STATUS "Pangolin_DIR=${Pangolin_DIR}")
message(STATUS "Pangolin_INCLUDE_DIRS=${Pangolin_INCLUDE_DIRS}")
message(STATUS "PYTHON_LIBRARIES=${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_INCLUDE_DIRS=${PYTHON_INCLUDE_DIRS}")
message(STATUS "PYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}")
message(STATUS "CONDA_LIBS=${CONDA_LIBS}")
message(STATUS "OpenCV_DIR=${OpenCV_DIR}")
message(STATUS "catkin_DIR=${catkin_DIR}")

find_package(OpenCV 3.4) # was 4.2?
if(NOT OpenCV_FOUND)
   MESSAGE(FATAL_ERROR "OpenCV not found.")
else()
   MESSAGE("OPENCV VERSION:")
   MESSAGE(${OpenCV_VERSION})
   MESSAGE("OpenCV_LIBS=${OpenCV_LIBS}")
endif()

find_package(catkin REQUIRED COMPONENTS
             covins_comm
             eigen_catkin
             #opencv3_catkin
			 #pcl_ros
)

set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../../devel/include/eigen3)
find_package(Eigen3 3.3.4 EXACT REQUIRED)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated-declarations -Wno-attributes -Wno-deprecated-copy -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-attributes -Wno-deprecated-copy -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")

# Check C++14 or C++0x support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
  message(STATUS "Pangolin_INCLUDE_DIRS:${Pangolin_INCLUDE_DIRS}")
  message(STATUS "Pangolin_LIBRARIES:${Pangolin_LIBRARIES}")
else()
  message(STATUS "Pangolin NOT FOUND")
endif()

#find_package(realsense2)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
${covins_comm_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include/comm
${PROJECT_SOURCE_DIR}/Thirdparty/
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
#${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/pybind11/include
)

# You need to set to your own Python paths
add_subdirectory(${PROJECT_SOURCE_DIR}/pybind11)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/System_util.cc
src/Tracking.cc
src/Tracking_util.cc		
src/LocalMapping.cc
src/LocalMapping_util.cc
#src/ObjectMapping.cc
src/LoopClosing.cc
src/LoopClosing_util.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/KeyFrame.cc
src/KeyFrame_util.cc
src/MapPoint.cc
src/MapPoint_util.cc
src/MapObject.cc
src/Atlas.cc
src/Atlas_util.cc
src/Map.cc
src/Map_util.cc
src/MapDrawer.cc
src/ObjectDrawer.cc
src/ObjectDetection.cc
src/ObjectRenderer.cc
src/Optimizer.cc
src/Optimizer_util.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/TwoViewReconstruction.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
#include/ObjectMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/MapObject.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/ObjectDrawer.h
include/ObjectDetection.h
include/ObjectRenderer.h
include/ObjectPoseGraph.h
include/Renderer.hpp
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Initializer.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/TwoViewReconstruction.h
include/Config.h
# Comm
include/comm/communicator.hpp
src/comm/communicator.cpp
# tcp-Interface - mission systems data
include/tcp_interface.hpp
)

add_subdirectory(Thirdparty/g2o)

#add_definitions(-DBLAS_SYMBOL_SUFFIX=64_) # to solver openblas consistency with numpy
# to check which openblas numpy is using:
# 	import numpy
# 	numpy.show_config()
# to check which openblas c++ application is using:
# 	ldd dsp_slam | grep -i openblas
# OpenBlas can also be compiled from source using:
# 	make BINARY=64 INTERFACE64=1 CC=gcc FC=gfortran HOSTCC=gcc TARGET=HASWELL 
# note: INTERFACE64=1: Enables the ILP64 interface, which provides the 64_ suffix and uses 64-bit integers.
# then installed system wise using:
# 	sudo make install PREFIX=/usr/local

target_link_libraries(${PROJECT_NAME}
${CONDA_LIBS}
pybind11::embed
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${covins_comm_LIBRARIES}
-lcrypto
-lncurses   # for Jetson
-lncursesw   # for Jetson
-lffi   # for Jetson
snark_imaging   # Mission systems
snark_ouster  # Mission systems
comma_csv  # Mission systems
comma_string  # Mission systems
comma_xpath  # Mission systems
-lboost_serialization
-lboost_system # more stable when linked last
-lpthread # more stable when linked last
)

### Build examples

# Monocular examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
#add_executable(mono_tum Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})
#add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})
#add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})
#add_executable(mono_tum_vi Examples/Monocular/mono_tum_vi.cc)
#target_link_libraries(mono_tum_vi ${PROJECT_NAME})

# Monocular-Inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)
#add_executable(mono_inertial_euroc Examples/Monocular-Inertial/mono_inertial_euroc.cc)
#target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})
#add_executable(mono_inertial_tum_vi Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
#target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

# Stereo examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
#add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})
#add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

# Stereo-Inertial examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
#add_executable(stereo_inertial_euroc
#Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
#target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

### Build main demos

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
# DSP examples

add_executable(mono_inertial_kitti mono_inertial_kitti.cc)
target_link_libraries(mono_inertial_kitti ${PROJECT_NAME})

add_executable(stereo_inertial_kitti stereo_inertial_kitti.cc)
target_link_libraries(stereo_inertial_kitti ${PROJECT_NAME})

add_executable(stereo_kitti stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(dsp_slam dsp_slam.cc)
target_link_libraries(dsp_slam ${PROJECT_NAME})

add_executable(mono_snark mono_snark.cc)
target_link_libraries(mono_snark ${PROJECT_NAME})

#add_executable(reconstruct_frame reconstruct_frame.cpp)
#target_link_libraries(reconstruct_frame ${PROJECT_NAME})
#add_executable(image_from_bin image_from_bin.cc)
#target_link_libraries(image_from_bin ${PROJECT_NAME})
