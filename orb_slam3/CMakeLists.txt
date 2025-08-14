cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0048 NEW)
project(object_slam VERSION 1.0.0 LANGUAGES C CXX)

# ---------------------------------------
# Build type
# ---------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ---------------------------------------
# Compiler flags
# ---------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use With Caution
# -Wno-deprecated-copy
# -Wno-conversion-null
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -Wno-deprecated-declarations \
    -Wno-attributes \
    -Wno-deprecated-copy \
    -Wno-conversion-null \
    -Wno-cpp \
    -O3")

# ---------------------------------------
# CMake modules path
# ---------------------------------------
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# ---------------------------------------
# Python3
# ---------------------------------------

find_package(Python3 3.8 EXACT REQUIRED COMPONENTS Interpreter Development)
message(STATUS "Python3_EXECUTABLE = ${Python3_EXECUTABLE}")
message(STATUS "Python3_INCLUDE_DIRS = ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3_LIBRARIES = ${Python3_LIBRARIES}")

# ---------------------------------------
# Pytorch
# ---------------------------------------

# Pytorch
find_package(Torch REQUIRED)
message(STATUS "Pytorch includes: ${TORCH_INCLUDE_DIRS}")
message(STATUS "Pytorch libs: ${TORCH_LIBRARIES}")

# critical definition for PyTorch Python bindings
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)
#add_definitions(-DTORCH_EXTENSION_NAME=${PROJECT_NAME})
#add_definitions(-DPYBIND11_COMPILER_TYPE=\"_gcc\")
#add_definitions(-DPYBIND11_STDLIB=\"_libstdcpp\")

# ---------------------------------------
# Third-party Eigen (force this version!)
# ---------------------------------------

# Tell CMake where your custom EigenConfig.cmake is
#set(Eigen3_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/eigen/install/share/eigen3/cmake")

# Now find it
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 version found: ${Eigen3_VERSION}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")

# ---------------------------------------
# Pangolin
# ---------------------------------------
find_package(Pangolin REQUIRED)
message(STATUS "Pangolin includes: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "Pangolin libs: ${Pangolin_LIBRARIES}")

# ---------------------------------------
# OpenCV
# ---------------------------------------
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# ---------------------------------------
# Include directories
# ---------------------------------------

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${PROJECT_SOURCE_DIR}/include/comm
  ${PROJECT_SOURCE_DIR}/Thirdparty/pybind11/include
  ${PROJECT_SOURCE_DIR}/Thirdparty
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/../covins_backend/include
  ${PROJECT_SOURCE_DIR}/../covins_comm/include
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
  #${PROJECT_SOURCE_DIR}/tensor_utils
)

# ---------------------------------------
# Debug messages
# ---------------------------------------
foreach(var
    Eigen3_DIR
    EIGEN3_INCLUDE_DIR
    Pangolin_DIR
    Pangolin_INCLUDE_DIRS
    Pangolin_LIBRARIES
    Python3_LIBRARIES
    Python3_INCLUDE_DIRS
    Python3_EXECUTABLE
    CONDA_LIBS
    OpenCV_DIR
    catkin_DIR
)
  message(STATUS "${var} = ${${var}}")
endforeach()

# ---------------------------------------
# Output path
# ---------------------------------------
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ---------------------------------------
# Thirdparty subdirs
# ---------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/pybind11)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/Sophus)
#add_subdirectory(tensor_utils)

# ---------------------------------------
# Source files
# ---------------------------------------
set(SOURCE_FILES
  src/System.cc
  src/System_util.cc
  src/Tracking.cc
  src/Tracking_util.cc
  src/LocalMapping.cc
  src/LocalMapping_util.cc
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
  #src/comm/communicator.cpp
)

# ---------------------------------------
# Headers
# ---------------------------------------
set(HEADER_FILES
  include/System.h
  include/Tracking.h
  include/LocalMapping.h
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
  #include/car.h
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
  #include/comm/communicator.hpp
  include/tcp_interface.hpp
)

# ---------------------------------------
# Library target
# ---------------------------------------
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

# ---------------------------------------
# Link libraries
# ---------------------------------------
target_link_libraries(${PROJECT_NAME}
  torch_python
  ${Python3_LIBRARIES}
  pybind11::embed
  pybind11::module
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  #tensor_utils_gpu
  -lcrypto -lncurses -lncursesw -lffi
  snark_imaging snark_ouster comma_csv comma_string comma_xpath
  -lboost_serialization -lboost_system -lpthread
)

# ---------------------------------------
# Optional: print target summary
# ---------------------------------------
message(STATUS "Object SLAM build complete: ${PROJECT_NAME}")


### Build main demos

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(test_pybind test_pybind.cpp)
target_link_libraries(test_pybind ${PROJECT_NAME})

add_executable(test_pangolin test_pangolin.cpp)
target_link_libraries(test_pangolin ${PROJECT_NAME})

add_executable(stereo_kitti stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_inertial_kitti stereo_inertial_kitti.cc)
target_link_libraries(stereo_inertial_kitti ${PROJECT_NAME})

#add_executable(mono_inertial_kitti mono_inertial_kitti.cc)
#target_link_libraries(mono_inertial_kitti ${PROJECT_NAME})

#add_executable(dsp_slam dsp_slam.cc)
#target_link_libraries(dsp_slam ${PROJECT_NAME})

#add_executable(mono_snark mono_snark.cc)
#target_link_libraries(mono_snark ${PROJECT_NAME})

#add_executable(reconstruct_frame reconstruct_frame.cpp)
#target_link_libraries(reconstruct_frame ${PROJECT_NAME})

#add_executable(image_from_bin image_from_bin.cc)
#target_link_libraries(image_from_bin ${PROJECT_NAME})
