#!/bin/bash

#
# COVINS:
#   catkin tools
#   ros
#   
# mmlab:
#   Anaconda3 (Archiconda3-0.2.3-Linux-aarch64.sh)
#   cuda (10.2 or 11.3) + tensorrt (8.2.1.9-1+cuda10.2)
#   pytorch (1.10.0) + torchvision (0.11.1)
#
# orbslam3:
#   comma
#   snark
#


if [ $# -eq 0 ]
then
    NR_JOBS=""
    CATKIN_JOBS=""
else
    NR_JOBS=${1:-}
    CATKIN_JOBS="-j${NR_JOBS}"
fi

FILEDIR=$(readlink -f ${BASH_SOURCE})
BASEDIR=$(dirname ${FILEDIR})
# BASEDIR is ??/<ws_name>/src/covins-dsdf
echo "File directory: ${BASEDIR}"
cd ${BASEDIR}/..

# Clone all dependencies

git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/ceres_catkin.git
git clone https://github.com/ethz-asl/opengv.git
git clone https://github.com/ethz-asl/opencv3_catkin.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/ethz-asl/gflags_catkin.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/doxygen_catkin.git
git clone https://github.com/ethz-asl/suitesparse
git clone https://github.com/ethz-asl/yaml_cpp_catkin.git
git clone https://github.com/ethz-asl/catkin_boost_python_buildtool.git
git clone https://github.com/ethz-asl/minkindr.git
git clone https://github.com/ethz-asl/protobuf_catkin.git
git clone https://github.com/ethz-asl/aslam_cv2.git
git clone https://github.com/ethz-asl/numpy_eigen.git 
git clone https://github.com/VIS4ROB-lab/robopt_open.git -b fix_imu_residual

chmod +x object_slam/fix_eigen_deps.sh
./object_slam/fix_eigen_deps.sh

# Install dependencies and configure space

sudo apt-get install python3-catkin-tools
python3 -m pip install catkin_pkg
catkin config --merge-devel

# Bug fix

if [ ! -d src/aslam_cv2/aslam_cv_common ]; then
   FILE_PATH="src/aslam_cv2/aslam_cv_common/CMakeLists.txt"
   sed -i 's|cs_export(CFG_EXTRAS detect_simd.cmake export_flags.cmake setup_openmp.cmake)|cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}/compiled_proto\n          CFG_EXTRAS detect_simd.cmake export_flags.cmake setup_openmp.cmake)|' "$FILE_PATH"
fi


set -e
cd ${BASEDIR}/../..
PYTHON_EXEC=$(which python3)
PYTHON_INCLUDE=$(python3 -c "from sysconfig import get_paths; print(get_paths()['include'])")
catkin build ${CATKIN_JOBS} eigen_catkin opencv3_catkin -DPYTHON_EXECUTABLE=$PYTHON_EXEC -DPYTHON_INCLUDE_DIR=$PYTHON_INCLUDE -DCMAKE_CXX_STANDARD=14
source devel/setup.bash

# Build the backend

cd ${BASEDIR}/covins_backend/
cd thirdparty
cd DBoW2
if [ ! -d build ]; then
  mkdir build
fi
cd build
cmake ..
make -j8
cd ../..

cd ${BASEDIR}/../..
catkin build ${CATKIN_JOBS} covins_backend

# Extract vocabulary

cd ${BASEDIR}/covins_backend/
cd config
if [ ! -f "ORBvoc.txt" ]
then
  unzip ORBvoc.txt.zip
fi

# Source catkin settings

cd ${BASEDIR}/../..
source devel/setup.bash

# Build orb_slam3

cd ${BASEDIR}/orb_slam3
./build.sh

#if [[ $* == *--create-conda-env* ]]
#then
#   echo "Installing dsp-slam with new environment"
#   ./build_cuda113.sh --create-conda-env --build-dependencies
#else
#   echo "Installing dsp-slam with previous environment"
#   ./build_cuda113.sh --build-dependencies
#fi

#finish
exit 0
