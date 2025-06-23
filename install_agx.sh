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

# install ROS
#if [[ $* == *--install-ros* ]] ; then
#   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#   sudo apt update
#   sudo apt install ros-noetic-desktop-full
#   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
## Maybe dont do this one ##   echo 'export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages' >> ~/.bashrc
#   source ~/.bashrc
#   echo $PATH
#fi # --install-ros

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

# bug fix
if [ ! -d ${BASEDIR}/../aslam_cv2/aslam_cv_common ]; then
   git clone https://github.com/ethz-asl/aslam_cv2.git
   FILE_PATH="${BASEDIR}/../aslam_cv2/aslam_cv_common/CMakeLists.txt"
   sed -i 's|cs_export(CFG_EXTRAS detect_simd.cmake export_flags.cmake setup_openmp.cmake)|cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}/compiled_proto\n          CFG_EXTRAS detect_simd.cmake export_flags.cmake setup_openmp.cmake)|' "$FILE_PATH"
fi

#chmod +x ./src/covins-dsdf/fix_eigen_deps.sh
bash ${BASEDIR}/fix_eigen_deps.sh

set -e

#export PATH=/usr/local/cuda-11.4/bin:$PATH
#export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH
#source /opt/ros/noetic/setup.bash

#sudo apt install python3-catkin python3-catkin-tools python3-catkin-pkg
#python3 -m pip install catkin_pkg
#catkin config --merge-devel

PYTHON_EXEC=$(which python3)
PYTHON_INCLUDE=$(python3 -c "from sysconfig import get_paths; print(get_paths()['include'])")

cd ${BASEDIR}/../..
catkin build ${CATKIN_JOBS} eigen_catkin opencv3_catkin -DPYTHON_EXECUTABLE=$PYTHON_EXEC -DPYTHON_INCLUDE_DIR=$PYTHON_INCLUDE
cd ${BASEDIR}/../..
source devel/setup.bash

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
source devel/setup.bash

#cd ${BASEDIR}/orb_slam3
#if conda env list | grep -q 'mmdeploy'; then
#   echo "Starting dsp-slam with an already existing environment"
#   ./build_agx_5.1.2.sh --build-dependencies
#else
#   echo "Starting dsp-slam with a new environment"
#   ./build_agx_5.1.2.sh --build-dependencies --create-conda-env 
#fi

#finish
exit 0
