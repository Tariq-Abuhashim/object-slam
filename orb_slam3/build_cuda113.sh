#!/bin/bash -e
#
# This is a build script for DSP-ORBSLAM3.
#
# Use parameters:
# `--install-cuda` to install the NVIDIA CUDA suite
#
# Example:
#   ./build_cuda113.sh --install-cuda --build-dependencies --create-conda-env
#
#   which will
#   1. Install some system dependencies
#   2. Install CUDA-11.3 under /usr/local
#   3. Create and build:
#   - ./Thirdparty/opencv
#   - ./Thirdparty/eigen
#   - ./Thirdparty/Pangolin
#   4. Build:
#   - ./Thirdparty/g2o
#   - ./Thirdparty/DBoW2
#   5. Create conda env with PyTorch 1.10
#   6. Install mmdetection and mmdetection3d
#   7. Build DSP-ORBSLAM3

# Function that executes the clone command given as $1 iff repo does not exist yet. Otherwise pulls.
# Only works if repository path ends with '.git'
# Example: git_clone "git clone --branch 3.4.1 --depth=1 https://github.com/opencv/opencv.git"
function git_clone(){
  repo_dir=`basename "$1" .git`
  git -C "$repo_dir" pull 2> /dev/null || eval "$1"
}

source Thirdparty/bashcolors/bash_colors.sh
function highlight(){
  clr_magentab clr_bold clr_white "$1"
}

highlight "Starting DSP-SLAM3 build script ..."
echo "Available parameters:
        --install-cuda
        --build-dependencies
        --create-conda-env"

highlight "Installing system-wise packages ..."
sudo apt-get update > /dev/null 2>&1 &&
sudo apt -y install gcc-8 g++-8 # gcc-8 is a safe version 
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
sudo apt-get install -y cmake
sudo apt-get install -y \
  libglew-dev \
  libgtk2.0-dev \
  pkg-config \
  libegl1-mesa-dev \
  libwayland-dev \
  libxkbcommon-dev \
  wayland-protocols

# install CUDA 11.3
if [[ $* == *--install-cuda* ]] ; then
  highlight "Installing CUDA..."
  wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda_11.3.0_465.19.01_linux.run
  sudo sh cuda_11.3.0_465.19.01_linux.run
  rm cuda_11.3.0_465.19.01_linux.run
fi # --install-cuda
export PATH=/usr/local/cuda-11.3/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.3/lib64:$LD_LIBRARY_PATH

if [[ $* == *--build-dependencies* ]]; then

  cd Thirdparty

  highlight "Installing Eigen3 ..."
  if [ ! -d eigen ]; then
     git_clone "git clone --branch=3.4.0 --depth=1 https://gitlab.com/libeigen/eigen.git"
  fi
  cd eigen
  if [ ! -d build ]; then
    mkdir build
  fi
  if [ ! -d install ]; then
    mkdir install
  fi
  cd build
  cmake -DCMAKE_INSTALL_PREFIX="$(pwd)/../install" ..
  make -j8
  make install
  cd ../..

  highlight "Installing Pangolin ..."
  cd Pangolin
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake ..
  make -j8
  Pangolin_DIR=$(pwd)
  cd ../..

  highlight "Installing g2o ..."
  cd g2o
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake -DEigen3_DIR="$(pwd)/../../eigen/install/share/eigen3/cmake" ..
  make -j8
  cd ../..

  highlight "Installing DBoW2 ..."
  cd DBoW2
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake -DOpenCV_DIR=$OpenCV_DIR ..
  make -j8
  cd ../..

  highlight "Installing Sophus ..."
  cd Sophus
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j8
  cd ../../..

fi # --build-dependencies

#if [[ $* == *--create-conda-env* ]] ; then
#  highlight "Creating Python environment ..."
#  conda config --add channels conda-forge
#  conda env create -f environment_cuda113.yml
#fi # --create-conda-env

conda_base=$(conda info --base)
source "$conda_base/etc/profile.d/conda.sh"

if [[ $* == *--create-conda-env* ]] ; then
   conda create --name dsp-slam3 python=3.7
   conda activate dsp-slam3
   conda install pytorch=1.10.0 torchvision=0.11.0 torchaudio=0.10.0 cudatoolkit=11.3  -c pytorch -c conda-forge # FIXME

   highlight "Installing mmdetection and mmdetection3d ..."
   pip3 install pycocotools #==2.0.1
   pip3 install scikit-image==0.18.3 #FIXME
   #pip install mmcv-full==1.3.13 -f https://download.openmmlab.com/mmcv/dist/cu113/torch1.10.0/index.html
   #pip install mmdet==2.19.0
   #pip install mmsegmentation==0.20.0
   #cd Thirdparty
   #if [ ! -d mmdetection3d ]; then
   #  git_clone "git clone -b v0.18.1 https://github.com/open-mmlab/mmdetection3d.git"
   #fi
   #cd mmdetection3d
   #pip install -v -e .
   #cd ../..

   cd Thirdparty
   if [ ! -d mmcv ]; then
     git_clone "git clone -b v1.7.0 https://github.com/open-mmlab/mmcv.git"
   fi
   cd mmcv
   #export CMAKE_PREFIX_PATH=${CONDA_PREFIX:-"$(dirname $(which conda))/../"}
   python setup.py build
   MMCV_WITH_OPS=1 pip install -r requirements.txt -e .
   cd ..

   if [ ! -d mmdetection ]; then   # check mmdet/__init__.py  for compatible mmcv versions
     git_clone "git clone -b v2.28.2 https://github.com/open-mmlab/mmdetection.git"    # mmcv 1.3.17 to 1.8.0
   fi
   cd mmdetection
   python setup.py build
   pip install -r requirements.txt -e .
   #pip install -r requirements/build.txt
   #pip install -v -e .
   cd ..

   if [ ! -d mmsegmentation ]; then   # check mmseg/__init__.py  for compatible mmcv versions
     git_clone "git clone -b v0.30.0 https://github.com/open-mmlab/mmsegmentation.git"    # mmcv 1.3.13 to 1.8.0
   fi
   cd mmsegmentation
   python setup.py build
   pip install -r requirements.txt -e .
   #pip install -e .
   cd ..

   if [ ! -d mmdetection3d ]; then   # check mmdet3d/__init__.py  for compatible mmcv versions
     git_clone "git clone -b v1.0.0rc6 https://github.com/open-mmlab/mmdetection3d.git"   # mmcv 1.5.2 to 1.7.0, mmdet 2.24.0 to 3.0.0, mmseg 0.20.0 to 1.0.0
   fi
   cd mmdetection3d
   python setup.py build
   pip install -r requirements.txt -e .
   cd ../..
fi # --create-conda-env

highlight "building DSP-ORBSLAM3 ..."
conda activate dsp-slam3
pip install catkin_pkg  # this is for covins
if [ ! -d build ]; then
  mkdir build
fi
cd build
conda_python_bin=`which python`
conda_env_dir="$(dirname "$(dirname "$conda_python_bin")")"
cmake \
  -DEigen3_DIR="$(pwd)/../Thirdparty/eigen/install/share/eigen3/cmake" \
  -DPangolin_DIR="$(pwd)/../Thirdparty/Pangolin/build/src" \
  -DPYTHON_LIBRARIES="$conda_env_dir/lib/libpython3.7m.so" \
  -DPYTHON_INCLUDE_DIRS="$conda_env_dir/include/python3.7m" \
  -DPYTHON_EXECUTABLE="$conda_env_dir/bin/python3.7" \
  -DCMAKE_BUILD_TYPE=Debug \
  ..
#-DOpenCV_DIR="$(pwd)/../Thirdparty/opencv/build" \
#-DPangolin_INCLUDE_DIRS="$(pwd)/../Thirdparty/Pangolin/include" \
make -j8
