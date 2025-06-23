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

#ssh agx@192.168.42.128
#Quad123

#agx@vulcan-xavier-agx-ind-base:~$ cat /etc/nv_tegra_release
## R35 (release), REVISION: 4.1, GCID: 33958178, BOARD: t186ref, EABI: aarch64, DATE: Tue Aug  1 19:57:35 UTC 2023

#agx@vulcan-xavier-agx-ind-base:~$ sudo apt-cache show nvidia-jetpack
#[sudo] password for agx: 
#Package: nvidia-jetpack
#Version: 5.1.2-b104
#Architecture: arm64
#Maintainer: NVIDIA Corporation
#Installed-Size: 194
#Depends: nvidia-jetpack-runtime (= 5.1.2-b104), nvidia-jetpack-dev (= 5.1.2-b104)
#Homepage: http://developer.nvidia.com/jetson
#Priority: standard
#Section: metapackages
#Filename: pool/main/n/nvidia-jetpack/nvidia-jetpack_5.1.2-b104_arm64.deb
#Size: 29304
#SHA256: fda2eed24747319ccd9fee9a8548c0e5dd52812363877ebe90e223b5a6e7e827
#SHA1: 78c7d9e02490f96f8fbd5a091c8bef280b03ae84
#MD5sum: 6be522b5542ab2af5dcf62837b34a5f0
#Description: NVIDIA Jetpack Meta Package
#Description-md5: ad1462289bdbc54909ae109d1d32c0a8


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
        --install-ros
        --build-dependencies
        --install-conda
        --create-conda-env"

ORBSLAM3=$(pwd)
THIRDPARTY=${ORBSLAM3}/Thirdparty

highlight "Installing system-wise packages ..."
sudo apt-get update > /dev/null 2>&1 &&
sudo apt-get install -y \
  libglew-dev \
  libgtk2.0-dev \
  pkg-config \
  libegl1-mesa-dev \
  libwayland-dev \
  libxkbcommon-dev \
  wayland-protocols \
  libdc1394-dev \
  libopenexr-dev

<< comment
# bashrc should contain
source /opt/ros/noetic/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
# set environment variable for conda
. ~/anaconda3/etc/profile.d/conda.sh
export PATH=$PATH:~/anaconda3/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/dv/anaconda3/lib #I post-append anaconda to make sure system files are used first
# set environment variable for pip
export OPENBLAS_CORETYPE=ARMV8
export PATH=/usr/local/cuda-11.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/extras/CUPTI/lib64:/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH
comment

# build dependencies (OpenCV, Eigen3, Pangolin, g2o, DBoW2, Sophus)
if [[ $* == *--build-dependencies* ]]; then

  highlight "Installing OpenCV ..."
  cd $THIRDPARTY
  if [ ! -d opencv ]; then
    git_clone "git clone --branch 3.4.1 --depth=1 https://github.com/opencv/opencv.git"
  fi
  cd opencv
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DWITH_CUDA=OFF  \
      -DBUILD_DOCS=OFF  \
      -DBUILD_PACKAGE=OFF \
      -DBUILD_TESTS=OFF  \
      -DBUILD_PERF_TESTS=OFF  \
      -DBUILD_opencv_apps=OFF \
      -DBUILD_opencv_calib3d=ON  \
      -DBUILD_opencv_cudaoptflow=OFF  \
      -DBUILD_opencv_dnn=OFF  \
      -DBUILD_opencv_dnn_BUILD_TORCH_IMPORTER=OFF  \
      -DBUILD_opencv_features2d=ON \
      -DBUILD_opencv_flann=ON \
      -DBUILD_opencv_java=ON  \
      -DBUILD_opencv_objdetect=ON  \
      -DBUILD_opencv_python2=OFF  \
      -DBUILD_opencv_python3=OFF  \
      -DBUILD_opencv_photo=ON \
      -DBUILD_opencv_stitching=ON  \
      -DBUILD_opencv_superres=ON  \
      -DBUILD_opencv_shape=ON  \
      -DBUILD_opencv_videostab=OFF \
      -DBUILD_PROTOBUF=OFF \
      -DWITH_1394=OFF  \
      -DWITH_GSTREAMER=OFF  \
      -DWITH_GPHOTO2=OFF  \
      -DWITH_MATLAB=OFF  \
      -DWITH_NVCUVID=OFF \
      -DWITH_OPENCL=OFF \
      -DWITH_OPENCLAMDBLAS=OFF \
      -DWITH_OPENCLAMDFFT=OFF \
      -DWITH_TIFF=OFF  \
      -DWITH_VTK=OFF  \
      -DWITH_WEBP=OFF  \
      ..
  make -j3
  OpenCV_DIR=$(pwd)

  highlight "Installing Eigen3 ..."
  cd $THIRDPARTY
  if [ ! -d eigen ]; then
     git_clone "git clone --branch=3.3.4 --depth=1 https://gitlab.com/libeigen/eigen.git"
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
  make -j3
  make install

  highlight "Installing Pangolin ..."
  #sudo apt-get install --reinstall libcaca0 libcaca-dev libncurses5-dev libncursesw5-dev
  # in Pangolin/src/CMakeLists.txt
  # line 131:     list(APPEND LINK_LIBS rt pthread ncursesw) # Tariq added lncursesw
  cd $THIRDPARTY/Pangolin
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake ..
  make -j3
  Pangolin_DIR=$(pwd)

  highlight "Installing g2o ..."
  cd $THIRDPARTY/g2o
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake -DEigen3_DIR="$(pwd)/../../eigen/install/share/eigen3/cmake" ..
  make -j3

  highlight "Installing DBoW2 ..."
  cd $THIRDPARTY/DBoW2
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake -DOpenCV_DIR=$OpenCV_DIR ..
  make -j3

  highlight "Installing Sophus ..."
  cd $THIRDPARTY/Sophus
  if [ ! -d build ]; then
    mkdir build
  fi
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j3
  
  #comma
  highlight "Installing comma ..."
  #sudo apt install ansible
  pip install qtconsole ruamel-yaml
  #pip install ansible
  pip install comma-py==1.0.0
  if ! command -v csv-play &> /dev/null; then
    echo "'comma' not found! Attempting to install ..."
    cd $THIRDPARTY
    if [ ! -d comma ]; then
       git_clone "git clone https://github.com/mission-systems-pty-ltd/comma.git"
    fi
    cd comma
    if [ ! -d build ]; then
      mkdir build
    fi
    cd build
    cmake ..
    make -j3 && sudo make install
  fi  

  #snark
  highlight "Installing snark ..."
  sudo apt install qt3d5-dev qt5-default libqt5charts5-dev libqt5charts5-dev libexiv2-dev libassimp-dev
  if ! command -v cv-cat &> /dev/null; then
    echo "'snark' not found! Attempting to install ..."
    cd $THIRDPARTY
    if [ ! -d snark ]; then
       git_clone "git clone https://github.com/mission-systems-pty-ltd/snark.git";
    fi
    cd snark
    if [ ! -d build ]; then
      mkdir build
    fi
    cd build
    cmake \
    -Dsnark_build_imaging_opencv_contrib=OFF \
    -Dsnark_build_navigation=ON \
    -Dsnark_build_sensors_dc1394=OFF \
    -Dsnark_build_sensors_ouster=ON \
    -Dsnark_build_ros=ON \
    ..
    make -j3 && sudo make install
  fi

fi # --build-dependencies


# (1) install conda
#https://repo.anaconda.com/archive/
#if [[ $* == *--install-conda* ]] ; then
if ! command -v conda &> /dev/null; then
  highlight "Installing anaconda ..."
  cd ~/
  wget https://repo.anaconda.com/archive/Anaconda3-2021.04-Linux-aarch64.sh
  bash Anaconda3-2021.04-Linux-aarch64.sh -b
  echo -e '\n# set environment variable for conda' >> ~/.bashrc
  echo ". ~/anaconda3/etc/profile.d/conda.sh" >> ~/.bashrc
  echo 'export PATH=$PATH:~/anaconda3/bin' >> ~/.bashrc
  echo -e '\n# set environment variable for pip' >> ~/.bashrc
  echo 'export OPENBLAS_CORETYPE=ARMV8' >> ~/.bashrc
  source ~/.bashrc
  conda --version
fi # --install-conda

# (2) conda environment
conda_base=$(conda info --base)
#conda_base="/home/agx/anaconda3" # FIXME this is hard coded path
source "$conda_base/etc/profile.d/conda.sh"
export PYTHON_VERSION=`python3 --version | cut -d' ' -f 2 | cut -d'.' -f1,2` # get the version of python3 installed by default
if [[ $* == *--create-conda-env* ]] ; then
  highlight "Creating Python environment ..."
  conda create -y -n mmdeploy python=${PYTHON_VERSION}
fi # --create-conda-env
conda activate mmdeploy

# general system installs
sudo apt-get -y update; 
sudo apt-get -y install \
   autoconf \
   bc \
   build-essential \
   g++-8 gcc-8 clang-8 \
   lld-8 \
   gettext-base \
   gfortran-8 \
   iputils-ping \
   libbz2-dev \
   libc++-dev \
   libcgal-dev \
   libffi-dev \
   libfreetype6-dev \
   libhdf5-dev \
   libjpeg-dev \
   liblzma-dev \
   libncurses5-dev \
   libncursesw5-dev \
   libpng-dev \
   libreadline-dev \
   libssl-dev \
   libsqlite3-dev \
   libxml2-dev \
   libxslt-dev \
   locales \
   moreutils \
   openssl \
   python-openssl \
   rsync \
   scons \
   python3-pip \
   libopenblas-dev;
   
# mmlab system installs
sudo apt-get -y install
   libssl-dev \
   pkg-config libhdf5-103 libhdf5-dev \
   libspdlog-dev;
sudo apt-get install llvm
pip install open3d pccm
   
export LD_LIBRARY_PATH=/usr/lib/llvm-10/lib:$LD_LIBRARY_PATH

# conda installs
conda install aiohttp numpy==1.23.5 numba==0.53.0 scipy cython ninja
conda install scikit-learn scikit-image
conda install -c conda-forge pycocotools protobuf==3.20.2 empy catkin_pkg onnx versioned-hdf5 pycuda

#  (3) pytorch
# https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
if [[ $* == *--install-pytorch* ]] ; then
  highlight "Installing pytorch ..."
  cd $THIRDPARTY
  #conda install cmake
  #sudo apt-get update
  #sudo apt-get install --reinstall libffi7 libffi-dev libp11-kit0
  #sudo apt-get install --reinstall git
  #sudo apt-get update
  #mv $conda_base/envs/mmdeploy/lib/libffi.so.7 $conda_base/envs/mmdeploy/lib/libffi.so.7.bak
  if [ ! -d pytorch ]; then
    git clone --recursive -b v1.11.0 https://github.com/pytorch/pytorch
  fi
  cd pytorch
  sed -i "s/\('8.6'\)]/\1, '8.7']/" torch/utils/cpp_extension.py # modify the file and add Orin architecture 8.7 (required to compile torchvision versus cuda)
  #pip install -r requirements.txt
  conda install astunparse expecttest future psutil pyyaml requests setuptools six 
  conda install -c conda-forge types-dataclasses typing_extensions dataclasses
  export _GLIBCXX_USE_CXX11_ABI=1
  export USE_CUDA=1
  export MAX_JOBS=8 # Adjust the number according to your CPU cores
  export CMAKE_PREFIX_PATH=${CONDA_PREFIX:-"$(dirname $(which conda))/../"}
  python setup.py develop
  cd $THIRDPARTY
  python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())" # Will print the version of torch
fi

# test:
# $ python3
# >>> import torch
# >>> print(torch.__version__)
# 2.2.0a0+gite716505
# >>> print(torch.cuda.is_available())

#To clean pytorch
#$ cd pytorch
#$ python3 setup.py clean
#$ rm -rf build
#$ rm -rf dist
#$ rm -rf torch.egg-info

# (4) torchvision
# gedit gedit /home/dv/catkin_ws/src/covins-dsdf/orb_slam3/Thirdparty/pytorch/torch/utils/cpp_extension.py
# search architectures and add 8.7
if [[ $* == *--install-pytorch* ]] ; then
  highlight "Installing torchvision ..."
  cd $THIRDPARTY
  if [ ! -d vision ]; then
    git clone -b v0.15.1 https://github.com/pytorch/vision.git
  fi
  cd vision
  python setup.py develop
  cd $THIRDPARTY
  python -c "import torchvision; print(torchvision.__version__)" # Will print the version of torchvision
fi


# (5) tensorrt
highlight "Installing tensorrt ..."
#sudo apt install tensorrt tensorrt-dev # SDKManager will install this one, when Orin is flashed
# #pip install nvidia-pyindex
# #sudo apt install nvidia-jetpack
dpkg-query -W tensorrt
# tensorrt	8.5.2.2-1+cuda11.4
export PYTHON_VERSION=`python3 --version | cut -d' ' -f 2 | cut -d'.' -f1,2` # get the version of python3 installed by default
cp -r /usr/lib/python${PYTHON_VERSION}/dist-packages/tensorrt* ${conda_base}/envs/mmdeploy/lib/python${PYTHON_VERSION}/site-packages/
conda deactivate
conda activate mmdeploy
python -c "import tensorrt; print(tensorrt.__version__)" # Will print the version of TensorRT
# set environment variable for building mmdeploy later on
export TENSORRT_DIR=/usr/include/aarch64-linux-gnu
echo -e '\n# set environment variable for TensorRT' >> ~/.bashrc
echo 'export TENSORRT_DIR=/usr/include/aarch64-linux-gnu' >> ~/.bashrc
source ~/.bashrc
conda activate mmdeploy

# (6) install mmlab stuff
# https://docs.nvidia.com/jetson/jetpack/install-jetpack/index.html
# https://github.com/open-mmlab/mmdeploy/blob/main/docs/en/01-how-to-build/jetsons.md
# https://developer.nvidia.com/embedded/vulkan

#MMCV
highlight "Installing MMCV (it takes about 1 hour 40 minutes to install on a Jetson Nano)."
cd $THIRDPARTY
sudo apt-get install -y libssl-dev
if [ ! -d mmcv ]; then
  git_clone "git clone -b v1.7.0 https://github.com/open-mmlab/mmcv.git"
fi
cd mmcv
#pip install -r requirements/build.txt
conda install --file requirements/build.txt
#MMCV_WITH_OPS=1 pip install -e .
MMCV_WITH_OPS=1 python setup.py develop

#ONNX
#python3 -m pip install onnx
# or
#conda install -c conda-forge onnx
#-----
#if they fail, install the following dependencies
#sudo apt-get install protobuf-compiler libprotoc-dev #only if next line fails, run this line
#-----

#Install h5py and pycuda (may take 6min)
#Model Converter employs HDF5 to save the calibration data for TensorRT INT8 quantization and needs pycuda to copy device memory.
#sudo apt-get install -y pkg-config libhdf5-103 libhdf5-dev 
#python3 -m pip install versioned-hdf5 pycuda
#conda install -c conda-forge versioned-hdf5 pycuda

#spdlog
#sudo apt-get install -y libspdlog-dev

#ppl.cv
highlight "Installing ppl.cv ..."
cd $THIRDPARTY
if [ ! -d ppl.cv ]; then
   git_clone "git clone https://github.com/openppl-public/ppl.cv.git"
   cd ppl.cv
   export PPLCV_DIR=$(pwd)
   echo -e '\n# set environment variable for ppl.cv' >> ~/.bashrc
   echo "export PPLCV_DIR=$(pwd)" >> ~/.bashrc
   #./build.sh cuda
fi


#mmdeploy
highlight "Installing mmdeploy ..."
cd $THIRDPARTY
if [ ! -d mmdeploy ]; then
   git_clone "git clone -b main --recursive https://github.com/open-mmlab/mmdeploy.git"
fi
cd mmdeploy
export MMDEPLOY_DIR=$(pwd)
# build TensorRT custom operators
mkdir -p build && cd build
cmake .. -DMMDEPLOY_TARGET_BACKENDS="trt"
make -j$(nproc) && make install
# install model converter (may take 5 minutes)
cd ${MMDEPLOY_DIR}
#pip install -v -e .
python setup.py develop

#mmdet
highlight "Installing mmdet ..."
cd $THIRDPARTY
if [ ! -d mmdetection ]; then   # check mmdet/__init__.py  for compatible mmcv versions
  git_clone "git clone -b v2.28.2 https://github.com/open-mmlab/mmdetection.git"    # mmcv 1.3.17 to 1.8.0
fi
cd mmdetection
python setup.py build
#pip install -r requirements/build.txt
#pip install -v -e .
python setup.py develop

#mmseg
highlight "Installing mmseg ..."
cd $THIRDPARTY
if [ ! -d mmsegmentation ]; then   # check mmseg/__init__.py  for compatible mmcv versions
  git_clone "git clone -b v0.30.0 https://github.com/open-mmlab/mmsegmentation.git"    # mmcv 1.3.13 to 1.8.0
fi
cd mmsegmentation
python setup.py build
#pip install -r requirements.txt -e .
python setup.py develop

#spconv (you may need to do this one)
#sudo apt-get install libboost-all-dev
#cd {$THIRDPARTY}
#git clone https://github.com/traveller59/spconv.git --recursive
#cd spconv
##this will create a wheel
#python setup.py bdist_wheel
#cd dist
#python -m pip install <name_of_the_whl_file>.whl

#mmdet3d
highlight "Installing mmdet3d ..."
cd $THIRDPARTY
if [ ! -d mmdetection3d ]; then   # check mmdet3d/__init__.py  for compatible mmcv versions
  git_clone "git clone -b v1.0.0rc6 https://github.com/open-mmlab/mmdetection3d.git"   # mmcv 1.5.2 to 1.7.0, mmdet 2.24.0 to 3.0.0, mmseg 0.20.0 to 1.0.0
fi
cd mmdetection3d
#pip install -e .
python setup.py build
python setup.py develop

#Object-SLAM
highlight "building DSP-ORBSLAM3 ..."
cd $ORBSLAM3
if [ ! -d build ]; then
  mkdir build
fi
cd build
conda activate mmdeploy
source $ORBSLAM3/../../../devel/setup.bash

conda_python_bin=`which python3` # ~/anaconda3/envs/mmdeploy/bin/python3
conda_env_dir="$(dirname "$(dirname "$conda_python_bin")")" # ~/anaconda3/envs/mmdeploy
PYTHON_VERSION_FULL=$(python3 --version 2>&1) # Python 3.8.17
PYTHON_VERSION_SHORT=$(echo $PYTHON_VERSION_FULL | cut -d' ' -f2 | cut -d'.' -f1,2) # 3.8
#export CMAKE_PREFIX_PATH=${conda_env_dir}
#export LD_LIBRARY_PATH=${conda_env_dir}"/lib:${LD_LIBRARY_PATH}"

#FIXME - why do I have to create this link?
ln -s /usr/lib/aarch64-linux-gnu/libgeos_c.so ${conda_env_dir}/lib/libgeos_c.so

cmake \
  -DEigen3_DIR="$(pwd)/../Thirdparty/eigen/install/share/eigen3/cmake" \
  -DPangolin_DIR="$(pwd)/../Thirdparty/Pangolin/build/src" \
  -DPYTHON_LIBRARIES="${conda_env_dir}/lib/libpython${PYTHON_VERSION_SHORT}.so" \
  -DPYTHON_INCLUDE_DIRS="${conda_env_dir}/include/python${PYTHON_VERSION_SHORT}" \
  -DPYTHON_EXECUTABLE="${conda_env_dir}/bin/python${PYTHON_VERSION_SHORT}" \
  -DCMAKE_BUILD_TYPE=Debug \
  ..
  
#  -DCONDA_LIBS="/usr/lib/aarch64-linux-gnu" \ #"${conda_env_dir}/lib" \  
#  -DOpenCV_DIR="$(pwd)/../Thirdparty/opencv/build" \
#  -DPangolin_INCLUDE_DIRS="$(pwd)/../Thirdparty/Pangolin/include" \
#  -Dcatkin_DIR="/opt/ros/${ROS_VERSION}/share/catkin/cmake" \
#  -Deigen_catkin_DIR="../../../../devel/share/eigen_catkin/cmake/" \
#  -Dopencv3_catkin_DIR="../../../../devel/share/opencv3_catkin/cmake/" \

make -j$(nproc)
