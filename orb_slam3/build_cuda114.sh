#!/bin/bash -e
#
# Build script for object-slam.
#
# Usage:
#   ./build_cuda114.sh [--install-cuda] [--build-dependencies] [--create-conda-env]
#
# Example:
#   ./build_cuda114.sh --build-dependencies --create-conda-env
#

# Always resolve the location of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR"
BUILD_DIR="$SRC_DIR/build"

# -------------------------------------------------------------
# Functions
# -------------------------------------------------------------

# Clones a repo if missing, otherwise pulls
function git_clone() {
  cmd=("$@")
  target_dir="${cmd[-1]}"
  if [ -d "$target_dir" ]; then
    echo "Repo exists. Pulling latest in $target_dir ..."
    git -C "$target_dir" pull
  else
    "${cmd[@]}"
  fi
}

source Thirdparty/bashcolors/bash_colors.sh
function highlight(){
  clr_magentab clr_bold clr_white "$1"
}

# -------------------------------------------------------------
# Script starts
# -------------------------------------------------------------

highlight "Starting object-slam build script ..."

echo "Available parameters:
    --build-dependencies
    --create-conda-env
"

highlight "Installing system packages ..."
sudo apt-get update > /dev/null 2>&1
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  libglew-dev \
  libgtk2.0-dev \
  pkg-config \
  libegl1-mesa-dev \
  libwayland-dev \
  libxkbcommon-dev \
  wayland-protocols \
  libgl1-mesa-dev \
  libpython3-dev \
  libepoxy-dev

# -------------------------------------------------------------
# Optional: Build third-party dependencies
# -------------------------------------------------------------

if [[ $* == *--build-dependencies* ]]; then

  mkdir -p Thirdparty
  cd Thirdparty

  highlight "Installing Eigen3 ..."
  git_clone git clone --branch=3.4.0 --depth=1 https://gitlab.com/libeigen/eigen.git eigen
  cd eigen
  mkdir -p build install
  cd build
  cmake -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX="$(pwd)/../install" ..
  make -j$(nproc)
  make install
  cd ../..

  highlight "Installing Pangolin ..."
  git_clone git clone --recursive https://github.com/stevenlovegrove/Pangolin.git Pangolin
  cd Pangolin
  git checkout v0.9
  mkdir -p build
  cd build
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF #\
    #-DBUILD_PANGOLIN_PYTHON=OFF  # Optional: Disable Python if not needed
  make -j$(nproc)
  Pangolin_DIR=$(pwd)
  cd ../..

  highlight "Installing OpenCV ..."
  #git_clone git clone --branch 3.4.1 --depth=1 https://github.com/opencv/opencv.git opencv
  #cd opencv
  #mkdir -p build install
  #cd build
  #cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(pwd)/../install" ..
  #make -j$(nproc)
  #make install
  #OpenCV_DIR="$(pwd)/../install"
  #cd ../..

  highlight "Installing g2o ..."
  #git_clone git clone https://github.com/RainerKuemmerle/g2o.git g2o
  cd g2o
  mkdir -p build
  cd build
  cmake -DEigen3_DIR="$(pwd)/../../eigen/install/share/eigen3/cmake" ..
  make -j$(nproc)
  cd ../..

  highlight "Installing DBoW2 ..."
  #git_clone git clone https://github.com/dorian3d/DBoW2.git DBoW2
  cd DBoW2
  mkdir -p build
  cd build
  cmake .. #-DOpenCV_DIR="$OpenCV_DIR" ..
  make -j$(nproc)
  cd ../..

  highlight "Installing Sophus ..."
  #git_clone git clone https://github.com/strasdat/Sophus.git Sophus
  cd Sophus
  mkdir -p build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j$(nproc)
  cd ../../..

  cd ..

fi # --build-dependencies

# -------------------------------------------------------------
# Optional: Create Conda environment
# -------------------------------------------------------------

if [[ $* == *--create-conda-env* ]]; then
  highlight "Creating conda environment ..."
  conda_base=$(conda info --base)
  source "$conda_base/etc/profile.d/conda.sh"
  conda create -y --name dsp-slam3 python=3.8
  conda activate dsp-slam3

  # get conda env paths
  conda_python_bin=$(which python)
  conda_env_dir="$(dirname "$(dirname "$conda_python_bin")")"
else
  # fallback to system python
  conda_env_dir="/usr"
fi

# -------------------------------------------------------------
# Install mmcv, mmdet etc.
# -------------------------------------------------------------

highlight "Installing mmlab components ..."
python3 -m pip install mmcv-full==1.7.0 mmdet==2.28.2 mmsegmentation==0.30.0 \
      mmdet3d==1.0.0rc6 -f https://download.openmmlab.com/mmcv/dist/cu114/torch1.12.0/index.html

# -------------------------------------------------------------
# Build object-slam
# -------------------------------------------------------------

highlight "Building object-slam ..."

python3 -m pip install catkin_pkg
sudo apt install libncurses-dev

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake \
  -DEigen3_DIR="$(pwd)/../Thirdparty/eigen/install/share/eigen3/cmake" \
  -DPangolin_DIR="$(pwd)/../Thirdparty/Pangolin/build/src" \
  -DPYTHON_LIBRARIES="$conda_env_dir/lib/libpython3.8.so" \
  -DPYTHON_INCLUDE_DIRS="$conda_env_dir/include/python3.8" \
  -DPYTHON_EXECUTABLE="$conda_env_dir/bin/python3.8" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DOpenCV_DIR="$(pwd)/../Thirdparty/opencv/install" \
  "$SRC_DIR"

make VERBOSE=1 -j$(nproc)

highlight "object-slam build complete!"

