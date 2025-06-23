#!/bin/bash

# Define the Unicode Characters
CHECK_MARK=✔️
CROSS_MARK=❌

# Function to check the availability and print the version of a command
check_and_print_version() {
    command -v "$1" &> /dev/null
    if [ $? -eq 0 ]; then
        echo -e "${CHECK_MARK} $2 is available."
        eval "$3"
    else
        echo -e "${CROSS_MARK} Error: $2 is NOT available."
    fi
}

# Check and print the version of cmake
check_and_print_version "cmake" "cmake" "cmake --version"

# Check and print the version of conda (from Anaconda3)
check_and_print_version "conda" "Anaconda3" "conda --version"

# Check and print the version of nvcc (from CUDA)
check_and_print_version "nvcc" "nvcc (Cuda compiler)" "nvcc --version"

# Check for CUDA library and print the version using nvcc
if ldconfig -p | grep -q "libcudart"; then
    echo -e "${CHECK_MARK} CUDA Library is available."
    nvcc --version
else
    echo -e "${CROSS_MARK} Error: CUDA Library is NOT available."
fi

# Check for TensorRT library
if ldconfig -p | grep -q "libnvinfer"; then
    echo -e "${CHECK_MARK} TensorRT is available."
    echo "For TensorRT version, please refer to the documentation or the system where it was installed."
else
    echo -e "${CROSS_MARK} Error: TensorRT is NOT available."
fi

# Check for ROS and print the version
if [ -n "$ROS_DISTRO" ]; then
    echo -e "${CHECK_MARK} ROS ($ROS_DISTRO) is available."
    echo "For detailed ROS version, please refer to the environment variable ROS_DISTRO or the system where it was installed."
else
    echo -e "${CROSS_MARK} Error: ROS is NOT available."
fi

# Check for catkin and print the version
#check_and_print_version "catkin_make" "catkin" "catkin_make --version"
check_and_print_version "catkin_make" "catkin" #"echo \"catkin_make is available.\""

# Check and print the version of comma (from Anaconda3)
check_and_print_version "csv-play" "comma" #"csv-play --version"

# Check and print the version of snark (from Anaconda3)
#check_and_print_version "cv-cat" "snark" "conda --version"
