
# Collaborative Visual-Inerial SLAM with DeepSDF Prior

## Dependencies
Pre-requisits tested for x86_64:  
- cmake=3.20  
- cuda=11.3  
- ROS melodic  
- Anaconda3 (https://repo.anaconda.com/archive/Anaconda3-2020.07-Linux-x86_64.sh)  
- python=3.7 (no need to preinstall)  
- pytorch=1.10 (no need to preinstall)  
- comma (https://github.com/mission-systems-pty-ltd/comma.git")  
- snark (https://github.com/mission-systems-pty-ltd/snark.git")  
Check file ``environment_cu113.yml`` for more detailed requirements.  
.  
Pre-requisits tested for aarch64:  
- cmake=3.16.3
- cuda=11.4  
- ROS noetic  
- Anaconda3 4.10.1 (https://repo.anaconda.com/archive/Anaconda3-2021.04-Linux-aarch64.sh)  
- python=3.8 (no need to preinstall)  
- pytorch=1.11 (compiled from source against cuda 11.4, no need to preinstall)  
- comma  
- snark  
```
git clone --recursive -b v1.11.0 https://github.com/pytorch/pytorch
```
- torchvision=0.15.1 (compiled from source against cuda 11.4, no need to preinstall)  
```
git clone -b v0.15.1 https://github.com/pytorch/vision.git
```
Check file ``environment_orin_cu114.yml`` for more detailed requirements.  
.  
Additional libraries:  
```
sudo apt-get update
sudo apt-get install libpthread-stubs0-dev build-essential cmake git doxygen libsuitesparse-dev libyaml-cpp-dev libvtk6-dev python3-wstool libomp-dev libglew-dev
sudo apt-get install python3-catkin-tools
```

## Get the source from GitLab
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://gitlab.com/missionsystems/hyperteaming/covins-dsdf.git object_slam
cd object_slam
git lfs pull
```
Before setting up the workspace and running installation files, check that all the required components are recognised:
```
cd ~/catkin_ws
./src/covins-dsdf/check_components.sh
```

## Setup workspace
```
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/melodic/
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Installation
For x86_64:
```
chmod +x src/object_slam/install_linux.sh
./src/object_slam/install_linux.sh 8
```
For aarch64:
```
chmod +x src/object_slam/install_agx.sh
./src/object_slam/install_agx.sh 8
```

### Testing if deep objects works
```
cd orb_slam3
python3 reconstruct_frame.py --config configs/config_kitti.json --sequence_dir data/kitti/07 --frame_id 100
```

### Config files
configs/calib.txt : has the camera(s) projective transformations.  
configs/config_kitti.json : has the network configs used for pointpillars and maskrcnn.  
configs/config_maskrcnn.py : has maskrcnn network architecture.  
configs/hv_pointpillars_secfpn_6x8_160e_kitti-3d-car.py : has pointpillars network architecture.  
configs/orientation.yaml : has vulcan orientation (r,p,q) for hand tuning (testing only).  
configs/ports.yaml : has sensors read ports, and detections write ports.  
configs/vulcan.yaml : has all the vulcan object-slam runtime parameters.  

### Weight files
weights/deepsdf : pretrained deepsdf weights.  
weights/maskrcnn : pretrained maskrcnn weights.  
weights/pointpillars : pretrained pointpillars weights.  
weights/second : pretrained second weights.  

### How To Run mono_snark demo in realtime
Run the listener (port 4003 is for 2d detections)
```
nc -l 4003
```
Play the camera data
```
format=t,3ui,s[15197952]
cat cameras/alvium_1800_forward/20230629T043419.333403.bin | csv-play --binary $format --slow 1 | io-publish tcp:4001 --size $( echo $format | csv-format size )

```
Play the lidar data
```
format=$( ouster-to-csv lidar --output-format )
cat ouster/lidar/20230629T043417.457335.bin | ouster-to-csv lidar --config configs/config.json:ouster | csv-play --binary $format --slow 1 | io-publish tcp:4000 --size $( echo $format | csv-format size )
```
Play the imu data
```
format=$( ms-log-multitool data --include advanced-navigation-imu --output-format )
cat advanced-navigation/20230302T125833.060048.bin | ms-log-multitool data --include advanced-navigation-imu | csv-play --binary $format --slow 1 | io-publish tcp:4002 --size $( echo $format | csv-format size )
```
Run the demo
```
./mono_snark Vocabulary/ORBvoc.bin configs/vulcan.yaml configs

```

### 2d detections
Sent as comma separated for each image:
```
format: d,i,[i,4d,f],...,[i,4d,f]
```
```
timestamp(d), #detections(i), label1(i), bbox1(4d), score1(f), label2(i), bbox2(4d), score2(f), ... , labelN(i), bboxN(4d), scoreN(f)
```

### Documentation
Check Object-Oriented SLAM [confluence page](https://missionsystems.atlassian.net/wiki/spaces/SoftDev/pages/1753808897/Object-Oriented+SLAM)

