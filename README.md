# Marker-based SLAM
***
此项目是SPM-SLAM的改进版本，也是一种基于AprilTag或者Aruco类型标记（marker）的SLAM系统，通过在环境中布置不同ID的marker，即可快速实现高精度的相机定位，经过在实验室环境下的测试，可以达到厘米级精度。具体流程可看图1。绿色部分是对SPM-SLAM的修改部分。  
![图1](https://github.com/BIT-wangzb/Marker-based-SLAM/blob/master/image/Fig3-3.tif "图1")
## Papers
SPM-SLAM: Simultaneous Localization and Mapping with quared Planar Markers
## 安装需求
***
### OpenCV
Required at least 3.4, Tested with OpenCV3.4.12

### AprilTag 3
Download and instructions can be found at: [AprilTag3 library](https://github.com/AprilRobotics/apriltag).

### Eigen3
Required by g2o. Required at least 3.1.1.

## Install
***
`mkdir build && cd build`  
`cmake..`  
`make -j4`

## Usage Example
***
执行：`./spm-slam "video" "file1.yml" -conf "file2.yml" --outlog "cameraPose.txt"`  
video: 数据集  
file1.yml: camera parameters  
file2.yml: system parameters
cameraPose.txt: output camera poses.