# LiDAR GNSS mapping 
## Framework
![image](https://github.com/ZhuangYanDLUT/lidar_gnss_mapping/blob/master/lidar_gnss_mapping/imgs/framework.png)  
The overall system framework for large-scale 3D map building in partially GNSS-denied scenes, which consists of two operating modes: `LiDAR-only mode` and `LiDAR-GNSS mode`. While working in GNSS-denied scenes, LiDAR odometry runs with high frequency and outputs estimations with local registration errors, and LiDAR mapping provides more accurate pose estimations with low frequency.  
While moving from GNSS-denied scenes to open scenes, 3D-M-Box is working in `LiDAR-GNSS mode`. The auto coordinate alignment algorithm is applied to align the coordinate system between LiDAR and GNSS by registering a group of poses obtained from LiDAR mapping and GNSS within a certain time window. Finally, GNSS-constrained LiDAR mapping outputs the pose and point clouds with high accuracy.  
## Sample map
![image](https://github.com/ZhuangYanDLUT/lidar_gnss_mapping/blob/master/lidar_gnss_mapping/imgs/Campus3.png)  
The picture shows that 3D-M-Box can accomplish online pose estimation and map building in GNSS-denied scenes.  
## Dependency  
**ROS** (tested with kinetic)  
**gtsam** (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)  
```
wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip   
cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/    
mkdir build && cd build    
cmake ..   
sudo make install    
```
## How to build with catkin    
```
cd ~/catkin_ws/src  
git clone https://github.com/ZhuangYanDLUT/lidar_gnss_mapping.git  
cd ~/catkin_ws  
catkin_make  
```
## Running
`roslaunch lidar_gnss_mapping lidar_gnss_mapping.launch`  
In second terminal play sample data from test.bag  
`rosbag play test.bag`  

	