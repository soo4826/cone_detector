# Cone Detector for track mission

## Cone Detection Process
1. Filter ground point using `ray_ground_filter` and generate `no_ground` pointcloud
2. Cluster `no_ground` pointcloud and detect conme
3. Classify cone's color based on cone's location
4. Visualize Cone's Location, Intensity, Color using ROS markerarray

## Installation
```sh
$ git clone <this repo>
$ catkin_make
$ source devel.setup.bash
$ rosrun velodyne_cone_detectodyne_cone_detector_node
```
## Node Info
- input topic: `/velodyne_points`
- output topic: `/cones_Intensity`, `/cones_location` `/conesSorted`