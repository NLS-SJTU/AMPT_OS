# Active Mapping with Prior Topology
## Introduction
Active mapping unknown environment given a prior topology of the environment starting at uncertain position.

## docs
### requirement
With this repo, you can run the similation experiment of localization and active localization with serveral baseline methods.

If you want to run it with Gazebo, we recommand the following repos for SLAM and similator.
* [slam](https://github.com/icesit/ORB_SLAM2)
* [simulator in gazebo](https://github.com/icesit/p3at_sim)
In addition, you need a navigator(to subscribe the destination of AMPT and reach the destination), an obstacle avoidance module(to provide safety directions), a topology extractor(to provide visited graph).

### start
0. clone this repo to you catkin workspace:
> git clone  
1. catkin_make:
> catkin_make  
2. read param/param.yaml to set the mode;
3. launch:
> roslaunch ampt_os sim_ampt.launch

### Graph data
each line in node data file denotes a node with coordinate xyz(id start from 0).
```
x0 y0 z0
x1 y1 z1
...
```

each line in edge data file denotes edge between two nodes.
```
0 1
0 4
1 4
...
```

### useful topics when you want to run with Gazebo(launch in sim_ampt.launch)
* subscribe topic
1. obstacle/safezone(std_msgs/Float32MultiArray)
  Subscribe from obstacle detection module. Every four float define a safezone (except the last two and the first four, this six are of no use). they are left direction, left dist, right direction, right dist.
2. slam/posecov(geometry_msgs::PoseWithCovarianceStamped)
  Subscribe from SLAM. SLAM pose.
3. topo/savedone(std_msgs/String)
  Subscribe from SLAM. Topomap is saved to this path.(with "/" at the end)
* publish topic
1. point_to_point/path(nav_msgs/Path)
  Publish to navigation module. Set destination in slam frame.
2. slam/totopo(std_msgs/String)
  Publish to SLAM module. Ask slam to save map to this path.(with "/" at the end)
3. /point_to_point/path(nav_msgs::Path)
  Publish a target point in visited graph frame.

## thanks
cpp_solver is from [alsora](https://github.com/alsora/chinese-postman-problem.git).

## author
Wuyang Xue
