# GlobalPathPlannerPlugin
Path Planner Plugin for turtlebot3 using KinoDynamic A Star in ROS melodic

##what to do from here?
1. registering the plugin and making the respective launch file for gazebo (follow http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)
2. implementing the dwa local planner to control the velocity as per the one instructed by the algo. the required vel is changes in vstartx and vstart y by KinoDAStar() function in this plugin but we need to ensure that this velocity isactually used by local planner.

##References I followed:
-https://github.com/coins-lab/relaxed_astar/blob/master/src/RAstar_ros.cpp
-https://www.youtube.com/watch?v=t4A_niNlDdg
-http://docs.ros.org/hydro/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html#ac3b11af1fdb061507442faef13f53c48
