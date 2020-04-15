// Including general libraries
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <set>
#include "bits/stdc++.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

#include <pluginlib/class_list_macros.h>

// Including ROS specific libraries
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h> //will NEED TO CHANGE THIS MAY
#include <move_base_msgs/MoveBaseAction.h>

// To accomodate for moving base
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <angles/angles.h>

// To accomodate sensory input
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

// Navigation messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

// Costmap transform
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// To get costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

// Defining the whole thing in a namespace
using namespace std;
using std::string;

#ifndef KinoPathPlanner
#define KinoPathPlanner

struct cells {
	int currentCell;
	float fCost;

};

namespace KinoPlanner{
    class KinoPathPlanner : public nav_core::BaseGlobalPlanner{
        public:
        KinoPathPlanner();
        KinoPathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        // Overriden classes from interface nav_core::BaseGlobalPlanner
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
        
        //Base Functions for Planner
        bool isInsideMap(float x, float y);
        int convertToCellIndex(float x, float y);
        int getCellIndex(int i, int j){
            return (i*width) + j;
        }

        int getIndex(int i,int j){return (i*width)+j;}
        int getRow(int index){return index/width;}
        int getCol(int index){return index%width;}

        bool initialized_;
        float startX, startY; //start pos of rover
        float goalX, goalY; //goal pos of rover
        double originX, originY; //origin of costmap
        double resolution;
        float vstartx, vstarty; //starting velocity in x and y dirn (linear)
        float vgoalx, vgoaly; //goal velocity in x and y dirn (linear)

    }
}