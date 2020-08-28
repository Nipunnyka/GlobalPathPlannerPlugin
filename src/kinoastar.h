/*
KinoDAStar is supposed to change the vstartx and vstarty speds, you cant control
them from GloabalPlanner class you need to rewrite the DWA local planner for that

also pls check if minx and miny values are corrent in verify_node()
*/

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
	int cellIndex;
	//double cost;

};


namespace KinoPlanner{
  class KinoPathPlanner {
	public:
        KinoPathPlanner();
	      KinoPathPlanner(ros::NodeHandle &nh);
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
        double calc_heuristic(node n1, node n2);
        bool verify_node(node nodec, node parent);
        coordinates KinoDAStar(int startCell, int goalCell);
        vector <int> pathFinder(int startCell, int goalCell);
        vector<int> pathplanner(int startCell, int goalCell);
        void call_once(float& vx, float& vy);
        vector <int> getNeighbour (int CellID);
        bool isValid(int startCell,int goalCell);
        bool isFree(int CellID); //returns true if the cell is Free
        bool isFree(int i, int j);


        bool initialized_;
        float startX, startY; //start pos of rover
        float goalX, goalY; //goal pos of rover
        double originX, originY; //origin of costmap
        double resolution;
        float vstartx, vstarty; //starting velocity in x and y dirn (linear)
        float vgoalx, vgoaly; //goal velocity in x and y dirn (linear)
        float robot_radius;

        class coordinates
         {
            public:
            int cellIndex;
            double cost;
            coordinates(int cellIndex, double cost){
            this->cellIndex=cellIndex;
            this->cost=cost;
        };
        class node
        {
        public:
          coordinates cell;//gcost;
          int p_index;
          double vx,vy;

          node()
          {
        	cell.cellIndex=0; cell.cost=0;
            vx=0;vy=0;
          }
          node(coordinates cell,double vx,double vy, int p_index)
          {
            this->cell = cell;
            this->p_index=p_index;
            this->vx=0;
            this->vy=0;
          }

        };


    };

};
#endif
