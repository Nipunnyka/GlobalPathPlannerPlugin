
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>


#include "/home/nipunika/path_planner/src/kinoastar.h"


#include <pluginlib/class_list_macros.h>

#define pb push_back
#define ll long long int

float a =0;
float b_x = 0;
float b_y = 0;
float res = 0.05;
float c_x=0;
float c_y = 0;
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(KinoPlanner::KinoPathPlanner, nav_core::BaseGlobalPlanner)

namespace KinoPlanner{
    //Default
    KinoPathPlanner::KinoPathPlanner(){

    }

    KinoPathPlanner::KinoPathPlanner(ros::NodeHandle &nh)
    {
        ROSNodeHandle=nh;
    }
    KinoPathPlanner::KinoPathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }
}

void KinoPathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        costmap_ros_= costmap_ros;
        costmap_=costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/"+name);

        originX=costmap_->getOriginX();
        originY=costmap_->getOriginY();

        resolution=costmap_->getResolution();

        width=costmap_->getSizeInCellsX();
        height=costmap_->getSizeInCellsX();
        mapSize = width*height;

        // may need to enter start pos here

        initialized_=true;
    }
    else{
        ROS_WARN("already initialized.. doing nothing");
    }
}

bool KinoPathPlanner::ROSmakePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_)
    {
        ROS_ERROR("planner has not been initialized!");
        return false;
    }
    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
	plan.clear();

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
		costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
		return false;
	}
    
	tf::Stamped < tf::Pose > goal_tf;
	tf::Stamped < tf::Pose > start_tf;

	poseStampedMsgToTF(goal, goal_tf);
	poseStampedMsgToTF(start, start_tf);

	float startX = start.pose.position.x;
	float startY = start.pose.position.y;

	float goalX = goal.pose.position.x;
	float goalY = goal.pose.position.y;

	getCoordinate(startX, startY);
	getCoordinate(goalX, goalY);

    int startCell;
    int goalCell;
    //converting world float positions to map cell positions
    if(isInsideMap(startX, startY) && isInsideMap(goalX, goalY)){ //REMEMBER: this takes float as arg

        startCell = convertToCellIndex(startX, startY);
        goalCell = convertToCellIndex(goalX, goalY);
    }
    else
    {
        ROS_WARN("the start or goal is out of the map");
        return false;
    }

    	if (isValid(startCell, goalCell)){
		vector<int> bestPath;
		bestPath.clear();
		bestPath = KinoAStar(startCell, goalCell);
		if(bestPath.size()>0){
			for (int i = 0; i < bestPath.size(); i++){
				float x = 0.0;
				float y = 0.0;
				int index = bestPath[i];
				convertToCoordinate(index, x, y);
				geometry_msgs::PoseStamped pose = goal;

				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;

				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = 0.0;
				pose.pose.orientation.w = 1.0;

				plan.push_back(pose);
			}

			float path_length = 0.0;
			std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
			geometry_msgs::PoseStamped last_pose;
			last_pose = *it;
			it++;

			for (; it!=plan.end();++it){
				path_length += hypot((*it).pose.position.x - last_pose.pose.position.x, (*it).pose.position.y - last_pose.pose.position.y );
				last_pose = *it;
			}

			cout <<"The global path length: "<< path_length<< " meters"<<endl;
			return true;
		}
		else{
			ROS_WARN("The planner failed to find a path, choose other goal position");
			return false;
		}
	}
	
	else{
		ROS_WARN("Not valid start or goal");
		return false;
	}
}

vector <int> KinoPathPlanner::getNeighbour (int CellID){
	int rowID=getRow(CellID);
	int colID=getCol(CellID);
	int neighborIndex;
	vector <int>  freeNeighborCells;

	for (int i=-1;i<=1;i++)
		for (int j=-1; j<=1;j++){
			if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
				neighborIndex = getIndex(rowID+i,colID+j);
				if(isFree(neighborIndex) )
					freeNeighborCells.push_back(neighborIndex);
			}
		}

	return  freeNeighborCells;
}

bool KinoPathPlanner::isValid(int startCell,int goalCell){  //CHECK TO WRITE HELPER FUNCTION
	bool isvalid=true;
	
	bool isFreeStartCell=isFree(startCell);
	bool isFreeGoalCell=isFree(goalCell);
	
	if (startCell==goalCell){
		cout << "The Start and the Goal cells are the same..." << endl; 
		isvalid = false;
	}
	
	else{
		
		if(!isFreeStartCell && !isFreeGoalCell){
			cout << "The start and the goal cells are obstacle positions..." << endl;
			isvalid = false;
		}
		
		else{
			if(!isFreeStartCell){
				cout << "The start is an obstacle..." << endl;
				isvalid = false;
			}
			else{
				if(!isFreeGoalCell){
					cout << "The goal cell is an obstacle..." << endl;
					isvalid = false;
				}
				else{
					if (getNeighbour(goalCell).size()==0){
						cout << "The goal cell is encountred by obstacles... "<< endl;
						isvalid = false;
					}
					else{
						if(getNeighbour(startCell).size()==0){
							cout << "The start cell is encountred by obstacles... "<< endl;
							isvalid = false;
						}
					}
				}
			}
		}
	}

return isvalid;
}


}

void KinoPathPlanner::getCoordinate(float& x, float& y){
    x=x-originX;
    y=y-originY;
}


void KinoPathPlanner::convertToCoordinate(int index, float& x, float& y){
	x = getCol(index) * resolution;
	y = getRow(index) * resolution;
	x = x + originX;
	y = y + originY;
}

bool isInsideMap(float x, float y){
    bool valid=true;
    if (x > (width * resolution) || y > (height * resolution))
        valid = false;
  return valid;
}

int KinoPathPlanner::convertToCellIndex( float x, float y){
    int cellIndex;
    float newX = x/resolution;
    float newY= y/resolution;

    cellIndex= getCellIndex(newx, newY);
    return cellIndex;
}

void KinoPathPlanner::call_once(float& vx, float& vy)
{
  boost::shared_ptr<const nav_msgs::Odometry> msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
  vx=(float)msg->twist.twist.linear.x;
  vy=(float)msg->twist.twist.linear.y;
  //cout<<(float)msg->twist.twist.linear.z<<"z->linear"<<"\n";
 return;
}

//WRITE ISFREE FUNC

vector <int> KinoPathPlanner::KinoAStar(int startCell, int goalCell){
    vector<int> bestPath;
    float g_score[mapSize]; //DECLARE MAPSIZE PEHLE

    //manan used  ros::init(argc,argv,"listener"); which I am skipping hoping it doesnt make things go south

    call_once(vstartx, vstarty);
    vgoalx=0; vgoaly=0;

    

}







