
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
        mapSize = width*height;value =0;


		OGM = new bool [mapSize]; 
    	for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    	{	
    	 	for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
    	 	{
    	 	 unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
    	 	 //cout<<cost;
    	 	 if (cost == 0)
    	 	   OGM[iy*width+ix]=true;
    	 	 else
    	 	   OGM[iy*width+ix]=false;
			}
		}	



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
		bestPath = pathplanner(startCell, goalCell);
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
	}for(pppp:: const_iterator i = openset.begin(); i!=openset.end(); ++i)

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

 bool  KinoPathPlanner::isFree(int i, int j){
  	int CellID = getCellIndex(i, j);
 	return OGM[CellID];

 } 

 bool  KinoPathPlanner::isFree(int CellID){
 	return OGM[CellID];
 } 

vector<int> KinoPathPlanner::pathplanner(int startCell, int goalCell){
	vector<int> bestPath;
	bestPath.clear();	
	//manan used  ros::init(argc,argv,"listener"); which I am skipping hoping it doesnt make things go south
	call_once(vstartx, vstarty);
    vgoalx=0; vgoaly=0;

	bestPath=pathFinder(int startCell, goalCell);

	return bestPath;
}

vector <int> KinoPathPlanner::pathFinder(int startCell, int goalCell){
    vector<int> bestPath;
	vector<int> emptyPath;
    //robot_Radius=.105 for burger, .17 for waffle
    robot_radius=0.17;

	OPL.insert(startCell);
	bestPath.push_back(startCell);
	
	cout<<"My startCell::goalCell::vstartx::vstarty::vgoalx::vgoaly::"<<startCell<<"::"<<goalCell<<"::"<<vstartx<<"::"<<vstarty<<"::"<<vgoalx<<"::"<<vgoaly<"/n";
	
	while(1){
		value++;
		coordinates ggkk(67,67);

		ggkk=KinoDAStar(startCell, goalCell);
		//kinodastar needs to return new goalCell and should change vstartx and vstarty
		startCell=ggkk.cellIndex;
		cout<<"go to "<<startCell<<" you will reach with vx and vy "<<vstartx", "<<vstarty<<endl;
		bestPath.push_back(startCell);


		if(startCell==goalCell)
			break;
		
		reviseObstacles(); //this means move to new start Cell, I havent written this function yet IMPORTANT!

	}
	printf("reached the target!");	
	return bestPath;
}

coordinates KinoPathPlanner::KinoDAStar(int startCell, int goalCell){
	coordinates startcooridnate=coordinates(startCell, 0.0);
	coordinates goalcoordinate=coordinates(goalCell, 0.0);
	node nstart=node(startcoordinate, -1, vstartx, vstarty);
	node ngoal=node(goalcoordinate, -1, vgoalx, vgoaly);
	
	typedef std::tr1::unordered_map<int, node> pppp;
	pppp openset;
	pppp closedset;
	node curnode=nstart;
	openset[curnode.cell.cellIndex]=(curnode);
	int testing=0;

	while(testing<2)
	{
		double min_temp=INT_MAX;
		int cid=0;
		for(pppp:: const_iterator i = openset.begin(); i!=openset.end(); ++i)
		{
			double rr=(i->second).cell.cost + calc_heuristic(ngoal, i->second);
			if(rr<min_temp){
				min_temp=rr;
				cid = (i->second).cell.cellIndex;
			}
		}
		curnode=openset[cid];

		vstartx=curnode.vx;
		vstarty=curnode.vy;
		coordinates returnthis=coordinates(curnode.cell.cellIndex, curnode.cell.cost);

		if(curnode.cell.cellIndex==ngoal.cell.cellIndex){
			ngoal.p_index=curnode.p_index;
			ngoal.cost=current.cost;
			break;
		}
		openset.erase(cid);
		closedset[cid]=current;
		for(int vi=-3; vi<=3; vi++) //DEFINE vi vj here
		for(int vj=-3; vj<=3; vj++)
		{
			double cost = pow((pow((vi-current.vx),2)+pow((vj-current.vy),2)),0.5);
			
			float curx=0.0;
			float cury=0.0;
			int index=curnode.cell.cellIndex;
			convertToCoordinate(index, curx,cury);
			int dummyindex=convertToCellIndex(curx + vi*1, cury +vj*1);
			coordinates dummycoord=coordinates(dummyindex, cost);

			node dummy=node(dummycoord, curnode.vx+vi, curnode.vy+vj, cid);

			if(closedset.find(dummyindex)!=closedset.end())
				continue;
			if(verify_node(dummy, curnode))//HELLO write verify node here
				continue;
			if(openset.find(dummyindex)==openset.end())
			{
				openset[dummyindex]=dummy;
			}
			else{
				if(openset[dummyindex].cell.cost>=dummy.cell.cost)
					openset[dummyindex]=dummy;
			}

		}
		testing++;

	}
	
	return returnthis;
}

bool KinoPathPlanner::verify_node(node nodec, node parent)
{
	float tempx, tempy;
	convertToCoordinate(nodec.cell.cellIndex, tempx, tempy);
	if(!isInsideMap(tempx, tempy))
		return 0;
	//check if this is free node
	if(!isFree(nodec.cell.cellIndex))
		return 0;
	
	//I am putting vel x=[-3, 3]
	//and vel y=[-3,3]
	
	if(!((nodec.vx<3 && nodec.vx>-3)|| (nodec.vy>-3 && nodec.vx<3)))
		return 0;
	
	double px, py, curx, cury;
	convertToCoordinate(nodec.cell.cellIndex, curx, cury);
	convertToCoordinate(parent.cell.cellIndex, px, py);

	if(curx==px){
		double smally=min(cury, py);
		double bigy=max(cury,py);

		//assuming origin to be minimum pos and origin+size to be max
		for(int i=smally; i<=(int)bigy; i++){
			//minx=0; miny=0 i have replaced these values for minx and miny through in the original program since we are working im map frame;
			int index=convertToCellIndex(curx, i)
			if(!isFree(index)){
				return 0;
			}
		}
	}
	if(cury==py){
		double smallx=min(curx,px);
		doublebigx=max(curx, px);
		for(int i=smallx; i<=(int)bigx; i++){
			int index=convertToCellIndex(i, cury);
			if(!isfree(index))
				return 0;
		}

		if(cury!=py && curx!=px){
			if(curx>px){
				for(int i=(int)px; i<=(int)curx; i++)
				{
					double y=(((int)curx-px)*(int)((double)i -px)/(int)(cury-py))+py;
					int temp_y=(int)y;
					if(temp_y-y)==0)
					{
						int index=convertToCellIndex(i, temp_y);
						if(!isFree(index)){
							return 0;
						}
					}
				}
			}

		}

		else if(curx<px)
		{
			for(int i=(int)curx; i<=(int)<=px; i++)
				{
					double y=(((int)(px-curx)*(int)((double)i -curx)/(int)(cury-py))+py;
					int temp_y=(int)y;
					if(temp_y-y)==0)
					{
						int index=convertToCellIndex(i, temp_y);
						if(!isFree(index)){
							return 0;
						}
					}
				}
		}

	}
	return 1;

}

double KinoPathPlanner::calc_heuristic(node n1, node n2)
{
    double w = 1.0;  //# weight of heuristic
	float n1x, n2x, n1y, n2y;
	convertToCoordinate(n1.cell.cellIndex, n1x, n1y);
	convertToCoordinates(n2.cell.cellIndex, n2x, n2y);
    double d = w * sqrt((n1x - n2x)*(n1x - n2x) + (n1y - n2y)*(n1y - n2y));
    return d;
}






