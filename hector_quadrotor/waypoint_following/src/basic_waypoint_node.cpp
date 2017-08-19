#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "hector_uav_msgs/TakeoffAction.h"
#include "hector_uav_msgs/LandingAction.h"
#include "hector_uav_msgs/PoseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "std_msgs/Float32MultiArray.h" 

#include <vector>
using namespace std;

// Pose Goal message
geometry_msgs::PoseStamped current_goal,current_goal2;
geometry_msgs::Pose tmp_wp;
std::vector<geometry_msgs::Pose> waypoints,waypoints2;
int wp_counter = 0,wp_counter2=0;
double wp_radius = 1;
bool quit_loop = false;
std_msgs::Float32MultiArray rob_pose;
int flag=0;

// Takeoff client (SimpleActionClient from actionlib)
typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseActionClient;

// Callback function for the subscribe() function
void position_cb( const geometry_msgs::PoseStamped::ConstPtr& current_pos ) {
ROS_INFO("Current position: [%f,%f,%f]", current_pos->pose.position.x, current_pos->pose.position.y, current_pos->pose.position.z);
ROS_INFO("Current goal: [%f,%f,%f]", current_goal.pose.position.x, current_goal.pose.position.y, current_goal.pose.position.z);
ROS_INFO("abs( current_pos->pose.position.y - current_goal.pose.position.y ): %f", fabs( current_pos->pose.position.y -current_goal.pose.position.y ));	
	
//If the current position is close to the current goal for X, Y, & Z
	if( fabs( current_pos->pose.position.x - current_goal.pose.position.x ) < wp_radius ) {
		if( fabs( current_pos->pose.position.y - current_goal.pose.position.y ) < wp_radius ) {
			if( fabs( current_pos->pose.position.z - current_goal.pose.position.z ) < wp_radius ) {
				// If there are more waypoints
				wp_counter++;	//Move to the next waypoint
				if( wp_counter < waypoints.size() ) {
					current_goal.pose = waypoints.at(wp_counter);
				} else {
					quit_loop = true;
					ROS_INFO( "Finished the waypoint path!" );
				}
				
			}
		}
	}
}
// void position_cb2( const geometry_msgs::PoseStamped::ConstPtr& current_pos2 ) {
// ROS_INFO("Current position: [%f,%f,%f]", current_pos2->pose.position.x, current_pos2->pose.position.y, current_pos2->pose.position.z);
// ROS_INFO("Current goal: [%f,%f,%f]", current_goal2.pose.position.x, current_goal2.pose.position.y, current_goal2.pose.position.z);
// ROS_INFO("abs( current_pos2->pose.position.y - current_goal2.pose.position.y ): %f", fabs( current_pos2->pose.position.y -current_goal2.pose.position.y ));	
	
// //If the current position is close to the current goal for X, Y, & Z
// 	if( fabs( current_pos2->pose.position.x - current_goal2.pose.position.x ) < wp_radius ) {
// 		if( fabs( current_pos2->pose.position.y - current_goal2.pose.position.y ) < wp_radius ) {
// 			if( fabs( current_pos2->pose.position.z - current_goal2.pose.position.z ) < wp_radius ) {
// 				// If there are more waypoints
// 				wp_counter2++;	//Move to the next waypoint
// 				if( wp_counter2 < waypoints.size() ) {
// 					current_goal2.pose = waypoints2.at(wp_counter2);
// 				} else {
// 					quit_loop = true;
// 					ROS_INFO( "Finished the waypoint path!" );
// 				}
				
// 			}
// 		}
// 	}
// }

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%f]", msg->data(0));
   geometry_msgs::Pose tmp_wp,tmp_wp2;
  	flag=flag+1;
	tmp_wp.orientation.w = 1.0;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	tmp_wp.position.x = msg->data[0]/4;
	tmp_wp.position.y = msg->data[1]/4;
	// cout<<tmp_wp.position.y<<endl;
	tmp_wp.position.z = 21;	//First waypoint is at [0, 0, 5]
	waypoints.push_back(tmp_wp);

	// tmp_wp2.orientation.w = 1.0;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	// tmp_wp2.position.x = 5+msg->data[0]/4;
	// tmp_wp2.position.y = 5+msg->data[1]/4;
	// // cout<<tmp_wp.position.y<<endl;
	// tmp_wp2.position.z = 22;	//First waypoint is at [0, 0, 5]
	// waypoints2.push_back(tmp_wp2);
  flag++;
}

void generate_waypoints() {
	// Local variables
	geometry_msgs::Pose tmp_wp;
	// Processing
	//Waypoint 1
	
	tmp_wp.orientation.w = 1.0;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	tmp_wp.position.x = 20.0;
	tmp_wp.position.y = -20.0;
	tmp_wp.position.z = 20;	//First waypoint is at [0, 0, 5]
	waypoints.push_back(tmp_wp);

	//Waypoint 2
	tmp_wp.position.x = 30.0;	//[1, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 3
	tmp_wp.position.y = 5.0;	//[1, 1.0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 4
	tmp_wp.position.x =2.0;	//[-1, -5, 50]
	tmp_wp.position.y = 1.0;
	 waypoints.push_back(tmp_wp);
	
	//Waypoint 5
	tmp_wp.position.x = -2.0;	//[0, 0, 50]
	tmp_wp.position.y = 1.0;
	waypoints.push_back(tmp_wp);
}

int main(int argc, char **argv) {
	int i=0;
	// ros::init(argc, argv, "listener");
	geometry_msgs::Pose tmp_wp,tmp_wp2;
	// Processing
	//Waypoint 1
	tmp_wp.orientation.w = 1.0;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	tmp_wp.position.x=0.0;
	tmp_wp.position.y=-1.0;
	tmp_wp.position.z = 21;	//First waypoint is at [0, 0, 5]
	waypoints.push_back(tmp_wp);

	// tmp_wp2.orientation.w = 1.0;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	// tmp_wp2.position.x=0.0;
	// tmp_wp2.position.y=1.0;
	// tmp_wp2.position.z = 22;	//First waypoint is at [0, 0, 5]
	// waypoints2.push_back(tmp_wp2);
	//Setup node (must be called before creating variables)
	ros::init( argc, argv, "basic_waypoint2" );
	ros::NodeHandle nh,nh2;
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/robot_traj", 5, chatterCallback);
	ros::Rate loop_rate( 10 );	
	while(flag<5)
	{
		
	ros::spinOnce();
	// loop_rate.sleep();
	}

	// Local variables
	geometry_msgs::PoseStamped HBII0;		// Initial pose in inertial coords. Set to zero. 	
	hector_uav_msgs::LandingGoal landGoal,landGoal2;	
	hector_uav_msgs::PoseGoal poseGoal,poseGoal2;		// PoseGoal object for simpleactionclient PoseActionClient
	ros::Subscriber pos_sub,pos_sub2;
	hector_uav_msgs::TakeoffGoal goal,goal2;		// Goal (empty message) for TakeoffClient



	//Publishers & Subscribers
	// Publish to /position_goal, and Listen to /ground_truth_to_tf/pose (current pose)
	pos_sub = nh.subscribe( "/uav1/ground_truth_to_tf/pose", 1000, position_cb );
	// pos_sub2 = nh2.subscribe( "/uav2/ground_truth_to_tf/pose", 1000, position_cb2 );
	// Generate the waypoints
	// generate_waypoints();
	
			//First waypoint is at [0, 0, 5]
	
		
	//Format output message
	current_goal.header.frame_id = "world";
	current_goal.pose = waypoints.at(wp_counter); //initialize with wp 0

	// current_goal2.header.frame_id = "world";
	// current_goal2.pose = waypoints2.at(wp_counter2); //initialize with wp 0

	//Write something so we know the node is running
	ROS_INFO( "Publishing position goal..." );


	// Initialise the PoseActionClient
	PoseActionClient poc(nh, "/uav1/action/pose"),poc2(nh2, "/uav2/action/pose");
	poc.waitForServer();
	// poc2.waitForServer();
	ROS_INFO("Pose client initialised.");
	
	// Initialise the TakeoffClient
	TakeoffClient toc(nh, "/uav1/action/takeoff");
	toc.waitForServer();

	// TakeoffClient toc2(nh, "/uav2/action/takeoff");
	// toc2.waitForServer();
	ROS_INFO("Takeoff client initialised.");

	
	// Send take-off goal to toc
	toc.sendGoal(goal);
	// toc2.sendGoal(goal2);
	
	// Main while loop
	while ( ros::ok() && !quit_loop ) {
		//Update our message so the receiving node knows it is recent
		
		
		current_goal.header.stamp = ros::Time::now();
		current_goal.header.seq++;
		current_goal.header.frame_id = "world";
		// 

		// current_goal2.header.stamp = ros::Time::now();
		// current_goal2.header.seq++;
		// current_goal2.header.frame_id = "world";
		// // 
	
		current_goal.pose = waypoints.at(wp_counter);
		// Send current goal to pose
		poseGoal.target_pose = current_goal; 
		poc.sendGoal(poseGoal);

		// current_goal2.pose = waypoints2.at(wp_counter2);
		// // Send current goal to pose
		// poseGoal2.target_pose = current_goal2; 
		// poc2.sendGoal(poseGoal2);
		
		//Update subscribers and sleep
		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	
	// Land UAVadvertise
	// Initialise landing client
	LandingClient lnc(nh, "/uav1/action/landing");
	lnc.waitForServer();
	ROS_INFO("Landing client initialised.");

	// LandingClient lnc2(nh, "/uav2/action/landing");
	// lnc2.waitForServer();
	// Set initial inertial pose
	HBII0.header.stamp = ros::Time::now();
	HBII0.header.seq++;
	HBII0.header.frame_id = "world";
	// Set target pose
	landGoal.landing_zone = HBII0;
	// Send land goal
	lnc.sendGoal(landGoal);

	// landGoal2.landing_zone = HBII0;
	// // Send land goal
	// lnc2.sendGoal(landGoal2);


	ros::shutdown();

	return 0;
}

