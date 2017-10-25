#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include <cmath>

turtlesim::Pose goalPose;
turtlesim::Pose currPose;												// current pose of turtle
turtlesim::Pose mycurrPose;
geometry_msgs::Twist cmdVel;
int nTurtle=0;
double xtot=0, ytot=0;

void currPoseCallback(const turtlesim::Pose::ConstPtr& msg);
void mycurrPoseCallback(const turtlesim::Pose::ConstPtr& msg);

float GetXError(turtlesim::Pose currPoseL, turtlesim::Pose goalPoseL);
float GetThetaError(turtlesim::Pose currPoseL, turtlesim::Pose goalPoseL);

int main(int argc, char **argv){
	ros::init(argc, argv, "my_consensus_node");
	ros::NodeHandle nh;
	
	if (argc != 5){ROS_ERROR("need turtle name and pose as argument"); return -1;};

	ros::service::waitForService("spawn");
	ros::ServiceClient spawnTurtle = nh.serviceClient<turtlesim::Spawn>("spawn");
	
	turtlesim::Spawn::Request req;
	turtlesim::Spawn::Response resp;
	
	req.name	= argv[1];
	req.x 		= std::atoi(argv[2]);
	req.y		= std::atoi(argv[3]);
	req.theta	= std::atoi(argv[4]);

	bool success=spawnTurtle.call(req,resp);
	if(success) { ROS_INFO_STREAM ("Spawned turtle: "); }
	else { ROS_INFO_STREAM ("Error, unable to spawn turtle"); }
	
	ros::param::get("nTurtle",	nTurtle);

	//ros::param::set("~xavg",xavg);
	//ros::param::set("~yavg",yavg);
	
	ros::Rate my_rate(0.2);
	ros::Subscriber curr_pose_sub1 = nh.subscribe("/myturtle1/pose", 1, currPoseCallback);
	ros::Subscriber curr_pose_sub2 = nh.subscribe("/myturtle2/pose", 1, currPoseCallback);
	ros::Subscriber curr_pose_sub3 = nh.subscribe("/myturtle3/pose", 1, currPoseCallback);

	if (ros::ok() && nh.ok())	{	
		usleep(2000000);
		ros::spinOnce();
		usleep(2000000);
	}
	goalPose.x=xtot/nTurtle;
	goalPose.y=ytot/nTurtle;
	goalPose.theta=0;
	ROS_INFO_STREAM("Centroid : (" << goalPose.x << "," << goalPose.y << ")" );
	usleep(2000000);
	
	float ErrorX = 0;
	float ErrorTheta = 0;
	ros::Subscriber my_curr_pose_sub = nh.subscribe("/"+req.name+"/pose", 1, mycurrPoseCallback);
	ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("/"+req.name+"/cmd_vel", 1000);
	
	while(ros::ok() && nh.ok()){
		ros::spinOnce();
		ErrorX		= GetXError(mycurrPose, goalPose);
		ErrorTheta	= GetThetaError(mycurrPose, goalPose);
		ROS_INFO_STREAM("ErrorX: "<<ErrorX <<"	ErrorTheta: " << ErrorTheta);
		if (ErrorX >0) 	{ cmdVel.linear.x 	= 0.2 * ErrorX; }
		else 			{	cmdVel.linear.x 	= 0; }
		cmdVel.angular.z 	= 1.0 * ErrorTheta; 
		turtle_vel_pub.publish(cmdVel);
		//if(abs(ErrorX)<.00000001 && abs(ErrorTheta)<.00000001) ros::shutdown();
	}
}

float GetThetaError (turtlesim::Pose currPoseL, turtlesim::Pose goalPoseL)
{
	float Ex = goalPoseL.x - currPoseL.x;									
	float Ey = goalPoseL.y - currPoseL.y;									
	
	// get desire angle
	float dest = atan2f(Ey, Ex); 										// use float version to get arc tangent
	
	// get angle error
	float Et = dest - currPoseL.theta;
	
	return Et;
}
float GetXError(turtlesim::Pose currPoseL, turtlesim::Pose goalPoseL)
{
	// create error vector
	float Ex = goalPoseL.x - currPoseL.x;									
	float Ey = goalPoseL.y - currPoseL.y;									
	float Et = GetThetaError(currPoseL, goalPoseL);							// get angle between vectors
	
	// project error onto turtle x axis
	float Etx = hypotf(Ex, Ey)*cos(Et); // improved c function
	
	return Etx;
}

void currPoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	currPose.x 		= msg->x;
	currPose.y 		= msg->y;
	currPose.theta 	= msg->theta;
	ROS_INFO_STREAM("Received curr pose: (" << currPose.x << "," << currPose.y << "," << currPose.theta << ")" );
	xtot=xtot+currPose.x;
	ytot=ytot+currPose.y;
	return;
}

void mycurrPoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	mycurrPose.x 		= msg->x;
	mycurrPose.y 		= msg->y;
	mycurrPose.theta 	= msg->theta;
	ROS_INFO_STREAM("Received my curr pose: (" << mycurrPose.x << "," << mycurrPose.y << "," << mycurrPose.theta << ")" );
	return;
}