#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <cstdlib>

struct sample {
	std::string turtle_name;
	int x, y, theta;
};

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
	req.y		= std::atoi(argv[3]);;
	req.theta	= std::atoi(argv[4]);;

	bool success=spawnTurtle.call(req,resp);
	if(success) { ROS_INFO_STREAM ("Spawned turtle: "); }
	else { ROS_INFO_STREAM ("Error, unable to spawn turtle"); }

}