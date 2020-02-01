#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <rosbag/bag.h>
#include <stdint.h>
#include <stdlib.h>
#include <rosbag/bag.h>
#include "rosbag/recorder.h"
#include "rosbag/stream.h"
#include <ctime>

using namespace std;
string filenamebag;

bool isRecording(false);
bool isFirst(true);
bool isFinish(false);
rosbag::RecorderOptions options;
  
void cb_start_recording_bag(const std_msgs::String::ConstPtr& msg){
	filenamebag = msg-> data;
	isRecording = true;
	isFirst = true;
	isFinish = false;
}

void cb_stop_recording(const std_msgs::Bool::ConstPtr& msg){
	if (isRecording){
		isFinish = true;
		}
	isRecording = false;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosma2");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);
  ros::Subscriber sub[2];
  sub[0] = nh.subscribe("/rosma/gui/filename",1000, &cb_start_recording_bag);
  sub[1] = nh.subscribe("/rosma/gui/stop_recording",1000, &cb_stop_recording);

int i;
string path = "/media/irivas/TOSHIBA EXT/dvrkData/";
bool stop = false;

options.max_duration = ros::Duration(-1.0);
//options.record_all = true;
options.topics.push_back("/dvrk/MTML/position_cartesian_current");
options.topics.push_back("/dvrk/MTML/twist_body_current");
options.topics.push_back("/dvrk/MTML/gripper_position_current");
options.topics.push_back("/dvrk/MTML/state_joint_current");
options.topics.push_back("/dvrk/MTMR/position_cartesian_current");
options.topics.push_back("/dvrk/MTMR/twist_body_current");
options.topics.push_back("/dvrk/MTMR/wrench_body_current");
options.topics.push_back("/dvrk/MTMR/gripper_position_current");
options.topics.push_back("/dvrk/MTMR/state_joint_current");
options.topics.push_back("/dvrk/PSM1/position_cartesian_current");
options.topics.push_back("/dvrk/PSM1/twist_body_current");
options.topics.push_back("/dvrk/PSM1/wrench_body_current");
options.topics.push_back("/dvrk/PSM1/state_joint_current");
options.topics.push_back("/dvrk/PSM2/position_cartesian_current");
options.topics.push_back("/dvrk/PSM2/twist_body_current");
options.topics.push_back("/dvrk/PSM2/wrench_body_current");
options.topics.push_back("/dvrk/PSM2/state_joint_current");
options.topics.push_back("/rosma/gui/filename");
options.topics.push_back("/rosma/gui/stop_recording");
options.topics.push_back("/dvrk/footpedal/coag");
options.topics.push_back("/usb_cam/image_raw/compressed");

  while(ros::ok())
  {	
	if (isRecording){
		if (isFirst){
			ROS_INFO("Start Recording BAGFILE");
			options.prefix = path + filenamebag;		
			rosbag::Recorder recorder(options); 
			i = recorder.run();	
			isFirst = false;
		}
	}
	
    ros::spinOnce();
    loop_rate.sleep();
  }

  return(0);
}

