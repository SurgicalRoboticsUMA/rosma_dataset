#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace std;

ofstream File;
string filename;

float data[168];

bool isRecording(false);
bool isFirst(true);
bool isFinish(false);

// LEFT MASTER TOOL MANIPULATOR
void cb_mtml_position(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  data[0] = msg->pose.position.x;
  data[1] = msg->pose.position.y;
  data[2] = msg->pose.position.z;
  data[3] = msg->pose.orientation.x;
  data[4] = msg->pose.orientation.y;
  data[5] = msg->pose.orientation.z;
  data[6] = msg->pose.orientation.w;
}

void cb_mtml_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  data[7] = msg->twist.linear.x;
  data[8] = msg->twist.linear.y;
  data[9] = msg->twist.linear.z;
  data[10] = msg->twist.angular.x;
  data[11] = msg->twist.angular.y;
  data[12] = msg->twist.angular.z;
}

void cb_mtml_effort(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  data[13] = msg->wrench.force.x;
  data[14] = msg->wrench.force.y;
  data[15] = msg->wrench.force.z;
  data[16] = msg->wrench.torque.x;
  data[17] = msg->wrench.torque.y;
  data[18] = msg->wrench.torque.z;
}


void cb_mtml_gripper(const std_msgs::Float32::ConstPtr& msg)
{
  data[19] = msg->data;
}

void cb_mtml_joint(const sensor_msgs::JointState::ConstPtr& msg)
{
  data[20] = msg->position[0];
  data[21] = msg->position[1];
  data[22] = msg->position[2];
  data[23] = msg->position[3];
  data[24] = msg->position[4];
  data[25] = msg->position[5];
  data[26] = msg->position[6];
  data[27] = msg->position[7];

  data[28] = msg->velocity[0];
  data[29] = msg->velocity[1];
  data[30] = msg->velocity[2];
  data[31] = msg->velocity[3];
  data[32] = msg->velocity[4];
  data[33] = msg->velocity[5];
  data[34] = msg->velocity[6];
  data[35] = msg->velocity[7];

  data[36] = msg->effort[0];
  data[37] = msg->effort[1];
  data[38] = msg->effort[2];
  data[39] = msg->effort[3];
  data[40] = msg->effort[4];
  data[41] = msg->effort[5];
  data[42] = msg->effort[6];
  data[43] = msg->effort[7];
}

// RIGTH MASTER TOOL MANIPULATOR
void cb_mtmr_position(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  data[44] = msg->pose.position.x;
  data[45] = msg->pose.position.y;
  data[46] = msg->pose.position.z;
  data[47] = msg->pose.orientation.x;
  data[48] = msg->pose.orientation.y;
  data[49] = msg->pose.orientation.z;
  data[50] = msg->pose.orientation.w;
}

void cb_mtmr_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  data[51] = msg->twist.linear.x;
  data[52] = msg->twist.linear.y;
  data[53] = msg->twist.linear.z;
  data[54] = msg->twist.angular.x;
  data[55] = msg->twist.angular.y;
  data[56] = msg->twist.angular.z;
}

void cb_mtmr_effort(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  data[57] = msg->wrench.force.x;
  data[58] = msg->wrench.force.y;
  data[59] = msg->wrench.force.z;
  data[60] = msg->wrench.torque.x;
  data[61] = msg->wrench.torque.y;
  data[62] = msg->wrench.torque.z;
}

void cb_mtmr_gripper(const std_msgs::Float32::ConstPtr& msg)
{
  data[63] = msg->data;
}

void cb_mtmr_joint(const sensor_msgs::JointState::ConstPtr& msg)
{
  data[64] = msg->position[0];
  data[65] = msg->position[1];
  data[66] = msg->position[2];
  data[67] = msg->position[3];
  data[68] = msg->position[4];
  data[69] = msg->position[5];
  data[70] = msg->position[6];
  data[71] = msg->position[7];

  data[72] = msg->velocity[0];
  data[73] = msg->velocity[1];
  data[74] = msg->velocity[2];
  data[75] = msg->velocity[3];
  data[76] = msg->velocity[4];
  data[77] = msg->velocity[5];
  data[78] = msg->velocity[6];
  data[79] = msg->velocity[7];

  data[80] = msg->effort[0];
  data[81] = msg->effort[1];
  data[82] = msg->effort[2];
  data[83] = msg->effort[3];
  data[84] = msg->effort[4];
  data[85] = msg->effort[5];
  data[86] = msg->effort[6];
  data[87] = msg->effort[7];
}

// PATIENT SIDE MANIPULATOR 1

void cb_psm1_position(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  data[88] = msg->pose.position.x;
  data[89] = msg->pose.position.y;
  data[90] = msg->pose.position.z;
  data[91] = msg->pose.orientation.x;
  data[92] = msg->pose.orientation.y;
  data[93] = msg->pose.orientation.z;
  data[94] = msg->pose.orientation.w;
}

void cb_psm1_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  data[95] = msg->twist.linear.x;
  data[96] = msg->twist.linear.y;
  data[97] = msg->twist.linear.z;
  data[98] = msg->twist.angular.x;
  data[99] = msg->twist.angular.y;
  data[100] = msg->twist.angular.z;
}

void cb_psm1_effort(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  data[101] = msg->wrench.force.x;
  data[102] = msg->wrench.force.y;
  data[103] = msg->wrench.force.z;
  data[104] = msg->wrench.torque.x;
  data[105] = msg->wrench.torque.y;
  data[106] = msg->wrench.torque.z;
}

void cb_psm1_joint(const sensor_msgs::JointState::ConstPtr& msg)
{
  data[107] = msg->position[0];
  data[108] = msg->position[1];
  data[109] = msg->position[2];
  data[110] = msg->position[3];
  data[111] = msg->position[4];
  data[112] = msg->position[5];
  data[113] = msg->position[6];

  data[114] = msg->velocity[0];
  data[115] = msg->velocity[1];
  data[116] = msg->velocity[2];
  data[117] = msg->velocity[3];
  data[118] = msg->velocity[4];
  data[119] = msg->velocity[5];
  data[120] = msg->velocity[6];

  data[121] = msg->effort[0];
  data[122] = msg->effort[1];
  data[123] = msg->effort[2];
  data[124] = msg->effort[3];
  data[125] = msg->effort[4];
  data[126] = msg->effort[5];
  data[127] = msg->effort[6];
}

// PATIENT SIDE MANIPULATOR 2

void cb_psm2_position(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  data[128] = msg->pose.position.x;
  data[129] = msg->pose.position.y;
  data[130] = msg->pose.position.z;
  data[131] = msg->pose.orientation.x;
  data[132] = msg->pose.orientation.y;
  data[133] = msg->pose.orientation.z;
  data[134] = msg->pose.orientation.w;
}

void cb_psm2_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  data[135] = msg->twist.linear.x;
  data[136] = msg->twist.linear.y;
  data[137] = msg->twist.linear.z;
  data[138] = msg->twist.angular.x;
  data[139] = msg->twist.angular.y;
  data[140] = msg->twist.angular.z;
}

void cb_psm2_effort(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  data[141] = msg->wrench.force.x;
  data[142] = msg->wrench.force.y;
  data[143] = msg->wrench.force.z;
  data[144] = msg->wrench.torque.x;
  data[145] = msg->wrench.torque.y;
  data[146] = msg->wrench.torque.z;
}

void cb_psm2_joint(const sensor_msgs::JointState::ConstPtr& msg)
{
  data[147] = msg->position[0];
  data[148] = msg->position[1];
  data[149] = msg->position[2];
  data[150] = msg->position[3];
  data[151] = msg->position[4];
  data[152] = msg->position[5];
  data[153] = msg->position[6];

  data[154] = msg->velocity[0];
  data[155] = msg->velocity[1];
  data[156] = msg->velocity[2];
  data[157] = msg->velocity[3];
  data[158] = msg->velocity[4];
  data[159] = msg->velocity[5];
  data[160] = msg->velocity[6];

  data[161] = msg->effort[0];
  data[162] = msg->effort[1];
  data[163] = msg->effort[2];
  data[164] = msg->effort[3];
  data[165] = msg->effort[4];
  data[166] = msg->effort[5];
  data[167] = msg->effort[6];
}

void cb_start_recording(const std_msgs::String::ConstPtr& msg){
	filename = msg-> data;
	isRecording = true;
	isFirst = true;
	isFinish = false;
}

void cb_stop_recording(const std_msgs::Bool::ConstPtr& msg){
	if (isRecording){
		isFinish = true;}
	isRecording = false;
}

void OpenRecordingFile(string filename){
	
  File.open(filename.c_str());
  
  File
  << "MTML_position_x" << ", MTML_position_y" << ", MTML_position_z"
  << ", MTML_orientation_x" << ", MTML_orientation_y" << ", MTML_orientation_z"<<", MTML_orientation_w"
  << ", MTML_velocity_linear_x" << ", MTML_velocity_linear_y" << ", MTML_velocity_linear_z"
  << ", MTML_velocity_angular_x" << ", MTML_velocity_angular_y" << ", MTML_velocity_angular_z"
  << ", MTML_wrench_force_x" << ", MTML_wrench_force_y" << ", MTML_wrench_force_z"
  << ", MTML_wrench_torque_x" << ", MTML_wrench_torque_y" << ", MTML_wrench_torque_z"
  << ", MTML_gripper_angle"
  << ", MTML_joint_position_1" << ", MTML_joint_position_2" << ", MTML_joint_position_3" <<", MTML_joint_position_4"
  << ", MTML_joint_position_5" << ", MTML_joint_position_6" << ", MTML_joint_position_7" <<", MTML_joint_position_8"
  << ", MTML_joint_velocity_1" << ", MTML_joint_velocity_2" << ", MTML_joint_velocity_3" <<", MTML_joint_velocity_4"
  << ", MTML_joint_velocity_5" << ",MTML_joint_velocity_6" << ", MTML_joint_velocity_7" <<", MTML_joint_velocity_8"
  << ", MTML_joint_effort_1" << ", MTML_joint_effort_2" << ", MTML_joint_effort_3" <<", MTML_joint_effort_4"
  << ", MTML_joint_effort_5" << ",MTML_joint_effort_6" << ", MTML_joint_effort_7" <<", MTML_joint_effort_8"

  << ", MTMR_position_x" << ", MTMR_position_y" << ", MTMR_position_z"
  << ", MTMR_orientation_x" << ", MTMR_orientation_y" << ", MTMR_orientation_z"<<", MTMR_orientation_w"
  << ", MTMR_velocity_linear_x" << ", MTMR_velocity_linear_y" << ", MTMR_velocity_linear_z"
  << ", MTMR_velocity_angular_x" << ", MTMR_velocity_angular_y" << ", MTMR_velocity_angular_z"
  << ", MTMR_wrench_force_x" << ", MTMR_wrench_force_y" << ", MTMR_wrench_force_z"
  << ", MTMR_wrench_torque_x" << ", MTMR_wrench_torque_y" << ", MTMR_wrench_torque_z"
  << ", MTMR_gripper_angle"
  << ", MTMR_joint_position_1" << ", MTMR_joint_position_2" << ", MTMR_joint_position_3" <<", MTMR_joint_position_4"
  << ", MTMR_joint_position_5" << ", MTMR_joint_position_6" << ", MTMR_joint_position_7" <<", MTMR_joint_position_8"
  << ", MTMR_joint_velocity_1" << ", MTMR_joint_velocity_2" << ", MTMR_joint_velocity_3" <<", MTMR_joint_velocity_4"
  << ", MTMR_joint_velocity_5" << ",MTMR_joint_velocity_6" << ", MTMR_joint_velocity_7" <<", MTMR_joint_velocity_8"
  << ", MTMR_joint_effort_1" << ", MTMR_joint_effort_2" << ", MTMR_joint_effort_3" <<", MTMR_joint_effort_4"
  << ", MTMR_joint_effort_5" << ",MTMR_joint_effort_6" << ", MTMR_joint_effort_7" <<", MTMR_joint_effort_8"

  << ", PSM1_position_x" << ", PSM1_position_y" << ", PSM1_position_z"
  << ", PSM1_orientation_x" << ", PSM1_orientation_y" << ", PSM1_orientation_z"<<", PSM1_orientation_w"
  << ", PSM1_velocity_linear_x" << ", PSM1_velocity_linear_y" << ", PSM1_velocity_linear_z"
  << ", PSM1_velocity_angular_x" << ", PSM1_velocity_angular_y" << ", PSM1_velocity_angular_z"
  << ", PSM1_wrench_force_x" << ", PSM1_wrench_force_y" << ", PSM1_wrench_force_z"
  << ", PSM1_wrench_torque_x" << ", PSM1_wrench_torque_y" << ", PSM1_wrench_torque_z"
  << ", PSM1_joint_position_1" << ", PSM1_joint_position_2" << ", PSM1_joint_position_3" <<", PSM1_joint_position_4"
  << ", PSM1_joint_position_5" << ", PSM1_joint_position_6" << ", PSM1_joint_position_7"
  << ", PSM1_joint_velocity_1" << ", PSM1_joint_velocity_2" << ", PSM1_joint_velocity_3" <<", PSM1_joint_velocity_4"
  << ", PSM1_joint_velocity_5" << ",PSM1_joint_velocity_6" << ", PSM1_joint_velocity_7"
   << ", PSM1_joint_effort_1" << ", PSM1_joint_effort_2" << ", PSM1_joint_effort_3" <<", PSM1_joint_effort_4"
  << ", PSM1_joint_effort_5" << ",PSM1_joint_effort_6" << ", PSM1_joint_effort_7"

  << ", PSM2_position_x" << ", PSM2_position_y" << ", PSM2_position_z"
  << ", PSM2_orientation_x" << ", PSM2_orientation_y" << ", PSM2_orientation_z"<<", PSM2_orientation_w"
  << ", PSM2_velocity_linear_x" << ", PSM2_velocity_linear_y" << ", PSM2_velocity_linear_z"
  << ", PSM2_velocity_angular_x" << ", PSM2_velocity_angular_y" << ", PSM2_velocity_angular_z"
  << ", PSM2_wrench_force_x" << ", PSM2_wrench_force_y" << ", PSM2_wrench_force_z"
  << ", PSM2_wrench_torque_x" << ", PSM2_wrench_torque_y" << ", PSM2_wrench_torque_z"
  << ", PSM2_joint_position_1" << ", PSM2_joint_position_2" << ", PSM2_joint_position_3" <<", PSM2_joint_position_4"
  << ", PSM2_joint_position_5" << ", PSM2_joint_position_6" << ", PSM2_joint_position_7"
  << ", PSM2_joint_velocity_1" << ", PSM2_joint_velocity_2" << ", PSM2_joint_velocity_3" <<", PSM2_joint_velocity_4"
  << ", PSM2_joint_velocity_5" << ",PSM2_joint_velocity_6" << ", PSM2_joint_velocity_7"
   << ", PSM2_joint_effort_1" << ", PSM2_joint_effort_2" << ", PSM2_joint_effort_3" <<", PSM2_joint_effort_4"
  << ", PSM2_joint_effort_5" << ",PSM2_joint_effort_6" << ", PSM2_joint_effort_7" <<", Coag pedal"
  << endl;
}

void RecordingFile (){
	File
    << data[0] <<","<<data[1]<<","<<data[2]<<","<<data[3]<<","<<data[4]
    <<"," << data[5] <<","<<data[6]<<","<<data[7]<<","<<data[8]<<","<<data[9]
    <<"," << data[10] <<","<<data[11]<<","<<data[12]<<","<<data[13]<<","<<data[14]
    <<"," << data[15] <<","<<data[16]<<","<<data[17]<<","<<data[18]<<","<<data[19]
    <<"," << data[20] <<","<<data[21]<<","<<data[22]<<","<<data[23]<<","<<data[24]
    <<"," << data[25] <<","<<data[26]<<","<<data[27]<<","<<data[28]<<","<<data[29]
    <<"," << data[30] <<","<<data[31]<<","<<data[32]<<","<<data[33]<<","<<data[34]
    <<"," << data[35] <<","<<data[36]<<","<<data[37]<<","<<data[38]<<","<<data[39]
    <<"," << data[40] <<","<<data[41]<<","<<data[42]<<","<<data[43]<<","<<data[44]
    <<"," << data[45] <<","<<data[46]<<","<<data[47]<<","<<data[48]<<","<<data[49]
    <<"," << data[50] <<","<<data[51]<<","<<data[52]<<","<<data[53]<<","<<data[54]
    <<"," << data[55] <<","<<data[56]<<","<<data[57]<<","<<data[58]<<","<<data[59]
    <<"," << data[60] <<","<<data[61]<<","<<data[62]<<","<<data[63]<<","<<data[64]
    <<"," << data[65] <<","<<data[66]<<","<<data[67]<<","<<data[68]<<","<<data[69]
    <<"," << data[70] <<","<<data[71]<<","<<data[72]<<","<<data[73]<<","<<data[74]
    <<"," << data[75] <<","<<data[76]<<","<<data[77]<<","<<data[78]<<","<<data[79]
    <<"," << data[80] <<","<<data[81]<<","<<data[82]<<","<<data[83]<<","<<data[84]
    <<"," << data[85] <<","<<data[86]<<","<<data[87]<<","<<data[88]<<","<<data[89]
    <<"," << data[90] <<","<<data[91]<<","<<data[92]<<","<<data[93]<<","<<data[94]
    <<"," << data[95] <<","<<data[96]<<","<<data[97]<<","<<data[98]<<","<<data[99]
    <<"," << data[100] <<","<<data[101]<<","<<data[102]<<","<<data[103]<<","<<data[104]
    <<"," << data[105] <<","<<data[106]<<","<<data[107]<<","<<data[108]<<","<<data[109]
    <<"," << data[110] <<","<<data[111]<<","<<data[112]<<","<<data[113]<<","<<data[114]
    <<"," << data[115] <<","<<data[116]<<","<<data[117]<<","<<data[118]<<","<<data[119]
    <<"," << data[120] <<","<<data[121]<<","<<data[122]<<","<<data[123]<<","<<data[124]
    <<"," << data[125] <<","<<data[126]<<","<<data[127]<<","<<data[128]<<","<<data[129]
    <<"," << data[130] <<","<<data[131]<<","<<data[132]<<","<<data[133]<<","<<data[134]
    <<"," << data[135] <<","<<data[136]<<","<<data[137]<<","<<data[138]<<","<<data[139]
    <<"," << data[140] <<","<<data[141]<<","<<data[142]<<","<<data[143]<<","<<data[144]
    <<"," << data[145] <<","<<data[146]<<","<<data[147]<<","<<data[148]<<","<<data[149]
    <<"," << data[150] <<","<<data[151]<<","<<data[152]<<","<<data[153]<<","<<data[154]
    <<"," << data[155] <<","<<data[156]<<","<<data[157]<<","<<data[158]<<","<<data[159]
    <<"," << data[160] <<","<<data[161]<<","<<data[162]<<","<<data[163]<<","<<data[164]
    <<"," << data[165] <<","<<data[166]<<","<<data[167]<<","<<data[168]
    << endl;	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosma");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  ros::Subscriber sub[20];
  sub[0] = nh.subscribe("/dvrk/MTML/position_cartesian_current",1000, &cb_mtml_position);
  sub[1] = nh.subscribe("/dvrk/MTML/twist_body_current",1000, &cb_mtml_velocity);
  sub[2] = nh.subscribe("/dvrk/MTML/wrench_body_current",1000, &cb_mtml_effort);
  sub[3] = nh.subscribe("/dvrk/MTML/gripper_position_current",1000, &cb_mtml_gripper);
  sub[4] = nh.subscribe("/dvrk/MTML/state_joint_current",1000, &cb_mtml_joint);

  sub[5] = nh.subscribe("/dvrk/MTMR/position_cartesian_current",1000, &cb_mtmr_position);
  sub[6] = nh.subscribe("/dvrk/MTMR/twist_body_current",1000, &cb_mtmr_velocity);
  sub[7] = nh.subscribe("/dvrk/MTMR/wrench_body_current",1000, &cb_mtmr_effort);
  sub[8] = nh.subscribe("/dvrk/MTMR/gripper_position_current",1000, &cb_mtmr_gripper);
  sub[9] = nh.subscribe("/dvrk/MTMR/state_joint_current",1000, &cb_mtmr_joint);

  sub[10] = nh.subscribe("/dvrk/PSM1/position_cartesian_current",1000, &cb_psm1_position);
  sub[11] = nh.subscribe("/dvrk/PSM1/twist_body_current",1000, &cb_psm1_velocity);
  sub[12] = nh.subscribe("/dvrk/PSM1/wrench_body_current",1000, &cb_psm1_effort);
  sub[13] = nh.subscribe("/dvrk/PSM1/state_joint_current",1000, &cb_psm1_joint);

  sub[14] = nh.subscribe("/dvrk/PSM2/position_cartesian_current",1000, &cb_psm2_position);
  sub[15] = nh.subscribe("/dvrk/PSM2/twist_body_current",1000, &cb_psm2_velocity);
  sub[16] = nh.subscribe("/dvrk/PSM2/wrench_body_current",1000, &cb_psm2_effort);
  sub[17] = nh.subscribe("/dvrk/PSM2/state_joint_current",1000, &cb_psm2_joint);
  
  sub[18] = nh.subscribe("/rosma/gui/filename",1000, &cb_start_recording);
  sub[19] = nh.subscribe("/rosma/gui/stop_recording",1000, &cb_stop_recording);

  
  while(ros::ok())
  {
	  
	if (isRecording){
		if (isFirst){
			ROS_INFO("Start Recording");
			OpenRecordingFile(filename);
			isFirst = false;
			}
			
		RecordingFile();
	}
	
	if (isFinish){
		ROS_INFO("Stop Recording");
		File.close();
		isFinish = false;
	}
	
    ros::spinOnce();
    loop_rate.sleep();
  }

  return(0);
}
