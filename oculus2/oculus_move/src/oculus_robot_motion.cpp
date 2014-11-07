//------------------------------------------------------------------------------
// @file   : oculus_robot_motion.cpp
// @brief  : Service client for oculus_move.cpp. Allows a motion simulation for
//           SmartPal5 with joints management.
// @author : Alaoui Hassani Atlas Omar
// @version: Ver1.0.0 (since 2014.06.04)
// @date   : 2014.06.18
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <time.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>
#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_rc/smartpal_control.h>
#include <tms_msg_rc/robot_tts.h>
#include <tms_msg_rs/rs_task.h>
#include <tms_msg_rs/robot_task.h>
#include <tms_msg_db/Tmsdb.h>
#include <sensor_msgs/JointState.h>

#include <mysql/mysql.h>

#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <sstream>
#include <iostream>

//------------------------------------------------------------------------------
#define deg2rad(x)  ((x)*M_PI/180.0)

//------------------------------------------------------------------------------
using std::string;
using namespace std;

//------------------------------------------------------------------------------
vector<string> joint;

//Robot position initialization
float rPosX = 1.0;
float rPosY = 1.0;
float Y = 0.0;

//For Shaking Hands
int handShake = 0;

//For joint and position initialization
int init_joint = 0;
int init_pos   = 0;

//Global Task
string task = "";

//------------------------------------------------------------------------------
int main(int argc, char **argv){
  //Init ROS node
  ros::init(argc, argv, "oculus_robot_motion");
  //ros::start();

  // ROS NodeHandle
  ros::NodeHandle nh;
  // ROS Service Client
  ros::ServiceClient get_data_client;
  ros::ServiceClient get_task_client;
  ros::ServiceClient control_real_sp5;
  ros::ServiceClient sp5_tts;
  // Client for services
  get_data_client  = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  get_task_client  = nh.serviceClient<tms_msg_rs::rs_task>("rs_task");
  control_real_sp5 = nh.serviceClient<tms_msg_rc::smartpal_control>("sp5_control");
  sp5_tts          = nh.serviceClient<tms_msg_rc::robot_tts>("smartpal5_tts");
  //Enable to publish joint states
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 0);

  //--------------------------------------------------------------------------

  while(ros::ok()){
    //Get information from rp_move service
    tms_msg_db::TmsdbGetData getRobotData;

    //Get information from rs_tasl
    tms_msg_rs::rs_task getTask;

    //Write information in joint_states topic
    sensor_msgs::JointState joint_state;
        
    // 2002 -> SmartPal 5
    getTask.request.id = 2002;
    getRobotData.request.tmsdb.id = 2002;

    //3005 -> fake | 3001 -> real
    getRobotData.request.tmsdb.sensor = 3005; 

    //Calling the services
    get_task_client.call(getTask);

    if (!(get_data_client.call(getRobotData))) {
      cout << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
    }

    else if (getRobotData.response.tmsdb.empty()==true) {
      cout << "[TmsAction] nothing on floor (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
    }

    else {
      ros::Rate loop_rate(50);

      //Get User's task for the robot
      task = getTask.response.data.task;

      //Get data from the data basis
      if (getRobotData.response.tmsdb[0].state==1) {
        joint.clear();
        string val;
        val = getRobotData.response.tmsdb[0].joint;
        boost::split(joint, val, boost::is_any_of(";"));

        rPosX = getRobotData.response.tmsdb[0].x/1000;
        rPosY = getRobotData.response.tmsdb[0].y/1000;
        Y = deg2rad(getRobotData.response.tmsdb[0].ry);
      }

      vector<double> flJoint(joint.size());
      for(size_t i = 0; i < joint.size(); ++i) {
          flJoint[i] = atof(joint[i].c_str());
      }

      if(task == "incline"){
        flJoint[1] = 35.0;
      }
      else if(task == "shake hand"){
        flJoint[4] = -20.0;
        flJoint[5] = 90.0;
        flJoint[9] = 10;
        handShake++;
      }
      else if(handShake%2 == 1){
        flJoint[4] = -20.0;
        flJoint[5] = 95.0;
        flJoint[9] = 8;
        handShake++;
      }
      else if(handShake%2 == 0 && handShake != 0 && handShake < 6){
        flJoint[4] = -20.0;
        flJoint[5] = 90.0;
        flJoint[9] = 8;
        handShake++;
      }
      else if(handShake == 6){
        flJoint[4] = -20.0;
        flJoint[5] = 90.0;
        flJoint[9] = 10;
        handShake++;
      }

      //--------------------------------------------------------------------------
      //Virtual control
      //Rviz state change :
      joint_state.header.stamp = ros::Time::now();

      joint_state.name.resize(24);
      joint_state.name[0] = "world_x_joint";
      joint_state.name[1] = "world_y_joint";
      joint_state.name[2] = "world_theta_joint";
      joint_state.name[3] = "vehicle__lumbar_j1_joint";
      joint_state.name[4] = "lumbar_j1__lumbar_j2_joint";
      joint_state.name[5] = "lumbar_j2__head_camera_joint";
      joint_state.name[6] = "lumbar_j2__leftArm_j1_joint";
      joint_state.name[7] = "leftArm_j1__leftArm_j2_joint";
      joint_state.name[8] = "leftArm_j2__leftArm_j3_joint";
      joint_state.name[9] = "leftArm_j3__leftArm_j4_joint";
      joint_state.name[10] = "leftArm_j4__leftArm_j5_joint";
      joint_state.name[11] = "leftArm_j5__leftArm_j6_joint";
      joint_state.name[12] = "leftArm_j6__leftArm_j7_joint";
      joint_state.name[13] = "leftArm_j7__leftGripper_joint";
      joint_state.name[14] = "leftGripper__leftGripper_thumb_joint";
      joint_state.name[15] = "lumbar_j2__rightArm_j1_joint";
      joint_state.name[16] = "rightArm_j1__rightArm_j2_joint";
      joint_state.name[17] = "rightArm_j2__rightArm_j3_joint";
      joint_state.name[18] = "rightArm_j3__rightArm_j4_joint";
      joint_state.name[19] = "rightArm_j4__rightArm_j5_joint";
      joint_state.name[20] = "rightArm_j5__rightArm_j6_joint";
      joint_state.name[21] = "rightArm_j6__rightArm_j7_joint";
      joint_state.name[22] = "rightArm_j7__rightGripper_joint";
      joint_state.name[23] = "rightGripper__rightGripper_thumb_joint";

      joint_state.position.resize(24);

      //Make SmartPal5 move in Rviz
      //vehicle
      joint_state.position[0] = rPosX;
      joint_state.position[1] = rPosY;
      joint_state.position[2] = Y;

      //lumbar
      joint_state.position[3] = deg2rad(flJoint[0]);
      joint_state.position[4] = deg2rad(flJoint[1]);

      //unused joints
      joint_state.position[5] = 0.0;
      joint_state.position[13] = 0.0;
      joint_state.position[22] = 0.0;
      
      //Arms
      for (int i = 2; i<9; i++){
        //Right
        joint_state.position[i+13] = deg2rad(flJoint[i]);
        //Left
        joint_state.position[i+4] = deg2rad(flJoint[i+8]);
      }

      //Grippers
      //Left
      joint_state.position[14] = -deg2rad(flJoint[17]);
      //Right
      joint_state.position[23] = deg2rad(flJoint[9]);

      //publish the state
      joint_pub.publish(joint_state);

      //End Rviz state change

      //--------------------------------------------------------------------------
      //Real Robot Control
        //Init Position to vicon data :
        if(init_pos == 0){
          bool init_ok = true;
          tms_msg_rc::smartpal_control sp5Control;
          tms_msg_rc::robot_tts sp5Tts;

          //Sometimes, there is a problem with Tts, please check before using this node.

          //State initialization
          sp5Tts.request.text = "Please wait.";
          if(!(sp5_tts.call(sp5Tts))) {
            cout << "[SpTts] Failed to call service sp5_tts" << endl;
          }

          //Finding position
          sp5Control.request.unit = 0;
          sp5Control.request.cmd  = 7;
          if(!(control_real_sp5.call(sp5Control))) {
            cout << "[SpControl] Failed to call service sp5_control...\n / ! \\ Please restart" << endl;
            init_ok = false;
          }

          if(init_ok){
            sp5Tts.request.text = "I am ready. Let's go!";
            if(!(sp5_tts.call(sp5Tts))) {
              cout << "[SpTts] Failed to call service sp5_tts" << endl;
            }
            init_pos = 1;
          }
          else{
            sp5Tts.request.text = "There is a problem. I cannot find my current position... Please restart my program!";
            if(!(sp5_tts.call(sp5Tts))) {
              cout << "[SpTts] Failed to call service sp5_tts" << endl;
            }
          }
        }

        //Init Joint Position
        if(init_joint == 0){
          tms_msg_rc::smartpal_control sp5Control;

          sp5Control.request.unit = 2; //Right Arm
          sp5Control.request.cmd  = 15; //Move Joints

          sp5Control.request.arg.resize(8);

          for(int i = 0; i<7; i++){
            sp5Control.request.arg[i] = 0;
          }

          sp5Control.request.arg[1] = -5;
          sp5Control.request.arg[0] = 0;
          sp5Control.request.arg[7] = 25; //velocity

          if(!(control_real_sp5.call(sp5Control))) {
            cout << "[SpControl] Failed to call service sp5_control" << endl;
          }

          sp5Control.request.unit = 3; //Left Arm

          if(!(control_real_sp5.call(sp5Control))) {
            cout << "[SpControl] Failed to call service sp5_control" << endl;
          }
          
          //End of init
          init_joint = 1;
        }

        // Shake Hand
        if(handShake != 0 && handShake <3){
          tms_msg_rc::smartpal_control sp5Control;
          tms_msg_rc::robot_tts sp5Tts;

          if(handShake != 0){
            sp5Tts.request.text = "Hello. My name is Smart Pal Five. Nice to meet you!";
            sp5Control.request.unit = 2; //Right Arm
            sp5Control.request.cmd  = 15; //Move Joints

            sp5Control.request.arg.resize(8);

            for(int i = 0; i<7; i++){
              sp5Control.request.arg[i] = flJoint[i+2];
            }

            sp5Control.request.arg[1] = -5;
            sp5Control.request.arg[0] = 0;
            sp5Control.request.arg[7] = 25;

            if(!(control_real_sp5.call(sp5Control))) {
              cout << "[SpControl] Failed to call service sp5_control" << endl;
            }

            if(handShake == 1){
              if(!(sp5_tts.call(sp5Tts))) {
                cout << "[SpTts] Failed to call service sp5_tts" << endl;
              }
            }
          }
        }
        else if(handShake == 4){
          tms_msg_rc::smartpal_control sp5Control;

          sp5Control.request.unit = 2; //Right Arm
          sp5Control.request.cmd  = 15; //Move Joints

          sp5Control.request.arg.resize(8);

          for(int i = 0; i<7; i++){
            sp5Control.request.arg[i] = 0;
          }

          sp5Control.request.arg[1] = -5;
          sp5Control.request.arg[0] = 0;
          sp5Control.request.arg[7] = 25; //velocity

          if(!(control_real_sp5.call(sp5Control))) {
            cout << "[SpControl] Failed to call service sp5_control" << endl;
          }
        }

      //--------------------------------------------------------------------------

      if(task == "incline"){
        ros::Duration(1).sleep();
      }
      else if(handShake != 0){
        ros::Duration(0.2).sleep();
        if(handShake == 7){ handShake = 0; }
      }

      loop_rate.sleep();
    }
  }

  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF