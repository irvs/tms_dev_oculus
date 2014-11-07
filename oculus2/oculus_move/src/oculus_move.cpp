//------------------------------------------------------------------------------
// @file   : oculus_move.cpp
// @brief  : use data provided by topic in order to move with Oculus Rift in Rviz
// @author : Alaoui Hassani Atlas Omar
// @version: Ver1.1.0 (since 2014.05.26)
// @date   : 2014.07.10
//------------------------------------------------------------------------------
#include <ros/ros.h>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tms_msg_db/TmsdbStamped.h>

#include <mysql/mysql.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <sstream>
#include <iostream>

//------------------------------------------------------------------------------
#define deg2rad(x)  ((x)*M_PI/180.0)

//------------------------------------------------------------------------------
using std::string;
using namespace std;

//------------------------------------------------------------------------------
//In order to avoid some display gliches if Vicon cannot track Oculus anymore
float x_old = 1;
float y_old = 1;
float z_old = 1.5;

//------------------------------------------------------------------------------
//filter for Vicon angles data
float lpfilter(float data, float filterVal, float filteredVal){

  filteredVal = (data * (1 - filterVal)) + (filteredVal * filterVal);

  return filteredVal;
}

//------------------------------------------------------------------------------
class OculusDb
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  // ROS Topic Subscriber
  ros::Subscriber data_sub;
  // MySQL structure 
  MYSQL *connector;
  // MySQL information
  string dbhost;
  string dbuser;
  string dbpass;
  string dbname;
  string dbdata[100];
  //Using Moverio?
  char choice;

//------------------------------------------------------------------------------
public:
  OculusDb(char choice) : 
    dbhost("192.168.4.170"),
    dbuser("root"),
    dbpass("tmsdb"),
    dbname("rostmsdb"),
    choice(choice)
  {
    //Init Vicon Stream
    ROS_ASSERT(init_oculusdb());
    // Subscriber for tms_db_data topic
    data_sub = nh.subscribe("tms_db_data", 100, &OculusDb::ocMoveCallback, this);
  }  

  //----------------------------------------------------------------------------
  ~OculusDb()
  {
    ROS_ASSERT(shutdown_oculusdb());
    nh.shutdown();
  } 

//------------------------------------------------------------------------------
private:
  bool init_oculusdb()
  {
    //Connection to a MySQL database 
    connector = mysql_init(NULL);
    if (!mysql_real_connect(connector, dbhost.c_str(), dbuser.c_str(), dbpass.c_str(), dbname.c_str(), 3306, NULL, CLIENT_MULTI_STATEMENTS)) 
    {
      fprintf(stderr, "%s\n", mysql_error(connector));
      return false;
    }

    printf("\nMySQL(rostmsdb) opened.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  bool shutdown_oculusdb()
  {
    //Close connection
    mysql_close(connector);
    printf("\nMySQL(rostmsdb) closed.\n");
    return true;
  }

  //----------------------------------------------------------------------------
  // ocMoveCallback function
  void ocMoveCallback(const tms_msg_db::TmsdbStamped::ConstPtr& msg){
    static tf::TransformBroadcaster brOc;

    //Oculus or Moverio
    float x  = 0;
    float y  = 0;
    float z  = 1.5;
    float rr = 0;
    float rp = 0;
    float ry = 0;

    //id of the device used (Moverio or Oculus)
    int id;

    if(choice == 0x01) { id = 1001; } // moverio
    else if (choice == 0x02) { id = 3006; } // Oculus
    else if (choice == 0x04) { id = 3019; } // Oculus DK2
    else std::cerr << "Unexpected choice" << std::endl;

    if( msg->tmsdb[0].id == id ){
      //Get position form the data basis
      x = msg->tmsdb[0].x/1000;
      y = msg->tmsdb[0].y/1000;
      z = msg->tmsdb[0].z/1000; 

      if(choice == 0x01){ // if moverio chosen
        //filter angles' data
        ry = round(lpfilter((msg->tmsdb[0].ry)*.9+(ry*.1), 0.005 , ry));
        rp = round(lpfilter((msg->tmsdb[0].rr)*.9+(rp*.1), 0.5 , rp)); // there is a problem I don't understand in the angles equivalence;

        //reajustment, otherwise, we cannot reach every angle between 0 and 360 degrees
        ry = 180*ry/161;
        if(ry > 0) { rp = -180*rp/80; }
        else {rp = 180*rp/80; }
      }

      //if Vicon cannot track Oculus' markers anymore. In order to avoid some display glitches
      if( (x == 0)&&(y == 0) ){
        x = x_old;
        y = y_old;
        z = z_old;
      }

      //transformation to send to Rviz
      tf::Transform transform;

      transform.setOrigin( tf::Vector3(x, y, z) );

      tf::Quaternion q;
      q.setRPY(rr, deg2rad(rp), deg2rad(ry)); //Mainly for Moverio

      transform.setRotation(q);

      brOc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "oculus_move"));
      brOc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "start_position", "oculus_move"));

      //Update of the data to avoid Vicon gliches
      x_old = x;
      y_old = y;
      z_old = z;     
    }         
  }
};

//------------------------------------------------------------------------------
int main(int argc, char **argv){
  //Open Vicon data stream, servers, and services on virtual terminals (more convinient) 
  //if there are not running on ROS_MASTER server, uncomment
  // system("xterm -T tms_db_manager -e roslaunch tms_db_manager tms_db_manager.launch &");
  // system("xterm -T vicon_stream -e rosrun tms_ss_vicon vicon_stream &");

  //Moverio Or Oculus ?
  char choice_in;
  string choice;
  do{
    cout << "Oculus (O/O2) or Moverio (M)?" << endl;
    getline(cin,choice);
  }while(choice != "O" && choice != "O2" && choice != "M");

  if(choice == "M") { choice_in = 0x01; }
  else if(choice == "O") { choice_in = 0x02; }
  else if(choice == "O2") { choice_in = 0x04; }

  //Path Planning :
  system("xterm -T Path_Planner -e roslaunch tms_rp_rps_planner rps_path_plan.launch &");

  //Virtual Control :
  system("xterm -T Virtual_Control -e rosrun tms_rc_smartpal_virtual_control smartpal_virtual_control &");

  //Tasks :
  system("xterm -T smartpal_simple_task -e rosrun tms_rs_action smartpal_simple_task &");
  system("xterm -T simpletask_service -e rosrun tms_rs_action simpletask_service &");

  //We have to use Choreonoid too !

  puts("\n//----------------------------\n  Click on this terminal\n  before pressing ctrl-C to quit\n----------------------------//");
  
  //Init ROS node
  ros::init(argc, argv, "oculus_move");
  OculusDb od(choice_in);
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF
