#include "ros/ros.h"
#include "iostream"
#include "fstream"
#include "orientation/PerceptionMsg.h"
#include "orientation/PerceptionListMsg.h"
#include <sensor_msgs/Imu.h>
#include "boost/bind.hpp"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
using namespace std;

ofstream ofs_p;
ofstream ofs_i;
void chatterCallback_perception(const orientation::PerceptionListMsg &pm)
{
  int p_sec = pm.header.stamp.sec;
  int p_nsec = pm.header.stamp.nsec;
  for(int i=0;i<pm.perceptions.size();i++)
  {
      if(pm.perceptions[i].tracker_id!=-1)
      {
        ROS_INFO("ID:%d---trackID:%d",pm.perceptions[i].id,pm.perceptions[i].tracker_id);
        float p_y = atan2f(pm.perceptions[i].velocity.y,pm.perceptions[i].velocity.x);
        tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., p_y);
        //geometry_msgs::Quaternion q = quat;
        float perception_yaw = tf::getYaw(quat);
        ofs_p << p_sec <<"  "<< p_nsec <<"  "<< pm.perceptions[i].tracker_id << "  " << pm.perceptions[i].yaw  <<  "  "<< pm.perceptions[i].velocity.x
         << " "<< pm.perceptions[i].velocity.y << " " << pm.perceptions[i].velocity.z << " " << pm.perceptions[i].angular_velocity << " "
          << perception_yaw << endl;
      }
      
    
    
  }
  
};

void chatterCallback_imu(const sensor_msgs::Imu &imu)
{
  int i_sec = imu.header.stamp.sec;
  int i_nsec = imu.header.stamp.nsec;
  geometry_msgs::Quaternion q = imu.orientation;
  float imu_yaw = tf::getYaw(q); 

  ROS_INFO("sec:%d---nsec:%d",i_sec,i_nsec);
  ofs_i << i_sec <<"  "<< i_nsec <<"  " << imu_yaw << endl;

  
};



int main(int argc, char **argv)
{
  ofs_p.open("orientation_data.txt");
  ofs_i.open("imu_data.txt");
  ros::init(argc,argv,"orientation");
  ros::NodeHandle n;
  ros::Rate loop(10);
  ros::Subscriber reve_per = n.subscribe("/rs_percept_result", 1000,chatterCallback_perception);
  ros::Subscriber reve_imu = n.subscribe("/imu", 1000,chatterCallback_imu);
  
  while(ros::ok())
  {
    ros::spinOnce();

    loop.sleep();
  }
  ofs_p.close();
  ofs_i.close();
}
