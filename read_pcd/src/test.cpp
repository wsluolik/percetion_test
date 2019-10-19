#include "ros/ros.h"
#include "iostream"
#include "fstream"
#include "read_pcd/PerceptionMsg.h"
#include "read_pcd/PerceptionListMsg.h"
#include <sensor_msgs/PointCloud2.h>
#include "boost/bind.hpp"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
using namespace std;



void chatterCallback_perception(const read_pcd::PerceptionListMsg &pm)
{
  int p_sec = pm.header.stamp.sec;
  int p_nsec = pm.header.stamp.nsec;
  stringstream p_num;
  p_num << pm.header.seq;
  ofstream ofs_p;
  string p_label_name = "/home/chen/ws/percetion_test/p_label/" + p_num.str() + ".txt";
  ofs_p.open(p_label_name);
   ROS_INFO("num:%d------size:%ld",pm.header.seq,pm.perceptions.size());
  for(int i=0;i<pm.perceptions.size();i++)
  {
        string p_type;
        if(pm.perceptions[i].type == 1)
        {
          p_type = "pedestrian";
        }
        else if(pm.perceptions[i].type == 2)
        {
          p_type = "nonMot";
        }
        else if(pm.perceptions[i].type == 3)
        {
          p_type = "smallMot";
        }
        else if(pm.perceptions[i].type == 4)
        {
          p_type = "bigMot";
        }
        else
        {
          p_type = "unknown";
        }
        ofs_p << p_type << " " << pm.perceptions[i].tracker_id << " " 
      << pm.perceptions[i].center.x << " " << pm.perceptions[i].center.y  <<  "  " << pm.perceptions[i].center.z << " "
      << pm.perceptions[i].center.x << " " << pm.perceptions[i].center.y  <<  "  " << pm.perceptions[i].center.z << " ";
  }
      
      ofs_p.close();
    
  }
  



int main(int argc, char **argv)
{


  ros::init(argc,argv,"orientation");
  ros::NodeHandle n;
  ros::Rate loop(10);
  ros::Subscriber reve_per = n.subscribe("/rs_percept_result", 10,chatterCallback_perception);

  ros::Publisher r_pcd = n.advertise<sensor_msgs::PointCloud2>("/rslidar_points", 100);
  sleep(3);
  //sleep(1);
  for(int i=1;i<201;i++)
  {
    if(ros::isShuttingDown()){
      return 0;
    }
    std::string r_path;
    
    std::ostringstream i_num;
    i_num << i;
    r_path +="/home/chen/ws/percetion_test/pcd/pingshancunshizilukou_";
    r_path = r_path + i_num.str() + ".pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr read_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(r_path,*read_cloud)==-1)
    {
      cout << "path error" << endl;
      return -1;
    }
      ros::Time newtime = ros::Time::now();

      sensor_msgs::PointCloud2 rp;
      pcl::toROSMsg(*read_cloud,rp);
      rp.header.frame_id = std::string("rslidar"); 
      rp.header.stamp = newtime;

      //rp.header.seq = callbackL;

      

    r_pcd.publish(rp);

    ros::spinOnce();

    loop.sleep();

  }

  while(ros::ok())
  {
    ros::spinOnce();
  }
  

}
