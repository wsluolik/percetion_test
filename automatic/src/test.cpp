#include "ros/ros.h"
#include "iostream"
#include "fstream"
#include <sensor_msgs/PointCloud2.h>
#include "boost/bind.hpp"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <time.h>
#include "tools/tools.h"
using namespace std;

int main(int argc, char **argv)
{




  ros::init(argc,argv,"automatic");
  ros::NodeHandle n;
  ros::Rate loop(1);

  ros::Publisher r_pcd = n.advertise<sensor_msgs::PointCloud2>("/rslidar_points", 10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
  
  std::string pcd_path;
  std::string label_path;

  tools my_tool;
  for(int l=1;l<200;l++)
  {
    if(ros::isShuttingDown()){
      return 0;
    }
    stringstream l_num;
    l_num << l;
    pcd_path = "/home/chen/ws/percetion_test/pcd/pingshancunshizilukou_"+l_num.str()+".pcd";
    label_path = "/home/chen/ws/percetion_test/label/pingshancunshizilukou_"+l_num.str()+".txt";
  
    pcl::PointCloud<pcl::PointXYZI>::Ptr read_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path,*read_cloud)==-1)
    {
      cout << "path error" << endl;
      return -1;
    }

    sensor_msgs::PointCloud2 p_cloud;
    pcl::toROSMsg(*read_cloud,p_cloud);
    p_cloud.header.frame_id = std::string("rslidar"); 
   


    visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
    vector<obstacle> label_array;
    
    if(my_tool.label_to_struct(label_array,label_path)!=0)
    {
      return -1;
    }
    cout << label_array.size() << endl;
    if(my_tool.draw_cube(marker_array,label_array)!=0)
    {
      return -1;
    }


    r_pcd.publish(p_cloud);
    marker_pub.publish(marker_array);
    ros::spinOnce();
    loop.sleep();

  }


}
