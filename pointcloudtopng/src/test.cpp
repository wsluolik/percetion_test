#include "ros/ros.h"
#include "iostream"
#include "fstream"
#include "pointcloudtopng/PerceptionMsg.h"
#include "pointcloudtopng/PerceptionListMsg.h"
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/image_encodings.h> 
#include <opencv2/imgproc/imgproc.hpp>

#include <unistd.h>
//#include <direct.h>
#define w 500

using namespace std;
using namespace cv;

std::string png_folder_path = "/home/chen/box_test/";

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

char atom_window[] = "Drawing num: track";
int frames_count = 0;

void callback(const pointcloudtopng::PerceptionListMsg::ConstPtr &pm,const sensor_msgs::PointCloud2::ConstPtr &points)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*points,*p_cloud);
    stringstream p_num;
    p_num << pm ->header.seq;
    
    pcl::CropBox<pcl::PointXYZRGB> crop;
    cout << "objetc num:" << pm->perceptions.size() << endl;

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr save_points(new pcl::PointCloud<pcl::PointXYZRGB>());

    Eigen::Vector4f b_min;
    Eigen::Vector4f b_max;

    for(int i=0;i< pm->perceptions.size();i++)
    {
        if(pm->perceptions[i].tracker_id!=-1 &&(pm->perceptions[i].type==3 || pm->perceptions[i].type==4))
        {
            Eigen::Vector3f dir = Eigen::Vector3f(0.0,0.0,pm->perceptions[i].yaw);
            const Eigen::Vector3f& center = Eigen::Vector3f(pm->perceptions[i].center.x,pm->perceptions[i].center.y,pm->perceptions[i].center.z);
            const Eigen::Vector3f& size = Eigen::Vector3f(pm->perceptions[i].length,pm->perceptions[i].width,pm->perceptions[i].height);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr save_points(new pcl::PointCloud<pcl::PointXYZRGB>);

            //根据长宽高得到盒子的两个顶点
            b_min = Eigen::Vector4f(-size(0)/2,-size(1)/2,-size(2)/2,1.0);
            b_max = Eigen::Vector4f(size(0)/2,size(1)/2,size(2)/2,1.0);

            //提取

            crop.setMin(Eigen::Vector4f(b_min)); //
            crop.setMax(Eigen::Vector4f(b_max)); //对角点，官方文档说明是最小和最大点

            crop.setTranslation(center); //坐标偏移
            crop.setRotation(dir);//旋转角度，roll pitch yaw
            
            crop.setInputCloud(p_cloud);
            crop.setKeepOrganized(false); 
            crop.filter(*save_points); 

            

            
            if(save_points -> points.size()>0)
            {
                

                //把点云转成png并画框
                 PointCloud::Ptr MergeCloud(new PointCloud);
                 for (std::size_t k = 0; k < save_points->points.size(); k += 1)
                 {
                    pcl::PointXYZRGB p;
                    p.x = save_points->points[k].x;
                    p.y = save_points->points[k].y;
                    p.z = 0;
                    p.r = save_points->points[k].r;
                    p.g = save_points->points[k].g;
                    p.b = save_points->points[k].b;
                    MergeCloud->points.push_back(p);
                 }

                MergeCloud->height = 1;
                MergeCloud->width = MergeCloud->points.size();
                MergeCloud->is_dense = true;

                Mat atom_image = Mat::zeros( w, w, CV_8UC3 );
                //Eigen:Vector3f = offset = Eigen:Vector3f(w/2.0,-w/2.0,0.0);
                int x = 0,y = 0;
                for (std::size_t n = 0; n < MergeCloud->points.size(); n += 1)
                {
                    //std::cout << "x: " << MergeCloud->points[n].x << std::endl;
                    //std::cout << "y: " << MergeCloud->points[n].y << std::endl;
                    // x = abs(100 - MergeCloud->points[n].x*10);
                    // y = abs(100 + MergeCloud->points[n].y*10);
                    x = (MergeCloud->points[n].x - center(0))*40 + w/2;
                    y = (MergeCloud->points[n].y- center(1))*40 + w/2;
                                    //像素加粗
                atom_image.at<Vec3b>(x-1, y-1) = Vec3b(MergeCloud->points[n].b, MergeCloud->points[n].g, MergeCloud->points[n].r);
                atom_image.at<Vec3b>(x, y-1) = Vec3b(MergeCloud->points[n].b, MergeCloud->points[n].g, MergeCloud->points[n].r);
                atom_image.at<Vec3b>(x+1, y-1) = Vec3b(MergeCloud->points[n].b, MergeCloud->points[n].g, MergeCloud->points[n].r);
                atom_image.at<Vec3b>(x-1, y) = Vec3b(MergeCloud->points[n].b, MergeCloud->points[n].g, MergeCloud->points[n].r);

                atom_image.at<Vec3b>(x+1, y) = Vec3b(MergeCloud->points[n].b, MergeCloud->points[n].g, MergeCloud->points[n].r);
                atom_image.at<Vec3b>(x-1, y+1) = Vec3b(MergeCloud->points[n].b, MergeCloud->points[n].g, MergeCloud->points[n].r);
                atom_image.at<Vec3b>(x, y+1) = Vec3b(MergeCloud->points[n].b, MergeCloud->points[n].g, MergeCloud->points[n].r);
                atom_image.at<Vec3b>(x+1, y+1) = Vec3b(MergeCloud->points[n].b, MergeCloud->points[n].g, MergeCloud->points[n].r);


                }

                
                // std::string save_local_path = png_folder_path+png_name+"_";
                // std::string local_frame_num = local_num.str();
                // local_frame_num.append(".png");
                // save_local_path.append(local_frame_num);
                
            //  int l_point_x = b_min(0) * 10 + w/2;
            //  int l_point_y = b_min(1)  * 10 + w/2;

            //  int r_point_x = b_max(0)  * 10 + w/2;
            //  int r_point_y = b_max(1)  * 10 + w/2;
            //画矩形
            float box_angle = 90 - pm ->perceptions[i].yaw * 57.3;

            RotatedRect rRect = RotatedRect(Point2f(w/2,w/2), Size2f(size(0)*40,size(1)*40),box_angle);

            Point2f vertices[4]; //定义4个点的数组
             rRect.points(vertices); //将四个点存储到vertices数组中 
             for (int i = 0; i < 4; i++) // 注意Scala中存储顺序 BGR 
             {
                line(atom_image, vertices[i],vertices[(i+1)%4], Scalar(0,255,0)); // 返回外接矩形 
             }
             
              //Rect brect = rRect.boundingRect();
               //rectangle(atom_image, brect, Scalar(255,0,0));

            
            
            //  rectangle(atom_image,
            //      Point(l_point_x, l_point_y ),
            //      Point(r_point_x, r_point_y ),
            //      Scalar( 0, 0, 255 ),
            //      1,
            //      LINE_8);
                
            //  //旋转
            //   Mat dst; float angle = -(pm->perceptions[i].yaw*180/pi); 
            //   Size src_sz = atom_image.size();
            //   Size dst_sz(src_sz.height, src_sz.width);
            //   cv::Point2f turn_center(w / 2.0, w / 2.0); //获取旋转矩阵（2x3矩阵） 
            //   Mat rot_mat = getRotationMatrix2D(turn_center, angle, 1.0); //根据旋转矩阵进行仿射变换
            //   warpAffine(atom_image, dst, rot_mat, dst_sz);
            std::ostringstream local_num;
            local_num << pm->perceptions[i].tracker_id;
            string png_path = png_folder_path + local_num.str() + "/" + p_num.str() + ".png";

            string folderPath = png_folder_path + local_num.str() + "/";
            if (0 != access(folderPath.c_str(), 0))
            { // if this folder not exist, create a new one. 
                 if(mkdir(folderPath.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)!=0)
                 {
                     cout << "error folder!!" << folderPath << endl;
                 }
                 //换成 ::_mkdir  ::_access 也行，不知道什么意思 
            }


                imwrite(png_path, atom_image);

            }
            


            // std::cout << "b_min: " << b_min<< std::endl;
            // std::cout << "b_max: " << b_max << std::endl;

            // int l_point_x = abs(100 - b_min(0));
            // int l_point_y = abs(100 + b_min(1));

            // int r_point_x = abs(100 - b_max(0));
            // int r_point_y = abs(100 + b_max(1));
            
            // rectangle(atom_image,
            //     Point(l_point_x, l_point_y ),
            //     Point(r_point_x, r_point_y ),
            //     Scalar( 0, 255, 255 ),
            //     1,
            //     LINE_8);
            
            // imwrite(save_local_path, atom_image);
            // cout << "save:" << save_local_path << endl;
        }
    }

    
}

int main(int argc, char **argv)
{
    std::cout << "point cloud to png begin !" << std::endl;
    ros::init(argc,argv,"pointcloudtopng");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string pointcloud_topic = "/cluster";
    std::string results_save_path;
    std::string folder_path;  

    // private_nh.param<std::string>("results_save_path", results_save_path, ros::package::getPath("pointcloudtopng") + "/data/");
    // //private_nh.param<std::string>("png_name", png_name, "");

    // time_t t = time(0);  // get time now
    // struct tm* now = localtime(&t);
    // char buffer[80];
    // strftime(buffer, 80, "%Y%m%d-%H%M", now);

    // folder_path = results_save_path + std::string(buffer) + "/";

    // boost::filesystem::path p(folder_path);
    // boost::filesystem::create_directories(p);
    
    // png_folder_path = folder_path + "imge/";
    // boost::filesystem::path p1(png_folder_path);
    // boost::filesystem::create_directories(p1);

    message_filters::Subscriber<pointcloudtopng::PerceptionListMsg> perception_sub(nh, "/rs_percept_result", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub(nh, "/cluster", 1000);

    typedef message_filters::sync_policies::ApproximateTime<pointcloudtopng::PerceptionListMsg, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100000), perception_sub, point_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::MultiThreadedSpinner s(4);  //多线程
    ros::spin(s);


    return 0;
}
