#include "tools/tools.h"

int  tools::draw_cube(visualization_msgs::MarkerArrayPtr &marker_array,vector<obstacle> label_array)
{
    uint32_t shape = visualization_msgs::Marker::CUBE;
    
    for(int i=0;i<label_array.size();i++)
    {
        obstacle ob;
        ob = label_array[i];
      
        visualization_msgs::Marker marker;
        marker.header.frame_id = "rslidar";
        marker.header.stamp = ros::Time::now();
    
        marker.ns = "test";
        marker.id = i;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = ob.x;
        marker.pose.position.y = ob.y;
        marker.pose.position.z = ob.z;
    
        // marker.pose.orientation.x = 0.0;
        // marker.pose.orientation.y = 0.0;
        // marker.pose.orientation.z = 0.0;
        // marker.pose.orientation.w = 1.0;

        tf::Quaternion quat = tf::createQuaternionFromYaw(ob.yaw);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = ob.length;
        marker.scale.y = ob.width;
        marker.scale.z = ob.height;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.5;

        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1);

        marker_array -> markers.push_back(marker);

        
    }
     
    return 0;
    

    //取消自动删除
    //marker.lifetime = ros::Duration(0.01);
}

int  tools::label_to_struct(vector<obstacle> &label_array,std::string label_path)
{
   ifstream ifs_label;
    ifs_label.open(label_path);
    if(!ifs_label)
    {
        cout << "open label:" << label_path << " error!";
        return -1;
    }
    string line;
    

    while(getline(ifs_label,line))
    {
        obstacle ob;
        stringstream input(line);
        string s_type;
        input >>  s_type;
        if(s_type=="smallMot")
        {
           ob.type = 3;
        }
        else if(s_type=="pedestrian")
        {
           ob.type = 1;
        }
        else if(s_type=="bigMot")
        {
           ob.type = 4; 
        }
        else if(s_type=="nonMot")
        {
            ob.type = 2;
        }
        else if(s_type=="unknown")
        {
           ob.type = 0;
        }
        else
        {
          cout << "label value error!" << endl;
          return -1;
        }

        input >> ob.id;
        input >> ob.x;
        input >> ob.y;
        input >> ob.z;
        input >> ob.length;
        input >> ob.width;
         input >> ob.height;
        input >> ob.pitch;
        input >> ob.roll;
        input >> ob.yaw;
   
       label_array.push_back(ob);
    }

    // for(int i=0;i<label_array.size();i++)
    // {
    //   obstacle ob;
    //   ob = label_array[i];
    //   cout << ob.type << "  " << ob.id << " "
    //   << ob.x << "  "<< ob.y << " " << ob.z << "  "
    //   << ob.length << "  "<< ob.width << " " << ob.height << "  "
    //   << ob.pitch << "  "<< ob.roll << " " << ob.yaw << "  "
    //   << endl;
    // }
    return 0;
}