#ifndef TOOLS_H
#define TOOLS_H
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "fstream"
#include "iostream"
#include "my_type.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
using namespace std;
class tools
{
   public:
   int draw_cube(visualization_msgs::MarkerArrayPtr &array,vector<obstacle> label_array);
   int label_to_struct(vector<obstacle> &label_array,std::string lable_path);
};
#endif //TOOLS_H