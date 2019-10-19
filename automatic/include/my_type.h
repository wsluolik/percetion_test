#ifndef MY_TYPE_H
#define MY_TYPE_H
struct obstacle
{
   //障碍物类型
//    enum type
//    {
//        unknown = 0,
//        pedestrian = 1,
//        nonMot = 2,
//        smallMot = 3,
//        bigMot = 4
//    };
   int type;
//    //label的id，感知结果的跟踪id  
   int id;

   //中点坐标
   float x;
   float y;
   float z;

   //长宽高
   float length;
   float width;
   float height;

   //旋转角度
   float pitch;
   float roll;
   float yaw;

};
#endif //MY_TYPE_H