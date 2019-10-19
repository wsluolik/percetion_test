# single perception, defined by ros message type

# basic info
float64 g_timestamp # global timestamp in seconds.
int32 id  # obstacle detection ID, only valid for one frame, i.e., will be reinit every frame.
# box info
geometry_msgs/Point center # obstacle geometry center position in global coordinate in meters.
geometry_msgs/Point anchor # stable anchor point of obstacle as the geometry center may varies dramatically since the sparsity of lidar points, you may use it instead of "center".
float32 length # obstacle length in meters, is always >= "width".
float32 width # obstacle width in meters.
float32 height # obstacle height in meters.
geometry_msgs/Vector3 direction # obstacle normalized direction in global coordinate; when without tracking, it parallel to the length edge direction.
float32 yaw # yaw angle in global coordinate with radian unit, another way to depict "direction" if only consider in x-y plane, in global coordinate.

# tracking info
bool is_tracked  # obstacle tracking state: tracked succeed or not, if not, the tracking information is not reliable.
int32 tracker_id  # obstacle tracking ID, valid accross frame sequence all the time.
geometry_msgs/Vector3 velocity # obstacle velocity in global coordinate with unit m/s.
geometry_msgs/Vector3 acceleration # obstacle acceleration in global coordinate with unit m/s^2.
float32 angular_velocity # angular velocity with radian in global coordinate with unit rad/s.
float32 association_score # the confidence for the obstacle when associated to a tracker, 0~1, the greater, the better.
float32 tracker_robustness # the robustness for a tracker that the obstacle belong to, 0~1, the greater, the better.
float64 tracking_time # the total time for the obstacle been tracked since been first detected, including visible or shadowed in seconds.
float64 visible_tracking_time # the time for the obstacle been tracked since been first detected, only for visible frames, in seconds.
geometry_msgs/Vector3[] history_velocity # historical velocity for the obstacle when tracked in global coordinate, default is 15 frames.
int32 track_type # obstacle type inherited from tracking: 0 mmeans unknown, 1 means pedestrain, 2 means bicycle, 3 means car, 4 means truck/bus.

# classification info
int32 type  # obstacle type from classification module: 0 mmeans unknown, 1 means pedestrain, 2 means bicycle, 3 means car, 4 means truck/bus.
float32 type_confidence  # the confidence for the classification from classification module, 0~1, the greater, the better.
bool is_background # obstacle is background or not, the background is always means the big static objects such as buildings.

geometry_msgs/Point[] polygon # corner points of the convex hull of the obstacle in global coordinate in meters.
geometry_msgs/Point[] trajectory # historical trajectory for the obstacle when tracked in global coordinate in meters, default is 15 frames.

# geometry_msgs/Point[] cloud #original pointcloud of the obstacle in global coordinate in meters.
