std_msgs/Header header

int16 anchor_id
int16 tag_id
int32 range
int32 seq
float64 rss
float64 error_estimation
geometry_msgs/Pose anchor_pose

uint8 los_type
uint8 LOS_TYPE_LOS = 0
uint8 LOS_TYPE_NLOS = 1
uint8 LOS_TYPE_NLOS_SOFT = 2
uint8 LOS_TYPE_NLOS_HARD = 3