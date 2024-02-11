geometry_msgs/Point32 pos_start             # start position of trajectory
geometry_msgs/Point32 pos_end               # end position of trajectory
float32 trajectory_delta_time               # total time of travel
int8 task_type                             # task type, see configuration file
int8 is_joint_velocity_profile              # add a quintic velocity profile in joint space
int8 is_trajectory_absolute_coordinates     # pos_end as relative or absolute coordinates