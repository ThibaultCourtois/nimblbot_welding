uint8 command_type        	# telesoud instruction code
uint8 COMMAND_STOP=0
uint8 COMMAND_GET_ROBOT_DATA=1
uint8 COMMAND_SET_DYNAMIC=7
uint8 COMMAND_START_DYNAMIC=8
uint8 COMMAND_PLAY_CARTESIAN=15
uint8 COMMAND_PLAY_JOINT=16

geometry_msgs/Pose target_pose   
float64 speed                   
geometry_msgs/Twist speed_vector 

uint32 command_id		# command identifier
