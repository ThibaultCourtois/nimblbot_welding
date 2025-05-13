# Position data
geometry_msgs/Pose pose  		# brut pos/orient format
float64[] xyzwpr			# euler pos/orient format for Telesoud compatibility 

# Robot states
bool robot_in_fault_status
string error_message
bool robot_in_slave_mode_status 	# compatibility purpose only
bool collision_status			# compatibility purpose only
bool emergency_stop			# compatibility purpose only
bool welding_trigger_plc_signal
uint8 operation_mode

# Timestamp
builtin_interfaces/Time timestamp

