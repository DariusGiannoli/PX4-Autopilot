# Coherent EKF2 health, published from the same update as vehicle_odometry.
# Flags below are project protocol bits, independent of EKF internal bit positions.
uint64 timestamp
uint64 timestamp_sample
uint64 fusion_horizon_us
uint64 ev_pos_last_fuse
uint64 ev_hgt_last_fuse
uint64 ev_yaw_last_fuse
uint32 control_flags
uint32 fault_flags
uint32 innovation_flags
float32 heading_variance
uint16 solution_flags
uint8 estimator_instance
uint8 ev_pose_frame
uint8 reset_counter
uint8 xy_reset_counter
uint8 z_reset_counter
uint8 vxy_reset_counter
uint8 vz_reset_counter
uint8 heading_reset_counter
bool heading_good_for_control
