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

# Read-only startup diagnosis. These fields are ULog/uORB only; they do not
# extend or change the existing MHE1 MAVLink health payload.
int32 effective_ev_ctrl
int32 effective_ev_qmin
int32 effective_hgt_ref
# Settings used by the last EV-pop check, which can predate current settings.
int32 ev_startup_ev_ctrl
int32 ev_startup_ev_qmin
int32 ev_startup_hgt_ref
uint64 ev_startup_checked_at_us
uint64 ev_startup_sample_us
uint64 ev_startup_previous_sample_us
uint64 ev_startup_newest_sample_us
uint64 ev_startup_buffer_push_us
# Failed common-start checks: current quality=1, sample gap=2,
# previous quality=4, newest quality=8, recent buffer push=16.
# checked_at=0 means no EV sample has reached this check yet, not a pass.
uint8 ev_startup_failed_checks
