# Matrix actually assigned to the allocator after failure masking and weak-row removal.
uint64 timestamp # Assignment time, microseconds since boot
uint64 timestamp_angle # Accepted angle input associated with this assignment
uint32 matrix_generation # Join to morphing_geometry.matrix_generation
uint16 handled_motor_failure_mask # PX4 motor-order bits
float32[24] effectiveness # Assigned B (not inverse/normalized mixer), row-major 6x4; MOTOR order FR RL FL RR
