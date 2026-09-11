# Diagnostic snapshot from the morphing effectiveness provider; not servo feedback.
uint64 timestamp # Snapshot time, microseconds since boot
uint64 timestamp_input # Last input examined (may have been rejected)
uint64 timestamp_angle # Last accepted angle input; zero means default neutral
uint32 matrix_generation # Increments when the provider supplies a new matrix
uint32 rejected_inputs # Invalid or non-increasing timestamp inputs
uint8 INPUT_NONE = 0
uint8 INPUT_ACCEPTED = 1
uint8 INPUT_INVALID = 2
uint8 INPUT_OLD = 3
uint8 input_result # Outcome of last input examined
bool matrix_updated # A new geometry matrix was supplied on this call
bool geometry_valid # False means geometry/matrix fields are NaN
float32[4] input_angle # Last input examined, radians, FR RR RL FL
float32[4] angle # Accepted/assumed angles, radians, FR RR RL FL
float32[3] cog # PX4 forward/right/down, metres from CAD origin
float32[4] ct # Coefficients in MOTOR order FR RL FL RR
float32[4] km # Signed yaw/thrust ratios in MOTOR order
float32[12] rotor_position # MOTOR order, xyz per motor, PX4 FRD metres from CAD origin
float32[24] effectiveness # Raw B, row-major: index=4*row+motor; rows torque xyz, force xyz
