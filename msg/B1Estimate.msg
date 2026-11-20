# kf_b1 estimate of B1 (nu = B1 u): rows [a_z, alpha_x, alpha_y, alpha_z] in body FLU,
# columns = rotors [FL, RL, RR, FR] = actuator_motors.control[2, 1, 3, 0].

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # timestamp of the vehicle_angular_velocity sample this estimate includes (microseconds)

float32[16] b1            # row-major 4x4

bool valid                # true while the filter is updating (initialised, armed, not landed)

uint8 INIT_PRIOR = 0      # filter started from the prior G0 (KFB1_G0_*, at arming)
uint8 INIT_THROW = 1      # filter restarted from the batch fit on the throw_sysid pulses
uint8 init_source
