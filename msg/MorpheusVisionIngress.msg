# SD diagnostic for a decoded strict vision ODOMETRY packet, at the timestamp gate.
uint64 timestamp                 # FC boot time at timestamp conversion
uint64 remote_timestamp_us       # original host capture timestamp from the packet
uint64 mapped_timestamp_us        # zero on rejection
uint64 clock_uncertainty_us       # UINT64_MAX means unknown
uint64 capture_uncertainty_us     # UINT64_MAX means conversion failed before computing this
uint64 clock_age_us               # UINT64_MAX means unknown
uint32 clock_generation
uint16 clock_samples
uint8 clock_fault                # BoundedVisionClock::Fault
uint8 decision
uint8 ACCEPTED = 0
uint8 CLOCK_NOT_READY = 1
uint8 INVALID_REMOTE_STAMP = 2
uint8 TIMESTAMP_ARITHMETIC = 3
uint8 CAPTURE_UNCERTAINTY_EXCEEDED = 4
uint8 FUTURE_CAPTURE = 5
uint8 mavlink_sequence
uint8 mavlink_channel
bool clock_ready
uint8 ORB_QUEUE_LENGTH = 16
