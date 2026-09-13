# Command-to-angle bridge health; does not confirm physical servo positions.
uint64 timestamp
uint64 timestamp_command # Last accepted command with at least one arm value
uint64 timestamp_angle # Last successfully published arm-state timestamp
uint32 published_messages
uint32 queue_overruns # Lost command queue entries invalidate partial arm state
uint32 rejected_commands
uint8 initialized_mask # Bits 0..3: FR RR RL FL known; 15 means complete
bool enabled
bool calibration_valid
bool stale # Timeout or disabled; allocator keeps its last valid geometry
