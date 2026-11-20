# throw_sysid hover pulses: status, and the per-motor offsets that control_allocator adds to its output.
# control_allocator applies offset only in PHASE_EXCITE and only while this message is fresh (< 20 ms).

uint64 timestamp            # time since system start (microseconds)

uint8 PHASE_IDLE = 0
uint8 PHASE_EXCITE = 1      # pulses running (kf_b1 fits B1 here)
uint8 PHASE_DONE = 2

uint8 phase
float32[4] offset           # added to actuator_motors.control[0..3] (actuator_motors order)
uint8 aborted_motors        # bit i: motor i's pulse was cut by the rate margin
uint8 end_reason            # why the pulses ended

uint8 END_NONE = 0
uint8 END_COMPLETE = 1      # all 4 motors pulsed
uint8 END_DROP = 2          # dropped more than THRSID_MAX_DROP
uint8 END_RATE = 3          # body rate above THRSID_RATE_MAX
uint8 END_DISARMED = 4
uint8 END_CANCEL = 5        # 'throw_sysid cancel'
uint8 END_TILT = 6          # tilt above THRSID_TILT_MAX
