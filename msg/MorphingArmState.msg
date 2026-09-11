# Accepted arm angles published by the servo-driving code for control allocation.
# These are commanded/assumed positions, not measured feedback or raw PWM values.
uint64 timestamp # Monotonic publication time, microseconds since boot
float32[4] angle # Radians from CAD neutral, positive CAD +Z; FR, RR, RL, FL; +/- pi/6
