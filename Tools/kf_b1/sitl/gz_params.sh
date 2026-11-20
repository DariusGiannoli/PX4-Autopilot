#!/bin/bash
# Sets the Gazebo x500_vision instance (instance 1) up for the kf_b1 SITL test; restart PX4 afterwards.
P="${PX4_BUILD:-$(cd "$(dirname "$0")/../../.." && pwd)/build/px4_sitl_default}/bin/px4-param --instance 1"
set_p() { $P set "$1" "$2" >/dev/null && echo "  $1 = $2"; }

echo "estimator: external vision only"
set_p SENS_EN_GPSSIM 0
set_p SENS_EN_MAGSIM 0
set_p SYS_HAS_MAG 0
set_p SYS_HAS_BARO 0
set_p EKF2_GPS_CTRL 0
set_p EKF2_BARO_CTRL 0
set_p EKF2_MAG_TYPE 5
set_p EKF2_EV_CTRL 15
set_p EKF2_HGT_REF 3
set_p EKF2_EV_DELAY 0

echo "x500: thrust curve, hover thrust, motor lag"
set_p THR_MDL_FAC 0.67
set_p MPC_THR_HOVER 0.6
set_p KFB1_TAU_M 0.02

echo "no RC, offboard loss -> land, high-rate logging"
set_p COM_RC_IN_MODE 4
set_p COM_OF_LOSS_T 1.0
set_p COM_OBL_RC_ACT 4
set_p SDLOG_PROFILE 17

echo "prior B1 about 15 % off the x500 (4.15 / 68.7 / 68.7 / 3.42)"
for kv in 00:4.8 01:4.8 02:4.8 03:4.8 10:80 11:80 12:-80 13:-80 20:-60 21:60 22:60 23:-60 30:2.9 31:-2.9 32:2.9 33:-2.9; do
    set_p KFB1_G0_${kv%%:*} ${kv#*:}
done

$P save >/dev/null && echo "saved; restart PX4 (shutdown, then start again)"
