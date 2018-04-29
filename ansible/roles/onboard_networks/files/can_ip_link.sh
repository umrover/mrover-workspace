#!/bin/sh
ip link set can0 type can bitrate 1000000
ip link set can0 txqueuelen 1000
ip link set up can0
nvpmodel -m 0
echo 255 > /sys/kernel/debug/tegra_fan/target_pwm
