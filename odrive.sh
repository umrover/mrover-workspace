# !/bin/bash

echo "starting odrives"
LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec jetson/odrive_bridge 0 &
LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec jetson/odrive_bridge 1 &
LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec jetson/odrive_bridge 2 &
