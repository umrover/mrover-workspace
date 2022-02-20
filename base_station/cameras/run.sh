#!/usr/bin/env bash
for CAM_NUM_0 in {0..1}
do
	gst-launch-1.0 udpsrc port=500${CAM_NUM_0} ! "application/x-rtp, encoding-name=(string)H264, payload=96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
done
for CAM_NUM_1 in {3..8}
do
	gst-launch-1.0 udpsrc port=500${CAM_NUM_1} ! "application/x-rtp, encoding-name=(string)H264, payload=96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
done
