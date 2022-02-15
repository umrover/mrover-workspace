for CAM_NUM in {0..1}
do
    gst-launch-1.0 udpsrc port=500${CAM_NUM} ! \"application/x-rtp, encoding-name=(string)H264, payload=96\" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
done
for CAM_NUM in {3..8}
do
    gst-launch-1.0 udpsrc port=500${CAM_NUM} ! \"application/x-rtp, encoding-name=(string)H264, payload=96\" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
done