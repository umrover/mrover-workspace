for CAM_NUM in {0..3}
do
    /usr/local/bin/video-viewer /dev/video${CAM_NUM} rtp://10.0.0.1:500${CAM_NUM} --headless
done
