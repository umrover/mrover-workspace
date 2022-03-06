import subprocess

ports = [3, 4, 5, 6, 7, 8, 9, 10]
pipeline = [None, None, None, None, None, None, None, None]

for i, val in enumerate(ports):
    cmd = "gst-launch-1.0 udpsrc port=500" + str(val) + " ! \"application/x-rtp, encoding-name=(string)H264, payload=96\" ! rtph264depay ! decodebin ! videoconvert ! autovideosink"
    pipeline[i] = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)

while True:
    pass
