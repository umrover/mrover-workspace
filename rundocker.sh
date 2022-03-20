xhost local:root && \
docker run -it --env="DISPLAY" \
  --volume=${HOME}:/mnt/ \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --privileged --net=host \
  umrover1/travis:latest