xhost local:root
docker run -it --env="DISPLAY" \
  --volume=/home/${USER}/:/mnt/ \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --privileged \
  percep:latest /bin/zsh
