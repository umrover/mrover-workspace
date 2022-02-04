FROM stereolabs/zed:3.6-gl-devel-cuda11.4-ubuntu18.04
RUN apt-get update
RUN set -x && \
    echo ">> Adding build-dependencies..." && \
    sed -i 's/# \(.*multiverse$\)/\1/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get -y upgrade
RUN apt-get install -y build-essential \
    software-properties-common \
    apt-transport-https \
    git \
    zsh \
    wget
# Install Ansible from PPA
RUN apt-add-repository ppa:ansible/ansible && \
    apt-get update && \
    apt-get install -y ansible && \
    echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections
RUN apt-get install -y libglib2.0-dev
# Build LCM from source
RUN wget -O lcm.zip https://github.com/lcm-proj/lcm/releases/download/v1.4.0/lcm-1.4.0.zip && \
    unzip lcm.zip && \
    rm lcm.zip && \
    cd lcm-1.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make all -j && \
    make install
RUN apt-get install -y libglm-dev libglfw3-dev libglew-dev x11-apps \
    python3-venv pkg-config autoconf dh-autoreconf libgtk2.0-dev python3-dev libgirepository1.0-dev rapidjson-dev
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y build-essential unzip cmake ninja-build \
    libgtk-3-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
    python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev jasper libdc1394-22-dev \
    libopencv-contrib-dev libopencv-dev
RUN ldconfig
