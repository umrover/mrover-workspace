FROM ubuntu:18.04

#Setup Ansible
ENV ANSIBLE_VERSION 2.9.4
ENV DEBIAN_FRONTEND noninteractive

COPY ansible /tmp/ansible

ENV TZ=UTC-4
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN set -x && \
    echo ">> Adding build-dependencies..." && \
    sed -i 's/# \(.*multiverse$\)/\1/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get -y upgrade && \
    apt-get install -y build-essential \
    software-properties-common \
    apt-transport-https \
    git \
    wget \
    && apt-add-repository ppa:ansible/ansible && \
    apt-get update && \
    apt-get install -y ansible && \
    echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections

RUN ansible-playbook -i "localhost," -c local /tmp/ansible/devbox.yml

ENV ANSIBLE_GATHERING smart
ENV ANSIBLE_HOST_KEY_CHECKING false
ENV ANSIBLE_RETRY_FILES_ENABLED false
ENV ANSIBLE_ROLES_PATH /ansible/playbooks/roles
ENV ANSIBLE_SSH_PIPELINING True
ENV PYTHONPATH /ansible/lib
ENV PATH /ansible/bin:$PATH
ENV ANSIBLE_LIBRARY /ansible/library

WORKDIR /root

ENTRYPOINT ["/bin/bash"]

# Install OpenCV 3.2.0
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get autoremove -y && \
    apt-get install -y --no-install-recommends cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev \ 
    libswscale-dev python3.6-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff5-dev jasper \
    libdc1394-22-dev libeigen3-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common libtbb-dev \
    yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenexr-dev libgstreamer-plugins-base1.0-dev \
    libavutil-dev libavfilter-dev libavresample-dev

RUN apt-get install -y gcc

RUN cd /opt && \
    wget --no-check-certificate -O opencv-3.2.0.tar.gz https://github.com/opencv/opencv/archive/3.2.0.tar.gz && \
    tar -xzf opencv-3.2.0.tar.gz && \
    wget --no-check-certificate -O opencv_contrib-3.2.0.tar.gz https://github.com/opencv/opencv_contrib/archive/3.2.0.tar.gz && \
    tar -xzf opencv_contrib-3.2.0.tar.gz && \
    rm opencv-3.2.0.tar.gz && \
    rm opencv_contrib-3.2.0.tar.gz && \
    cd opencv-3.2.0 && \
    mkdir release && \
    cd release && \
    cmake -D WITH_CUDA=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-3.2.0/modules -D BUILD_opencv_aruco=ON -D BUILD_opencv_bgsegm=OFF \
    -D BUILD_opencv_bioinspired=OFF -D BUILD_opencv_ccalib=OFF -D BUILD_opencv_cnn_3dobj=OFF -D BUILD_opencv_cvv=OFF \
    -D BUILD_opencv_datasets=OFF -D BUILD_opencv_dnn_objdetect=OFF -D BUILD_opencv_dnn_superres=OFF \
    -D BUILD_opencv_dnns_easily_fooled=OFF -D BUILD_opencv_dpm=OFF -D BUILD_opencv_face=OFF -D BUILD_opencv_fuzzy=OFF \
    -D BUILD_opencv_freetype=OFF -D BUILD_opencv_hdf=OFF -D BUILD_opencv_line_descriptor=OFF -D BUILD_opencv_matlab=OFF \
    -D BUILD_opencv_optflow=OFF -D BUILD_opencv_ovis=OFF -D BUILD_opencv_plot=OFF -D BUILD_opencv_reg=OFF \
    -D BUILD_opencv_rgbd=OFF -D BUILD_opencv_saliency=OFF -D BUILD_opencv_sfm=OFF -D BUILD_opencv_stereo=OFF \
    -D BUILD_opencv_structured_light=OFF -D BUILD_opencv_surface_matching=OFF -D BUILD_opencv_text=OFF \
    -D BUILD_opencv_tracking=OFF -D BUILD_opencv_xfeatures2d=OFF -D BUILD_opencv_ximgproc=OFF \
    -D BUILD_opencv_xobjdetect=OFF -D BUILD_opencv_xphoto=OFF /opt/opencv-3.2.0/ && make -j4 && make install && ldconfig

RUN  cd /usr/local && rm -rf opencv-3.2.0