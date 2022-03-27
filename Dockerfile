# INSTRUCTIONS #
#This Dockerfile needs to be built into an image prior to testing
#This image, once build, should be pushed to umrover/travis
#Our Travis system will then download this image from DockerHub and run our current system within this image

# UPDATING THE DOCKERFILE #
#If you would like to make changes to the DockerFile you must first install Docker on your system
#You can then download the pre-built image from DockerHub the way specified in the travis.yml
#Now that the image is downloaded you can take advantage of Docker Layers and when you build you
#won't need to rebuild the entire image, but rather, only the RUN commands that you added or modified will be built
#You can build the Dockerfile with the command: 'docker build -t mrover .'

# PUSHING THE DOCKERIMAGE TO DOCKERHUB #
#Now that you have built the Docker Image you must push it to DockerHub
#This can be done by signing into the mrover DockerHub account via the command line
#Once you are signed in make sure the tag for your image is named appropriately. Use 'docker tag' to do any renaming
#Then run 'docker push umrover/travis' to push the appropriately named and versioned image to DockerHub
#Travis is configured to download whatever the latest umrover/travis image is, so once your changes are pushed they
#should be picked up by travis

FROM ubuntu:18.04

# Setup Ansible
ENV ANSIBLE_VERSION 2.9.4
ENV DEBIAN_FRONTEND noninteractive

COPY ansible /tmp/ansible

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
    ninja-build cmake && \
    apt-add-repository ppa:ansible/ansible && \
    apt-get update -y && \
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

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get autoremove -y && \
    apt-get install -y --no-install-recommends cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev \ 
    libswscale-dev python3.6-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff5-dev jasper \
    libdc1394-22-dev libeigen3-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common libtbb-dev \
    yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenexr-dev libgstreamer-plugins-base1.0-dev \
    libavutil-dev libavfilter-dev libavresample-dev

RUN apt-get install -y gcc

# Install OpenCV 3.4 (modified)
RUN cd ~ && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout 3.4 && cd .. && \
    git clone https://github.com/JasonLiu2002/opencv_contrib.git && \
    cd opencv_contrib && git checkout 3.4 && cd .. && \
    cd opencv && mkdir build && \
    cd build && \
    cmake -D WITH_CUDA=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules -D BUILD_opencv_aruco=ON -D BUILD_opencv_bgsegm=OFF \
    -D BUILD_opencv_bioinspired=OFF -D BUILD_opencv_ccalib=OFF -D BUILD_opencv_cnn_3dobj=OFF -D BUILD_opencv_cvv=OFF \
    -D BUILD_opencv_datasets=OFF -D BUILD_opencv_dnn_objdetect=OFF -D BUILD_opencv_dnn_superres=OFF \
    -D BUILD_opencv_dnns_easily_fooled=OFF -D BUILD_opencv_dpm=OFF -D BUILD_opencv_face=OFF -D BUILD_opencv_fuzzy=OFF \
    -D BUILD_opencv_freetype=OFF -D BUILD_opencv_hdf=OFF -D BUILD_opencv_line_descriptor=OFF -D BUILD_opencv_matlab=OFF \
    -D BUILD_opencv_optflow=OFF -D BUILD_opencv_ovis=OFF -D BUILD_opencv_plot=OFF -D BUILD_opencv_reg=OFF \
    -D BUILD_opencv_rgbd=OFF -D BUILD_opencv_saliency=OFF -D BUILD_opencv_sfm=OFF -D BUILD_opencv_stereo=OFF \
    -D BUILD_opencv_structured_light=OFF -D BUILD_opencv_surface_matching=OFF -D BUILD_opencv_text=OFF \
    -D BUILD_opencv_tracking=OFF -D BUILD_opencv_xfeatures2d=OFF -D BUILD_opencv_ximgproc=OFF \
    -D BUILD_opencv_xobjdetect=OFF -D BUILD_opencv_xphoto=OFF ~/opencv/ -G Ninja && ninja -j $(($(nproc) - 1)) && ninja install && ldconfig


# Install ZED (also installs CUDA 10.2)
RUN echo "==================== Installing ZED SDK ====================" && \
    apt-get install -y lsb-release less udev sudo keyboard-configuration -y && \
    wget -O ZED_SDK_Linux_Ubuntu18.run https://download.stereolabs.com/zedsdk/3.7/cu102/ubuntu18 && \
    chmod +x ZED_SDK_Linux_Ubuntu18.run && \
    ./ZED_SDK_Linux_Ubuntu18.run -- silent && \
    rm ZED_SDK_Linux_Ubuntu18.run && \
    cd /usr/local && \
    chmod a+xwr --preserve-root --recursive zed

# GStreamer
RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x \
    gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

RUN apt-get install -y libvtk6-dev

#Install PCL
RUN echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!INSTALLING PCL" && \
    apt-get install -y libflann-dev libgtest-dev libboost-all-dev && \
    cd /usr/local && \
    curl https://codeload.github.com/PointCloudLibrary/pcl/tar.gz/pcl-1.11.1 --output pcl-pcl-1.11.1.tar.gz && \
    tar -xvzf pcl-pcl-1.11.1.tar.gz && \
    rm pcl-pcl-1.11.1.tar.gz && \
    cd pcl-pcl-1.11.1 && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j1 && make install && ldconfig && \
    cd /usr/local && \
    rm -rf pcl-pcl-1.11.1


# Set up ease of use shell tools
ENV TERM xterm
ENV ZSH_THEME dstufft
RUN apt-get install -y zsh net-tools neovim tmux
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true

ENTRYPOINT ["/bin/zsh"]