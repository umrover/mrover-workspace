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

#Setup Ansible
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

#Install CUDA Driver
COPY cuda-repo-ubuntu1804_10.0.130-1_amd64.deb /usr/local
RUN cd /usr/local && \
    apt-get update -y && apt-get upgrade -y && apt-get update && apt-get install -y gnupg2 && \
    dpkg -i cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
RUN apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
RUN apt-get update -y
RUN apt-get install -y cuda-10-0   

#Install ZED
RUN apt-get update -y && apt-get upgrade -y 
RUN apt-get install -y libturbojpeg0-dev
RUN apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https -y && \
    apt-get update -y && \
    wget -O ZED_SDK_Linux_Ubuntu18.run https://download.stereolabs.com/zedsdk/3.2/cu100/ubuntu18 && \
    chmod +x ZED_SDK_Linux_Ubuntu18.run && \
    ./ZED_SDK_Linux_Ubuntu18.run silent && \
    rm ZED_SDK_Linux_Ubuntu18.run && \
    cd /usr/local && \
    chmod a+xwr --preserve-root --recursive zed

#Install VTK
ENV TERM xterm
RUN echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!INSTALLING VTK" && \
    apt-get update -y && apt-get upgrade -y && apt-get install -y keyboard-configuration libboost-all-dev curl cmake libxt-dev && \
    apt-get install -y --no-install-recommends cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev \
    libswscale-dev python3.6-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff5-dev jasper \
    libdc1394-22-dev libeigen3-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common \
    libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libavutil-dev libavfilter-dev libavresample-dev && \
    curl https://vtk.org/files/release/8.2/VTK-8.2.0.tar.gz --output VTK-8.2.0.tar.gz && \
    tar -xvzf VTK-8.2.0.tar.gz && \
    mv VTK-8.2.0 /usr/local && \
    cd /usr/local/VTK-8.2.0/ && \
    mkdir VTK-Release-build && \
    cd VTK-Release-build/ && \
    cmake -DCMAKE_BUILD_TYPE:STRING=Release /usr/local/VTK-8.2.0/ -DVTK_USE_SYSTEM_PNG=ON && \
    make -j1 install

#Intall Eigen
RUN echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!INSTALLING EIGEN" && \
    cd /usr/local && \
    wget -qO- https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz | sudo tar xz && \
    apt install -y libblas-dev && \
    cd eigen-3.3.7 && sudo mkdir build && cd build && \
    cmake .. && \
    make install && \
    cd ../.. && sudo rm -rf eigen-3.3.7/ && sudo rm -f eigen-3.3.7.tar.gz

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
