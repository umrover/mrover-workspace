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

# Install OpenCV
RUN apt-get install -y libopencv-dev

# Install ZED (also installs CUDA 10.2)
RUN echo "==================== Installing ZED SDK ====================" && \
    apt-get install -y lsb-release less udev sudo keyboard-configuration -y && \
    wget -O ZED_SDK_Linux_Ubuntu18.run https://download.stereolabs.com/zedsdk/3.7/cu102/ubuntu18 && \
    chmod +x ZED_SDK_Linux_Ubuntu18.run && \
    ./ZED_SDK_Linux_Ubuntu18.run -- silent && \
    rm ZED_SDK_Linux_Ubuntu18.run && \
    chmod a+xwr --preserve-root --recursive /usr/local/zed

# GStreamer
RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x \
    gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

# Install VTK 6
RUN apt-get install -y libvtk6-dev

# Install PCL 11.1, not available via apt
RUN echo "==================== Building & Installing PCL ====================" && \
    apt-get install -y libflann-dev libgtest-dev libboost-all-dev && \
    cd /usr/local && \
    wget https://codeload.github.com/PointCloudLibrary/pcl/tar.gz/pcl-1.11.1 -qO- | tar xz && \
    cd pcl-pcl-1.11.1 && \
    cmake -DCMAKE_BUILD_TYPE=Release . -GNinja -Bbuild && \
    cd build && \
    ninja -j $(($(nproc) - 1)) && ninja install && ldconfig && \
    rm -rf /usr/local/pcl-1.11.1

# Set up ease of use shell tools
ENV TERM xterm
RUN apt-get install -y zsh net-tools neovim tmux
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true

ENTRYPOINT ["/bin/zsh"]