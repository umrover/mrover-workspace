#FROM ubuntu:18.04
FROM stereolabs/zed:3.0-devel-cuda10.0-ubuntu18.04
RUN apt-get update
CMD ["echo", "Hello World"]

RUN set -x && \
    echo ">> Adding build-dependencies..." && \
    sed -i 's/# \(.*multiverse$\)/\1/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get -y upgrade && \
    apt-get install -y build-essential \
    software-properties-common \
    apt-transport-https \
    git \
    wget && \ 
    apt-get update 

RUN apt-add-repository ppa:ansible/ansible && \
    apt-get update && \
    apt-get install -y ansible && \
    echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections

#RUN cd /usr/local/ && \
#    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin && \
#    mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 

#COPY cuda-repo-ubuntu2004-11-5-local_11.5.1-495.29.05-1_amd64.deb /usr/local/

#RUN cd /usr/local/ && \
#    dpkg -i cuda-repo-ubuntu2004-11-5-local_11.5.1-495.29.05-1_amd64.deb && \
#    apt-key add /var/cuda-repo-ubuntu2004-11-5-local/7fa2af80.pub && \
#    apt-get update && \
#    apt-get -y install cuda

RUN apt install -y libglm-dev && \
    apt install -y libglfw3-dev && \ 
    apt install -y freeglut3-dev && \
    apt install -y libglew-dev && \
    apt install -y x11-apps && \
    apt-get install -y python3-venv && \
    apt install -y pkg-config && \
    apt install -y autoconf && \
    apt-get install -y dh-autoreconf && \
    apt-get install -y libgtk2.0-dev && \
    apt-get install -y python3-dev && \
    apt-get install -y libgirepository1.0-dev


# for ubuntu 20.04 python-setuptools

#COPY ubuntu18 /usr/local/ubuntu18

#RUN cd /usr/local/ && \
#    chmod 777 ubuntu18 && \
#    ./ubuntu18 -- silent