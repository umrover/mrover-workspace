FROM nvidia/cuda:9.0-devel-ubuntu16.04

MAINTAINER Milo Hartsoe <hartsoe@umich.edu>

ENV ANSIBLE_VERSION 2.6.0

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
    echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections && \
    wget -O ZED_SDK_Linux_Ubuntu16.run https://www.stereolabs.com/developers/downloads/ZED_SDK_Ubuntu16_CUDA9_v2.8.0.run && \
    chmod +x ZED_SDK_Linux_Ubuntu16.run ; bash ZED_SDK_Linux_Ubuntu16.run silent

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
