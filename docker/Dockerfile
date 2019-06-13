
# General guidelines and recommendations for writing 'Dockerfile':
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/

################################################################################

# Pin the versions of the core tools and packages using global arguments for improved stability.
ARG nvidia_cudagl_version=9.0-devel-ubuntu16.04
ARG nvidia_cudnn_version=7.1.3.16-1+cuda9.0
#ARG ros_desktop_version=1.3.2-0xenial-20190320-232543-0800
ARG ros_tmc_version=0.10.0-1.xenial.20181129.0043.+0000

################################################################################

# Install Ubuntu Xenial Xerus (16.04) with CUDA and OpenGL support.
# https://hub.docker.com/r/nvidia/cudagl/
# https://gitlab.com/nvidia/cudagl
# https://github.com/NVIDIA/libglvnd

# Pull the official parent image from the Nvidia repository.
FROM nvidia/cudagl:${nvidia_cudagl_version}

# Install the Nvidia cuDNN library missing in the parent image.
# https://gitlab.com/nvidia/cuda/blob/ubuntu16.04/10.1/devel/cudnn7/Dockerfile
ARG nvidia_cudnn_version
ENV NVIDIA_CUDNN_VERSION=${nvidia_cudnn_version}
RUN apt-get update && apt-get install -y --no-install-recommends \
	libcudnn7=${NVIDIA_CUDNN_VERSION} \
	libcudnn7-dev=${NVIDIA_CUDNN_VERSION} \
	&& apt-mark hold libcudnn7 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install ROS Kinetic Kame.
# http://wiki.ros.org/kinetic/Installation/Ubuntu

# Update the package list.
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# Add the package keys.
RUN apt-key adv --keyserver 'hkp://ha.pool.sks-keyservers.net:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install 'ros-kinetic-desktop-full' packages (including ROS, Rqt, Rviz, and more).
#ARG ros_desktop_version
#ENV ROS_DESKTOP_VERSION=${ros_desktop_version}
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-desktop-full \
	&& rm -rf /var/lib/apt/lists/*

# Initialize 'rosdep'.
# http://wiki.ros.org/rosdep
RUN rosdep init

################################################################################

# Install the Toyota Motor Corporation ROS packages.
# https://docs.hsr.io/manual_en/howto/pc_install.html

# Install the missing required tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
	apt-transport-https \
	wget \
	&& rm -rf /var/lib/apt/lists/*

# Update the package lists.
RUN echo "deb https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu xenial main" > /etc/apt/sources.list.d/tmc.list \
	&& echo "deb https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu xenial multiverse main" >> /etc/apt/sources.list.d/tmc.list \
	&& echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" > /etc/apt/sources.list.d/gazebo-stable.list

# Add the package keys.
RUN wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -P /tmp \
	&& apt-key add /tmp/tmc.key \
	&& rm /tmp/tmc.key \
	&& wget http://packages.osrfoundation.org/gazebo.key -P /tmp \
	&& apt-key add /tmp/gazebo.key \
	&& rm /tmp/gazebo.key

# Install the Toyota Motor Corporation ROS packages (including the simulator).
ARG ros_tmc_version
ENV ROS_TMC_VERSION=${ros_tmc_version}
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-tmc-desktop-full=${ROS_TMC_VERSION} \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install dependencies for the 'flexbe_app' ROS package.
# https://github.com/FlexBE/flexbe_app

# Gather the dependencies from the Ubuntu repositories.
RUN apt-get update && apt-get install -y --no-install-recommends \
  libcanberra-gtk3-module \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install common optional packages/tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
	git \
	inetutils-ping \
	iproute2 \
	mesa-utils \
	nano \
	net-tools \
	openssh-client \
	python-catkin-tools \
	unzip \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install the SSH deploy key of the repository hosted on GitLab.com.

# Copy the SSH deploy key from the sources on the host to the container.
COPY ./.ssh/. /root/.ssh/
RUN chmod 700 /root/.ssh/ \
	&& chmod 600 /root/.ssh/sdewg_id_rsa

# Add GitLab.com as a known host using the SSH deploy key.
RUN echo "Host gitlab.com" > /root/.ssh/config \
	&& echo "  Hostname gitlab.com" >> /root/.ssh/config \
	&& echo "  IdentityFile /root/.ssh/sdewg_id_rsa" >> /root/.ssh/config \
	&& echo "  StrictHostKeyChecking no" >> /root/.ssh/config \
	&& echo "  User git" >> /root/.ssh/config

################################################################################

# Set up the Bash shell environment.

# Define the working directory.
WORKDIR /root/

# Copy the helper scripts to the working directory.
COPY ./docker/scripts/. /root/HSR/docker/scripts/
RUN chmod -R +x /root/HSR/docker/scripts/*

# Setup the Bash shell environment with '/root/.bashrc'.
# The value of 'TMP_HOSTNAME' is substituted by './RUN-DOCKER-CONTAINER.sh' when entering the container.
RUN echo "HSRB_HOSTNAME=\"TMP_HOSTNAME\"" >> /root/.bashrc
RUN echo "source /root/HSR/docker/scripts/initialize-bash-shell.sh" >> /root/.bashrc
RUN echo 'alias hsrviz="rosrun rviz rviz -d `rospack find hsrb_common_launch`/config/hsrb_display_full_hsrb.rviz"' >> /root/.bashrc
RUN echo 'echo $ROS_IP hsr >> /etc/hosts' >> /root/.bashrc


# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

# Overwrite the entry point of the parent image.
# ENTRYPOINT []

#
# AISL ROS packages and dependencies
# 

# NLP
RUN cd /tmp \ 
    && wget http://prdownloads.sourceforge.net/swig/swig-3.0.12.tar.gz \
    && tar -zxvf swig* \
    && cd swig* \
    && ./configure \
    && make \
    && make install

COPY jdk* /tmp/

RUN cd /tmp \ 
    && tar -zxvf jdk-8u121-linux-x64.tar.gz \
    && mkdir -p /usr/lib/jvm \
    && mv ./jdk1.8.0_121 /usr/lib/jvm/ \
    && update-alternatives --install "/usr/bin/java" "java" "/usr/lib/jvm/jdk1.8.0_121/bin/java" 1 \
    && update-alternatives --install "/usr/bin/javac" "javac" "/usr/lib/jvm/jdk1.8.0_121/bin/javac" 1 \
    && update-alternatives --install "/usr/bin/javaws" "javaws" "/usr/lib/jvm/jdk1.8.0_121/bin/javaws" 1 \
    && chmod a+x /usr/bin/java \
    && chmod a+x /usr/bin/javac \
    && chmod a+x /usr/bin/javaws \
    && chown -R root:root /usr/lib/jvm/jdk1.8.0_121 \
    && update-alternatives --config java \
    && update-alternatives --config javac \
    && update-alternatives --config javaws

# Stanford NLP
# TODO: Change this part so that the zip file is downloaded using curl
#       (Problem occured where the downloaded file can't be unzipped. File broken?)
# RUN cd /tmp && curl -O http://nlp.stanford.edu/software/stanford-corenlp-full-2015-12-09.zip \
COPY ./stanford-corenlp-full-2015-12-09.zip /tmp/.
RUN  cd /tmp && unzip stanford-corenlp-full-2015-12-09.zip -d /root/.

RUN apt-get update && apt-get install -y pulseaudio libpulse-dev mecab libmecab-dev portaudio19-dev \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN curl -kL https://bootstrap.pypa.io/get-pip.py | python
RUN pip install pyaudio pocketsphinx mecab-python pycorenlp 
# pyaudio

RUN echo 'rm -rf ~/.ros/params.yaml' >> /root/.bashrc
RUN echo 'ln -s /root/HSR/params.yaml ~/.ros/params.yaml' >> /root/.bashrc
# Enter the container with a Bash shell.
CMD ["/bin/bash"]
