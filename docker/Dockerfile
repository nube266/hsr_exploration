
# General guidelines and recommendations for writing 'Dockerfile':
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/

################################################################################

# Install Ubuntu Xenial Xerus (16.04) with CUDA and OpenGL support.
# https://hub.docker.com/r/nvidia/cudagl/
# https://gitlab.com/nvidia/cudagl
# https://github.com/NVIDIA/libglvnd

# Pull the official parent image from the Nvidia repository.
FROM nvidia/cudagl:9.0-devel-ubuntu16.04

# Install the Nvidia cuDNN library missing in the parent image.
# https://gitlab.com/nvidia/cuda/blob/ubuntu16.04/9.0/devel/cudnn7/Dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
	libcudnn7=7.1.3.16-1+cuda9.0 \
	libcudnn7-dev=7.1.3.16-1+cuda9.0 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install ROS Kinetic Kame.
# http://wiki.ros.org/kinetic/Installation/Ubuntu

# Update the package list.
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# Add the package keys.
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install 'ros-kinetic-desktop-full' packages (including ROS, Rqt, Rviz, and more).
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-desktop-full \
	&& rm -rf /var/lib/apt/lists/*

# Initialize 'rosdep'.
# http://wiki.ros.org/rosdep
RUN rosdep init

################################################################################

# Install the Toyota ROS packages.
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

# Install the Toyota ROS packages (including the simulator).
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-tmc-desktop-full \
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

# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

# Overwrite the entry point of the parent image.
ENTRYPOINT []

# Enter the container with a Bash shell.
CMD ["/bin/bash"]
