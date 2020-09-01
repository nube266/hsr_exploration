
# General guidelines and recommendations for writing 'Dockerfile':
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/

################################################################################

# Pin the versions of the core tools and packages using global arguments for improved stability.
ARG nvidia_cudagl_version=10.1-devel-ubuntu16.04
ARG nvidia_cudnn_version=7.6.0.64-1+cuda10.1
ARG ros_desktop_version=1.3.2-0xenial-20200812-160911+0000
ARG ros_tmc_version=0.11.0-1.xenial.20190426.0557.+0000
ARG gazebo_version=7.16.1-1~xenial

################################################################################

# Install Ubuntu Xenial Xerus (16.04) with CUDA and OpenGL support.
# https://hub.docker.com/r/nvidia/cudagl/
# https://gitlab.com/nvidia/cudagl
# https://github.com/NVIDIA/libglvnd

# Pull the official parent image from the Nvidia repository.
FROM nvidia/cudagl:${nvidia_cudagl_version}

# Install the Nvidia cuDNN library missing in the parent image.
# https://gitlab.com/nvidia/container-images/cuda/blob/ubuntu16.04/10.1/devel/cudnn7/Dockerfile
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

# Support Intel 3D acceleration when no Nvidia drivers are found:
# If the argument 'nvidia_gl' is 'false', remove the configuraion files related to the GL library provided by Nvidia and re-generate '/etc/ld.so.cache' by executing 'ldconfig'.
# This is required to avoid problems occurring when the host system does not have the GL library provided by Nvidia.
# In addition, remove the value of the environment variable 'LD_LIBRARY_PATH' which will be inherited from the base image.
# The variable is not necessary as long as the same information is provided by '/etc/ld.so.cache'.
ARG nvidia_gl="true"
RUN if [ "${nvidia_gl}" = "false" ]; then \
		rm /etc/ld.so.conf.d/nvidia.conf /etc/ld.so.conf.d/glvnd.conf; \
		ldconfig; \
	fi
ENV LD_LIBRARY_PATH=

# Update the package list.
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# Add the package keys.
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install 'ros-kinetic-desktop-full' packages (including ROS, Rqt, Rviz, and more).
ARG ros_desktop_version
ENV ROS_DESKTOP_VERSION=${ros_desktop_version}
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-desktop-full=${ROS_DESKTOP_VERSION} \
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
	apt-transport-https=1.2.32 \
	wget=1.17.1-1ubuntu1.5 \
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

# Install the main Toyota Motor Corporation ROS packages.
ARG ros_tmc_version
ENV ROS_TMC_VERSION=${ros_tmc_version}
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-tmc-desktop-full=${ROS_TMC_VERSION} \
	&& rm -rf /var/lib/apt/lists/*

# Install additional Toyota Motor Corporation ROS packages not included in 'ros-kinetic-tmc-desktop-full'.
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-hsrb-moveit=0.6.0-1.xenial.20190425.1022.+0000 \
	ros-kinetic-hsrb-moveit-config=0.6.0-1.xenial.20190425.1022.+0000 \
	ros-kinetic-hsrb-moveit-plugins=0.6.0-1.xenial.20190425.1022.+0000 \
	&& rm -rf /var/lib/apt/lists/*

# Upgrade 'gazebo7' to a version >= 7.4.0:
# This package is installed as a dependency of 'ros-kinetic-gazebo-dev' when installing 'ros-kinetic-desktop-full'.
# However, its version is 7.0.0 (2018-07-12), which has a problem making 'gzserver' crashing on specific environments such as virtual or remote machines.
# This problem is solved in version 7.4.0 and later according to https://bitbucket.org/osrf/gazebo/issues/1837/vmware-rendering-z-ordering-appears-random#comment-31716451.
# The latest 7.x.x version is included in the repository 'http://packages.osrfoundation.org/gazebo/ubuntu-stable' added when installing the Toyota HSR packages.
RUN apt-get update && apt-cache policy gazebo7 git python-catkin-tools
ARG gazebo_version
ENV GAZEBO_VERSION=${gazebo_version}
RUN apt-get update && apt-get install -y --no-install-recommends \
	gazebo7=${GAZEBO_VERSION} \
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

RUN apt-get update && apt-get install -y pulseaudio libpulse-dev mecab libmecab-dev portaudio19-dev ros-kinetic-tf2-sensor-msgs \
	&& rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN curl -kL https://bootstrap.pypa.io/get-pip.py | python
# RUN pip install pyaudio pocketsphinx mecab-python pycorenlp
# pyaudio

# RUN apt-get update && apt-get install -y ros-kinetic-move-base \
# 	&& rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
RUN apt-get update && apt-get install -y \
	ros-kinetic-navigation \
	ros-kinetic-octomap-server \
	python-tk \
	&& rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN echo 'rm -rf ~/.ros/params.yaml' >> /root/.bashrc
RUN echo 'ln -s /root/HSR/params.yaml ~/.ros/params.yaml' >> /root/.bashrc
# Enter the container with a Bash shell.
CMD ["/bin/bash"]
