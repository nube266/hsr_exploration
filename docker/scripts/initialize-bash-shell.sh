#!/bin/bash

################################################################################

# Link the default shell 'sh' to Bash.
alias sh='/bin/bash'

################################################################################

# Configure the terminal.

# Disable flow control. If enabled, inputting 'ctrl+s' locks the terminal until inputting 'ctrl+q'.
stty -ixon

################################################################################

# Configure 'umask' for giving read/write/execute permission to group members.
umask 0002

################################################################################

# Source the ROS environment.
echo "Sourcing the ROS environment from '/opt/ros/kinetic/setup.bash'."
source /opt/ros/kinetic/setup.bash

# Source the Catkin workspace.
echo "Sourcing the Catkin workspace from '/root/HSR/catkin_ws/devel/setup.bash'."
source /root/HSR/catkin_ws/devel/setup.bash

################################################################################

# Add the Catkin workspace to the 'ROS_PACKAGE_PATH'.
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/HSR/catkin_ws/src/

################################################################################

# Define Bash functions to conveniently execute the helper scripts in the current shell process.

function hsr-fix-git-paths () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/HSR/docker/scripts/fix-git-paths.sh
  popd
}

function hsr-initialize-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/HSR/docker/scripts/fix-git-paths.sh
  source /root/HSR/docker/scripts/fix-permission-issues.sh
  source /root/HSR/docker/scripts/initialize-catkin-workspace.sh
  popd
}

function hsr-reset-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/HSR/docker/scripts/fix-git-paths.sh
  source /root/HSR/docker/scripts/fix-permission-issues.sh
  source /root/HSR/docker/scripts/reset-catkin-workspace.sh
  popd
}

function hsr-fix-permission-issues () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/HSR/docker/scripts/fix-git-paths.sh
  source /root/HSR/docker/scripts/fix-permission-issues.sh
  popd
}

function hsr-download-model-data () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/HSR/docker/scripts/fix-permission-issues.sh
  source /root/HSR/docker/scripts/download-model-data.sh
  popd
}

function hsr-get-fully-started () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/HSR/docker/scripts/fix-git-paths.sh
  source /root/HSR/docker/scripts/fix-permission-issues.sh
  source /root/HSR/docker/scripts/download-model-data.sh
  source /root/HSR/docker/scripts/reset-catkin-workspace.sh
  popd
}

################################################################################

# Set HSR/ROS network interface.
# https://docs.hsr.io/manual_en/howto/pc_install.html
# https://docs.hsr.io/manual_en/howto/network_settings.html

# The value of 'HSRB_HOSTNAME' should be initialized in '~/.bashrc' by './RUN-DOCKER-CONTAINER.sh' when entering the container.
HSRB_IP=`getent hosts ${HSRB_HOSTNAME} | cut -d ' ' -f 1`
if [ -z "${HSRB_IP}" ]; then
  export ROS_IP=$(LANG=C /sbin/ifconfig docker0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*')
else
  export ROS_IP=`python /root/HSR/docker/scripts/print-interface-ip.py ${HSRB_IP}`
fi
echo "ROS_IP is set to '${ROS_IP}'."

export ROS_HOME=~/.ros

alias sim_mode='export ROS_MASTER_URI=http://localhost:11311; export PS1="\[[44;1;37m\]<local>\[[0m\]\w$ "'
alias hsrb_mode='export ROS_MASTER_URI=http://hsrb.local:11311; export PS1="\[[41;1;37m\]<hsrb>\[[0m\]\w$ "'

################################################################################

# Move to the working directory.
cd /root/HSR/
