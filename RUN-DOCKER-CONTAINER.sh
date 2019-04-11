#!/bin/bash

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
PROJECT=$1
if [ -z "${PROJECT}" ]; then
  PROJECT=${USER}
fi
CONTAINER="${PROJECT}_hsr_1"
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"

# Run the Docker container in the background.
# Any changes made to './docker/docker-compose.yml' will recreate and overwrite the container.
docker-compose -p ${PROJECT} -f ./docker/docker-compose.yml up -d

################################################################################

# Configure the known host names with '/etc/hosts' in the Docker container.
HSRB_HOSTNAME=hsrb.local
echo "Now resolving local host name '${HSRB_HOSTNAME}'..."
HSRB_IP=`avahi-resolve -4 --name ${HSRB_HOSTNAME} | cut -f 2`
if [ "$?" != "0" ]; then
  echo "Failed to execute 'avahi-resolve'. You may need to install 'avahi-utils'."
  docker exec -i ${CONTAINER} bash <<EOF
sed -i 's/TMP_HOSTNAME/${HSRB_HOSTNAME}/' ~/.bashrc
EOF
elif [ ! -z "${HSRB_IP}" ]; then
  echo "Successfully resolved host name '${HSRB_HOSTNAME}' as '${HSRB_IP}': '/etc/hosts' in the container is automatically updated."
  docker exec -i ${CONTAINER} bash <<EOF
sed -i 's/TMP_HOSTNAME/${HSRB_HOSTNAME}/' ~/.bashrc
sed -n -e '/^[^#[:space:]]*[[:space:]]\+${HSRB_HOSTNAME}\$/!p' /etc/hosts > /etc/hosts.tmp;
echo '${HSRB_IP} ${HSRB_HOSTNAME}' >> /etc/hosts.tmp
cp /etc/hosts.tmp /etc/hosts;
EOF
else
  echo "Failed to resolve host name '${HSRB_HOSTNAME}': '/etc/hosts' in the container was not automatically updated."
fi

################################################################################

# Display GUI through X Server by granting full access to any external client.
xhost +

################################################################################

# Enter the Docker container with a Bash shell (with or without a custom 'roslaunch' command).
case "$2" in
  ( "" )
  docker exec -i -t ${CONTAINER} bash
  ;;
  ( "flexbe_app_default.launch" | \
    "sdewg_chatter_default.launch" | \
    "sdewg_gazebo_default.launch" | \
    "sdewg_rviz_default.launch" | \
    "tmc_gazebo_default.launch" )
  docker exec -i -t ${CONTAINER} bash -i -c "source ~/HSR/docker/scripts/run-roslaunch-repeatedly.sh $2"
  ;;
  ( * )
  echo "Failed to enter the Docker container '${CONTAINER}': '$2' is not a valid argument value."
  ;;
esac
