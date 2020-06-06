<!-- --- title: Initial Setup -->

# Initial Setup

Follow this step-by-step guide to perform the initial setup of the HSR project on a development machine running [Ubuntu](https://www.ubuntu.com/).

**Content:**

[[_TOC_]]

> **Note 1:** The whole development environment of the HSR project is containerized/virtualized using [Docker](https://www.docker.com/). This ensures that all the contributors work in the exact same software environment.

> **Note 2:** The HSR robot and simulator interfaces are primarily implemented with [ROS](http://www.ros.org/).

> **Note 3:** The HSR project relies heavily on both hardware 3D graphic acceleration and Nvidia [CUDA](https://developer.nvidia.com/cuda-toolkit), thus a discrete Nvidia GPU is highly recommended. Machines without Nvidia GPUs (including virtual machines) are only partially supported: 3D accelerated tools such as [Rviz](https://github.com/ros-visualization/rviz) and [Gazebo](http://gazebosim.org/) will most likely crash at runtime.

> **Note 4:** Although this initial setup is meant to be performed only once, you can run it again would you want to reset the development environment as the project evolves. However, make sure to backup your latest changes beforehand.

## Step 0: Verify the Prerequisites

**Mandatory:**

*   A machine running Ubuntu 16.04 LTS (Xenial Xerus) on an AMD64 architecture.
*   Access to administrator privileges (`sudo`) on the Ubuntu machine.
*   Access to developer privileges on the GitLab project at https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/HSR.

**Recommended:**

*   A Toyota HSR robot for full operability. If not, the HSR simulator provides support for basic operations.
*   A Nvidia GPU capable of running CUDA 9.0 (compute capability >= 3.0) or newer. If not, 3D accelerated tools will most likely crash (including Rviz, Gazebo, and thus the HSR simulator).
*   Access to the Toyota HSR community portal at https://hsr.io/. Official membership is required to access the portal resources.

## Step 1: Set up the Development Environment

Set up the environment of the development machine with the following instructions.

1.   Install [Git](https://git-scm.com/) if necessary:

     ```shell
     sudo apt-get update && sudo apt-get install -y git
     ```

2.   Clone the HSR project repository in your home folder:

     ```shell
     cd ~/ && git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/hsr.git
     ```

     Enter your GitLab developer credentials if prompted.
3.   Configure the system environment:

     ```shell
     cd ~/HSR/ && ./SETUP-DEVEL-MACHINE.sh
     ```

     The execution of `SETUP-DEVEL-MACHINE.sh` requires `sudo` permissions to install the tools that allow virtualization, i.e. Docker, [Docker Compose](https://github.com/docker/compose), and [Nvidia Docker 2](https://github.com/NVIDIA/nvidia-docker). System changes made with `sudo` are kept to a strict minimum.
4.   Reboot the system (or log out and back in) for the changes to users and groups to take effect:

     ```shell
     sudo reboot
     ```

> **Note 5:** The `SETUP-DEVEL-MACHINE.sh` script is actually divided into `INCL-SUDO-ENV.sh` and `INCL-USER-ENV.sh`. The execution of `INCL-SUDO-ENV.sh` makes system-wide changes and thus requires `sudo` permissions. However, if your system has already all the necessary tools installed, you can directly set up your local user environment with `cd ~/HSR/ && ./INCL-USER-ENV.sh` which does not require `sudo` permissions.

> **Note 6:** You do not need to reboot if your local user has already been added to the `docker` group. If so, executing `docker --version` should not ask for `sudo`. In principle, you only need to reboot after the very first time you run `SETUP-DEVEL-MACHINE.sh`.

## Step 2: Build the Docker Image

Create a virtual environment using Docker (= Docker image) on the development machine with the following instructions.

1.   Build the Docker image:

     ```shell
     cd ~/HSR/ && ./BUILD-DOCKER-IMAGE.sh
     ```

     This script builds the image following the instructions found in `~/HSR/docker/Dockerfile`.

> **Note 7:** Avoid using a Wi-Fi connection the first time you build the image to greatly accelerate the process. Note that future builds will reuse cached data whenever possible.

## Step 3: Run the Docker Container

Enter a virtual instance of the Docker image (= Docker container) on the development machine with the following instructions.

1.   Run the Docker container:

     ```shell
     cd ~/HSR/ && ./RUN-DOCKER-CONTAINER.sh
     ```

     This script creates or updates the container following the instructions found in `~/HSR/docker/docker-compose.yml`. It allows the container to share system resources, such as volumes and devices, with the host machine.
2.   Use `ctrl+d` to exit the container at any time.

> **Note 8:** If no Nvidia drivers are present, the Docker runtime is set to `runc`, instead of `nvidia`, to bypass `nvidia-docker2` when entering the container. However, 3D accelerated tools, including the HSR simulator, will most likely not work. You can modify the default runtime in `~/.bashrc`.

> **Note 9:** The script `RUN-DOCKER-CONTAINER.sh` will try to resolve host name `hsrb.local` of the HSR robot and add its IP address to `/etc/hosts`. You can confirm the result with `ping hsrb.local` from inside the container. Note that you can ignore the errors if you do not plan to use the HSR robot at that time.

> **Note 10:** Be careful if you need to modify `docker-compose.yml` as the container will be recreated from scratch the next time you run `RUN-DOCKER-CONTAINER.sh`.

## Step 4: Test the Catkin Workspace

Build and test the ROS environment (= Catkin workspace) inside the Docker container with the following instructions.

1.   Enter the Docker container if necessary:

     ```shell
     cd ~/HSR/ && ./RUN-DOCKER-CONTAINER.sh
     ```

2.   Initialize the Catkin workspace:

     ```shell
     cd /root/HSR/ && ./docker/scripts/reset-catkin-workspace.sh
     ```

     This script will remove any existing Catkin workspace and build a new one inside `/root/HSR/catkin_ws/`.
3.   Make sure that the new Catkin workspace is sourced:

     ```shell
     cd /root/HSR/ && source ./catkin_ws/devel/setup.bash
     ```

3.   Launch a couple of publisher and subscriber test nodes from the `hsr_launch` package:

     ```shell
     roslaunch hsr_launch sdewg_chatter_default.launch
     ```

     If everything has installed correctly, you should see a node `sdewg_listener` that subscribes to a publisher node called `sdewg_talker` in a [Rqt](http://wiki.ros.org/rqt) GUI.

> **Note 11:** The script `reset-catkin-workspace.sh` will build the Catkin workspace using `catkin_make` instead of the newer/faster but still unofficially supported `catkin build` to solve some compatibility issues (at least for now). Please be sure to build using `catkin_make` to avoid strange issues.

## Step 5: Learn the Advanced Functions

The development environment inside the Docker container offers several useful functions that you should be aware of. These advanced functions will help you increase both the convenience and the quality of your work for the HSR project.

### Custom Bash Functions

The development environment contains the several useful Bash functions, all starting with the prefix `hsr-`, to make your work more convenient. Including, but not limited to:

*   `hsr-download-model-data`: Download from the cloud all the large binary files required at runtime (models, datasets, dictionaries, weights, etc.).
*   `hsr-initialize-catkin-workspace`: Initialize, build, and source the Catkin workspace on top of the system ROS environment.
*   `hsr-reset-catkin-workspace`: Remove built artefacts, then cleanly rebuild and source the Catkin workspace on top of the system ROS environment (to use after, for example, switching branches).
*   `hsr-fix-permission-issues`: Fix the various permission issues that may appear when manipulating, on the host machine, files generated by the `root` user of the Docker container.
*   `hsr-get-fully-started`: Execute several of the aforementioned functions to quickly get started when entering a freshly built Docker container.

> **Note 12:** These Bash functions are based on helper scripts that can be found in `/root/HSR/docker/scripts/` in the Docker container or in `~/HSR/docker/scripts/` in the host machine. You can see their definitions in `~/.bashrc` inside the container.

### Multiple Terminal Operation

You can simultaneously run multiple terminals using `RUN-TERMINATOR-TERMINAL.sh`. This script opens [Terminator](https://gnometerminator.blogspot.com/) with the default layout configuration stored in `~/HSR/terminator/config`. Each sub-terminal automatically executes `RUN-DOCKER-CONTAINER.sh` with a predefined ROS launch file for convenience. You can then select execution options by pressing specific keys as shown in the example below:

```
Run 'example_roslaunch_file.launch'? Press:
'r' to run with the robot,
's' to run in the simulator,
'c' to enter a child shell,
'q' to quit.
```

### Configuration of the Docker Container

A cascade of scripts performs the initialization of the Docker container. Although the boundaries between them can sometimes be blurry, each one has a specific function and future implementations/revisions should keep these functions separated as much as possible. They can be found on the host machine at:

*   `~/HSR/docker/Dockerfile`: Used by `BUILD-DOCKER-IMAGE.sh` to create the image of the shared development environment. It mainly describes the installations of the project tools/dependencies required inside the container.
*   `~/HSR/docker/docker-compose.yml`: Describes the interface between the host machine and the container. It includes external information from outside the container, such as device host names or network configuration. It is invoked when the container is (re)started, most frequently the first time that `RUN-DOCKER-CONTAINER.sh` is run after the container is either stopped (host reboot) or rebuilt.
*   `~/HSR/docker/scripts/initialize-docker-container.sh`: A startup script referenced in `docker-compose.yml` and thus also invoked when (re)starting the container, most frequently the first time that `RUN-DOCKER-CONTAINER.sh` is run after the container is either stopped (host reboot) or rebuilt. It executes the internal commands needed at the startup of the container (for example, to maintain the container running in the background).
*   `~/HSR/docker/scripts/initialize-bash-shell.sh`: A shell script appended to `/root/.bashrc` at the end of the `Dockerfile` to set up the Bash shell environment inside the container (terminal colors, shell functions, symbolic links, binary paths, environment sourcing, etc.). Theoretically, its content could be written directly in the `Dockerfile` but this would result in complex lines of code as helpful Bash syntax like here-documents are not supported by Docker. It is automatically sourced by `/root/.bashrc` every time the user enters the Bash shell inside the container, most likely every time `RUN-DOCKER-CONTAINER.sh` is run.

## Step 6: Develop on the Simulator/Robot

From here, you can continue with either one, or both, of the following options depending on the presence of a HSR robot within the same local network as the development machine.

*   Start developing on the HSR simulator: [https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/HSR/wikis/simulator-development](simulator-development).
*   Start developing on the HSR robot: [https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/HSR/wikis/robot-development](robot-development).

Also, before writing any new code, please make sure to have read the contribution guidelines at: [https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/HSR/blob/devel/CONTRIBUTING.md](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/wrs/HSR/blob/devel/CONTRIBUTING.md).

> **Note 13:** The most important rule is to avoid pushing large binary files (datasets, weights, etc.) in the repository. Instead, you need to provide a link to download all your necessary large binary files from the cloud with `hsr-download-model-data`. Ideally, all these files should be centralized in the OwnCloud online storage provided by Toyota at: https://share.hsr.io/.