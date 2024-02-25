# Robot Programming with ROS2

Welcome to the Robot Programming with ROS2 repository. This repository contains the source code examples and extended project materials based on the book "A Concise Introduction to Robot Programming with ROS2" by Francisco Martín Rico. This project aims to provide a comprehensive guide and additional resources for readers and practitioners of the book to deepen their understanding of robot programming using ROS2.

## About the Book

"A Concise Introduction to Robot Programming with ROS2" offers readers a structured and clear introduction to robot programming with ROS2.

![9781032264653](https://user-images.githubusercontent.com/3810011/183239477-c98ee6a0-332f-40d2-b368-08a1383747e6.jpg)



Order the book here: [Routledge](https://www.routledge.com/A-Concise-Introduction-to-Robot-Programming-with-ROS2)

## Original Repository

This repository extends and provides solutions to some issues found in the original book's repository. You can find the original source code and materials at [fmrico/book_ros2](https://github.com/fmrico/book_ros2/tree/main?tab=readme-ov-file).

## Requirements

This repository is tested and maintained for the `foxy-devel` branch, requiring:

- Ubuntu 20.04 LTS
- ROS2 Foxy Fitzroy

## Installation

Follow these steps to set up your environment and start with the robot programming projects:

### 1. Install Necessary Packages

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
```

### 2. Add ROS 2 Repository Key

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### 3. Add ROS 2 Repository to Sources List

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 4. Install ROS 2 Foxy Packages

```bash
sudo apt-get update
sudo apt install ros-foxy-desktop
```

### 5. Environment Setup

```bash
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

### 6. Initialize rosdep

```bash
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
```

### 7. Create a ROS 2 Workspace

```bash
mkdir -p ~/bookros2_ws/src
cd ~/bookros2_ws/src
git clone https://github.com/eather0056/Robot-Programming-with-ROS2.git
```
<!-- git clone -b foxy-devel https://github.com/fmrico/bookros2.git -->

### 8. Import Third-Party Packages

```bash
cd ~/bookros2_ws/src
sudo apt install python3-vcstool
vcs import . < book_ros2/third_parties.repos
```

### 9. Install Dependencies and Build Packages

```bash
cd ~/bookros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt update
sudo apt install python3-colcon-common-extensions
colcon build --symlink-install
```

### Additional Dependencies

If you encounter any missing dependencies, you may need to install them manually:

```bash
sudo apt install ros-foxy-diagnostic-updater
sudo apt-get install ros-foxy-control-msgs
sudo apt-get install ros-foxy-xacro
sudo apt-get install ros-foxy-behaviortree-cpp-v3
sudo apt-get install ros-foxy-nav2-msgs
sudo apt-get install ros-foxy-vision-msgs
sudo apt-get install ros-foxy-realtime-tools
sudo apt-get install ros-foxy-gazebo-dev
sudo apt-get install ros-foxy-control-toolbox
sudo apt-get install ros-foxy-gazebo-ros
```

## Usage

After installation, you can explore the various packages and launch files included in this repository. Each chapter of the book corresponds to specific packages and examples. Refer to the book for detailed explanations and instructions on running the examples.

## Contributing

Contributions to this repository are welcome. Please submit issues and pull requests with any suggestions, bug reports, or new content you believe would be beneficial for others learning ROS2.
