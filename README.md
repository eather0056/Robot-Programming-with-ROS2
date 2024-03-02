# Robot Programming with ROS2

Welcome to the Robot Programming with ROS2 repository. This repository contains the source code examples and extended project materials based on the book "A Concise Introduction to Robot Programming with ROS2" by Francisco Martín Rico. This project aims to provide a comprehensive guide and additional resources for readers and practitioners of the book to deepen their understanding of robot programming using ROS2.

## About the Book

"A Concise Introduction to Robot Programming with ROS2" offers readers a structured and clear introduction to robot programming with ROS2.

![9781032264653](https://user-images.githubusercontent.com/3810011/183239477-c98ee6a0-332f-40d2-b368-08a1383747e6.jpg)



Order the book here: [Book](https://www.routledge.com/A-Concise-Introduction-to-Robot-Programming-with-ROS2/Rico/p/book/9781032264653#:~:text=A%20Concise%20Introduction%20to%20Robot%20Programming%20with%20ROS2%20provides%20the,the%20new%20version%20of%20ROS.)

## Original Repository

This repository extends and provides solutions to some issues found in the original book's repository. You can find the original source code and materials at [fmrico/book_ros2](https://github.com/fmrico/book_ros2/tree/main?tab=readme-ov-file).

## Requirements

This repository is tested and maintained for the `Humble Hawksbill` branch, requiring:

- Ubuntu 22.04 LTS
- ROS2 [Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## Installation

Follow these steps to set up your environment and start with the robot programming projects:

### Set locale
```bash
locale

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale
```
### Install Necessary Packages

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
```

### Add ROS 2 Repository Key

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### 3. Add ROS 2 Repository to Sources List

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 4. Install ROS 2 Foxy Packages

```bash
sudo apt-get update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 5. Environment Setup

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
### Dependencies for Building Packages
```bash
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-vcstool python3-colcon-common-extensions
```
### Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Create a ROS 2 Workspace

```bash
mkdir -p ~/bookros2_ws/src
cd ~/bookros2_ws/src
git clone https://github.com/eather0056/Robot-Programming-with-ROS2.git
```
<!-- git clone -b foxy-devel https://github.com/fmrico/bookros2.git -->

### Import Third-Party Packages

```bash
cd ~/bookros2_ws/src
vcs import . < Robot-Programming-with-ROS2/third_parties.repos
```

### Install Dependencies and Build Packages

```bash
cd ~/bookros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt update
colcon build --symlink-install
```

### Additional Dependencies

Activate workspace as an overlay

```bash
source ~/bookros2_ws/install/setup.bash
```
Activating by default when opening a terminal

```bash
echo "source ~/bookros2_ws/install/setup.bash" >> ~/.bashrc
```

If you want to skip rebuilding packages that haven't had any changes since the last build in order to save time, you can use

```bash
colcon build --symlink-install --packages-up-to <package_name>
```
You must re-source the workspace since you have created a new program.

See the messages that are published
```bash
ros2 run rqt_console rqt_console
ros2 run rqt_graph rqt_graph
```

if any unwaned node running and overring your current mssages then you migh need to terminate the process ID(`PID`)
```bash
kill -SIGINT <PID>
```
You can find running processes by
```bash
ps aux
```
or specific one like `joystick_relay`
```bash
ps aux | grep joystick_relay
```
## Usage

After installation, you can explore the various packages and launch files included in this repository. Each chapter of the book corresponds to specific packages and examples. Refer to the book for detailed explanations and instructions on running the examples.

## Contributing

Contributions to this repository are welcome. Please submit issues and pull requests with any suggestions, bug reports, or new content you believe would be beneficial for others learning ROS2.
