# Pixhawk Controller

## Overview

The Pixhawk Controller is a ROS 2 package designed to interface with the Pixhawk flight controller. It provides functionality for logging IMU data and communicating with ground control stations (GCS) via MAVLink.

## Installation

Pre-requisites:

- Make sure you have ROS 2 Humble installed and sourced.
- Mavros should also be installed and configured.

Clone the repository:

```bash
git clone https://github.com/anggamys/pixhawk_controller.git
```

Install dependencies:

```bash
cd pixhawk_controller
rosdep install -i --from-path src --rosdistro humble -y
```

Build the package:

```bash
colcon build --symlink-install --packages-select pixhawk_controller
```

## Usage

Run mavros for communication with the Pixhawk:

```bash
# Example
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:115200 gcs_url:=udp://@:14555
```

Run the Pixhawk controller:

```bash
ros2 run pixhawk_controller pixhawk_controller_node
```

> Change `pixhawk_controller_node` to specify the desired node.
> You can also run the IMU logger node to log IMU data:
>
> ```bash
> ros2 run pixhawk_controller imu_logger_node
> ```
