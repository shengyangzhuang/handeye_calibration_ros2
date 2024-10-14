# handeye_calibration_ros2

This is a ROS2 handeye calibration package for RealSense and Azure Kinect depth camera.

<p align="left">
  <img src="https://github.com/user-attachments/assets/019c12a0-e329-48bf-8f3a-67fe79dc05a4" alt="sim_positions_1" width="400"/>
  <img src="https://github.com/user-attachments/assets/4ba7f0e9-954b-41c7-acd0-fe94134deabf" alt="real_positions_1" width="400"/>
</p>

## Table of Contents
1. [Overview](#overview)
2. [Installation](#installation)
3. [Usage](#usage)
4. [Example](#example)
5. [Contributing](#contributing)
6. [Citation](#citation)

## Overview
This package provides a solution for hand-eye calibration between a robot arm and a camera in a ROS2 environment. It is designed to support various robotic and vision systems, offering flexibility for different hardware configurations.

## Installation

1. Create your workspace and clone this repository:
    ```bash
    mkdir -p handeye_calibration_ws/src
    cd handeye_calibration_ws/src
    git clone https://github.com/shengyangzhuang/handeye_calibration_ros2.git
    ```

2. Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the package

    RealSense:
        ```
        colcon build --packages-select handeye_realsense
        ```

    Azure Kinect:
        ```
        colcon build --packages-select handeye_ak
        ```

4. Source your ROS2 workspace:
    ```
    source install/setup.bash
    ```
## Prerequisits

1. Install the [RealSense ROS2 wrapper](https://github.com/IntelRealSense/realsense-ros).

## Usage
1. Calibrate the camera intrinsics and record the intrinsic matrix `K` and distortion coefficient `D` at `realsense_info.yaml`. We also provided a package using the chessboard to calibrate the camera intrinsics, see `camera_calibration`.

2. Change the parameters at `config.yaml`

3. Start the camera node

    RealSense:
    ```bash
    ros2 run realsense2_camera realsense2_camera_node
    ```

4. To start the hand-eye calibration, launch the node as follows and press key **q** to record marker and robot poses:
    ```bash
    ros2 launch handeye_realsense taking_sample_launch.py
    ```

5. To calculate the hand-eye calibration result:
    ```bash
    ros2 run handeye_realsense handeye
    ```

6. To visualize the result in rviz, publish the hand-eye transformation by:
    ```bash
    ros2 run handeye_realsense eye2hand
    ```

## Example

We provided an example usage with RealSense camera. Please refer [**here**](https://github.com/shengyangzhuang/handeye_calibration_ros2/tree/main/handeye_realsense).

A video of hand-eye calibration process can be seen [**here**](https://youtu.be/EUSxnzzP8qk?si=HIvo1J5uhRv7PXBF), where we demonstrated ensuring calibration accuracy through simulation and replicating the similar poses in real LBR iiwa 7 robot and RealSense depth camera.

## Contributing

We welcome contributions from the community! If you'd like to contribute, please follow these steps:

1. **Fork the repository**:
   Click the "Fork" button at the top of this repository to create a copy on your GitHub account.

2. **Create a feature branch**:
   Clone the forked repository to your local machine and create a new branch for your feature or bug fix:
   ```bash
   git checkout -b feature/my-feature
   ```
3. **Make your changes**: 
   Implement your changes in the code, and don't forget to write tests if applicable.

4. **Commit your changes**: 
   Once your changes are ready, commit them with a descriptive commit message:
    ```bash
    git commit -m 'Add some feature'
    ```
   Push your changes: Push your changes to the branch on your forked repository:
    ```bash
    git push origin feature/my-feature
    ```
   Open a Pull Request: 
   Go to the original repository on GitHub and open a Pull Request (PR). Make sure to describe your changes in detail and explain why they are necessary.

## Citation
If you enjoyed using this package, we would really appreciate it if you could leave a ⭐ and / or cite it!

```
@mastersthesis{zhuang2024multirobot,
  author    = {Zhuang, Shengyang},
  title     = {Multi-Robot System Prototyping for Cooperative Control in Robot-Assisted Spine Surgery},
  school    = {Imperial College London},
  year      = {2024},
}
```
