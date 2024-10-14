# handeye_realsense

This is a ROS2 handeye calibration package for RealSense depth camera. Here we provided a more detailed example for the usage. 

A video of the full process can be viewed [here](https://youtu.be/EUSxnzzP8qk?si=RNuauInFx9T1qH7X)

## Example with RealSense D415
1. **Camera and robot setup**

    Prepare a camera holder. This image shows two camera holders example designed for KUKA LBR iiwa 7/14 and RealSense, Azure Kinect depth camera.
   
    <p align="left">
        <img src="https://github.com/user-attachments/assets/55bc2b9f-5eca-410b-9d8d-54c2c2df5c2b" alt="Camera and robot setup" width="500"/>
    </p>
    Prepare an ArUco marker. Attach the camera to the robot.

    <p align="left">
      <img src="https://github.com/user-attachments/assets/8ed1cc2a-9e33-48e4-9ff9-ca81aa78c576" alt="aruco" height="200"/>
      <img src="https://github.com/user-attachments/assets/dea609c1-006a-4650-b64e-a695c212bde5" alt="realsense_setup" height="200"/>
    </p>

3. **Camera intrinsic calibration**

   Calibrate the camera matrix `K` and distortion coefficient `D`. This can be done with the `camera_calibration` package we provided by using a chessboard. If you use RealSense/Azure Kinect camera, you can also get the camera parameters directly with `ros2 topic echo "/specific_camera_topic"`.

   When you are sure camera parameters are correct, you can record it in the `handeye_realsense/realsense_info.yaml`.

4. **Specifying the parameters of your robot**

   Under `handeye_realsense/config.yaml` file, modify the `aruco_marker_side_length`, `image_topic`, and relevant robot link names as per your setups.

5. **Bring up the robot and camera**

   If you use lbr_fri_ros2_stack for KUKA LBR robots, please refer to the guideline [here](https://github.com/lbr-stack/lbr_fri_ros2_stack)

   Start the RealSense camera ROS2 driver

   ```
    ros2 run realsense2_camera realsense2_camera_node
   ```

7. **Taking samples**
    You can use moveit, custom control script, or admittance control mode to guide the robot around to take unique samples of the camera poses.

    Go into your handeye calibration workspace and launch the calibration file

   ```
    cd handeye_calibration_ws
    colcon build --packages-select handeye_realsense
    source install/setup.bash
    ros2 launch taking_sample_launch.py
   ```
    <p align="left">
        <img src="https://github.com/user-attachments/assets/e226deb3-02cd-4fde-a34d-0f211de4c859" alt="cali_experiment_combined" width="500"/>
    </p>

8. **Computing and publishing the result**

   To compute the eye-in-hand calibration result

   ```
   ros2 run handeye_realsense handeye
   ```

   To publish the result and visualize in rviz

   ```
   ros2 run handeye_realsense eye2hand
   ```

   <p align="left">
        <img src="https://github.com/user-attachments/assets/db236acb-715a-4ad3-9ada-81123a5c0d1a" alt="cali_experiment_combined" width="500"/>
    </p>
