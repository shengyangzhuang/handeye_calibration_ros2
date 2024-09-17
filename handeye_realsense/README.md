# handeye_realsense

This is a ROS2 handeye calibration package for RealSense depth camera. Here we provided a more detailed example for the usage. 

## Example with RealSense D415
1. **Camera and robot setup**

    Prepare a camera holder. This image shows two camera holders example designed for KUKA LBR iiwa 7/14 and RealSense, Azure Kinect depth camera.

<img src="https://github.com/user-attachments/assets/55bc2b9f-5eca-410b-9d8d-54c2c2df5c2b" alt="Camera and robot setup" width="500"/>

2. **Camera intrinsic calibration**

   Calibrate the camera matrix `K` and distortion coefficient `D`. This can be done with the `camera_calibration` package we provided by using a chessboard. If you use RealSense/Azure Kinect camera, you can also get the camera parameters directly with `ros2 topic echo "/specific_camera_topic"`.

   When you are sure camera parameters are correct, you can record it in the `handeye_realsense/realsense_info.yaml`.

3. **Specifying the parameters of your robot**

   Under `handeye_realsense/config.yaml` file,
