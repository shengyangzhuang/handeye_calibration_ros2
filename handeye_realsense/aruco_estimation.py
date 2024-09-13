import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_transformations
from std_msgs.msg import String

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

# Create a QoS profile for subscribing to /tf_static
qos_profile = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# ArUco dictionary lookup
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        with open('src/handeye_realsense/config.yaml', 'r') as file:
            config = yaml.safe_load(file)
        aruco_dictionary_name = config["aruco_dictionary_name"]
        self.aruco_marker_name = config["aruco_marker_name"]
        self.aruco_marker_side_length = config["aruco_marker_side_length"]
        self.camera_calibration_parameters_filename = config["camera_calibration_parameters_filename"]
        self.image_topic = config["image_topic"]
        self.calculated_camera_optical_frame_name = config["calculated_camera_optical_frame_name"]
        self.marker_data_file_name = config["marker_data_file_name"]
        self.image_filename = config["image_filename"]

        # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
            self.get_logger().error(f"ArUCo tag of '{aruco_dictionary_name}' is not supported")
            return

        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        cv_file.release()

        # Load the ArUco dictionary
        self.get_logger().info(f"Detecting '{aruco_dictionary_name}' marker.")
        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()

        # Create the subscriber
        self.subscription = self.create_subscription(Image, self.image_topic, self.listener_callback, 10)
        self.keypress_publisher = self.create_publisher(String, 'keypress_topic', 10)

        self.pose_count = 0

        # Initialize the dynamic and static transform broadcasters
        self.tfbroadcaster = TransformBroadcaster(self)
        self.static_tfbroadcaster = StaticTransformBroadcaster(self)

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()


    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """ Convert a quaternion into a full three-dimensional rotation matrix. """
        return R.from_quat([x, y, z, w]).as_matrix()    


    def publish_transform(self, translation_vector, rotation_matrix, frame_id, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        # Set the translation
        t.transform.translation.x = translation_vector[0]
        t.transform.translation.y = translation_vector[1]
        t.transform.translation.z = translation_vector[2]

        # Convert the rotation matrix to a quaternion
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation_vector
        quaternion = tf_transformations.quaternion_from_matrix(transformation_matrix)

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tfbroadcaster.sendTransform(t)

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data)
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_side_length, self.mtx, self.dst)
            
            for i, marker_id in enumerate(marker_ids):
                # Create the coordinate transform
                t_marker_to_camera = TransformStamped()
                t_marker_to_camera.header.stamp = self.get_clock().now().to_msg()
                t_marker_to_camera.header.frame_id = self.calculated_camera_optical_frame_name
                t_marker_to_camera.child_frame_id = self.aruco_marker_name

                # Store the translation (i.e. position) information
                t_marker_to_camera.transform.translation.x = tvecs[i][0][0]
                t_marker_to_camera.transform.translation.y = tvecs[i][0][1]
                t_marker_to_camera.transform.translation.z = tvecs[i][0][2]


                # Store the rotation information
                rotation_matrix = cv2.Rodrigues(rvecs[i][0])[0]
                r = R.from_matrix(rotation_matrix)
                quat = r.as_quat()

                # Quaternion format
                t_marker_to_camera.transform.rotation.x = quat[0]
                t_marker_to_camera.transform.rotation.y = quat[1]
                t_marker_to_camera.transform.rotation.z = quat[2]
                t_marker_to_camera.transform.rotation.w = quat[3]

                # Draw the axes on the marker
                cv2.drawFrameAxes(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)

                # Send the transform from marker to the camera
                self.tfbroadcaster.sendTransform(t_marker_to_camera)

        # Create a named window that can be resized
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        # Resize the window to the desired size, e.g., 800x600 pixels
        cv2.resizeWindow("camera", 700, 500)

        cv2.imshow("camera", current_frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.save_marker_data(rvecs, tvecs)
            self.save_image(current_frame)
            self.get_logger().info(f"Saved pose_{self.pose_count} marker transform.")
            self.keypress_publisher.publish(String(data='q'))
        elif key == ord('e'):
            self.get_logger().info("Ending program...")
            self.keypress_publisher.publish(String(data='e'))
            cv2.destroyAllWindows()  # Close all OpenCV windows
            rclpy.shutdown()  # Shutdown ROS client library for Python

    def save_marker_data(self, rvecs, tvecs):
        yaml_file_path = self.marker_data_file_name  # Filename to save the YAML data
        try:
            with open(yaml_file_path, 'r') as file:
                data = yaml.safe_load(file) or {'poses': []}  # Use existing data or initialize if empty
        except FileNotFoundError:
            data = {'poses': []}  # Initialize if file does not exist

        for rvec, tvec in zip(rvecs, tvecs):
            R_mat = cv2.Rodrigues(rvec)[0]
            marker_data = {
                'rotation': R_mat.tolist(),
                'translation': tvec[0].tolist()
            }
            data['poses'].append(marker_data)

        with open(yaml_file_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)

        self.pose_count += 1
        self.get_logger().info(f"Pose {self.pose_count}:")
        self.get_logger().info("Rotation Matrix:\n" + str(R_mat))
        self.get_logger().info("Translation Vector:\n" + str(tvec[0]))
        self.get_logger().info(f'Transformation for Pose {self.pose_count} appended to marker_data_ak.yaml')

    def save_image(self, frame):
        image_filename = self.image_filename.format(pose_count=self.pose_count)
        cv2.imwrite(image_filename, frame)
        self.get_logger().info(f'Image saved as {image_filename}')


def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    try:
        rclpy.spin(aruco_node)
    finally:
        aruco_node.destroy_node()

if __name__ == '__main__':
    main()
