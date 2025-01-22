"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml

class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration_node')
        self.get_logger().info("Starting Hand-Eye Calibration Node")

        # Load transformation data from YAML files
        self.R_gripper2base, self.t_gripper2base = self.load_transformations('/home/szhuang/handeye_calibration_ws/robot_data_ak.yaml')
        self.R_target2cam, self.t_target2cam = self.load_transformations('/home/szhuang/handeye_calibration_ws/marker_data_ak.yaml')

        # Compute the hand-eye transformation matrix
        self.compute_hand_eye()

    def load_transformations(self, file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            poses = data['poses']

        # Initialize to handle yaml data format
        R = []
        t = []

        for pose in poses:
            rotation = np.array(pose['rotation'], dtype=np.float32)
            translation = np.array(pose['translation'], dtype=np.float32)

            R.append(rotation)
            t.append(translation)

        #R = [np.array(data['rotation'], dtype=np.float32) for pose in poses]
        #t = [np.array(data['translation'], dtype=np.float32) for pose in poses]
        #self.get_logger().info(f"Loaded {len(R)} rotations and {len(t)} translations from {file_path}")
        return R, t

    def compute_hand_eye(self):
        self.get_logger().info(f"Loaded {len(self.R_gripper2base)} rotations and {len(self.t_gripper2base)} translations for gripper to base")
        self.get_logger().info(f"Loaded {len(self.R_target2cam)} rotations and {len(self.t_target2cam)} translations for target to camera")
        # Prepare data for calibration (rotation matrixes not vectors)
        #rotations = [cv2.Rodrigues(r)[0] for r in self.R_gripper2base]
        #translations = [t.reshape((3, 1)) for t in self.t_gripper2base]
        #obj_rotations = [cv2.Rodrigues(r)[0] for r in self.R_target2cam]
        #obj_translations = [t.reshape((3, 1)) for t in self.t_target2cam]
        rotations = [r.reshape(3, 3) for r in self.R_gripper2base]
        translations = [t.reshape(3, 1) for t in self.t_gripper2base]
        obj_rotations = [r.reshape(3, 3) for r in self.R_target2cam]
        obj_translations = [t.reshape(3, 1) for t in self.t_target2cam]
        print(f"R_gripper2base: {rotations}")
        print(f"t_gripper2base: {translations}")
        print(f"R_target2cam: {obj_rotations}")
        print(f"t_target2cam: {obj_translations}")


        # Perform hand-eye calibration
        R, t = cv2.calibrateHandEye(
            rotations, translations, obj_rotations, obj_translations,
            method=cv2.CALIB_HAND_EYE_TSAI)

        # Save results to YAML
        # Output: camera relative to gripper frame (eye to hand)
        self.save_yaml(R, t)

    def save_yaml(self, R, t):
        data = {'rotation': R.flatten().tolist(), 'translation': t.flatten().tolist()}
        with open('/home/szhuang/handeye_calibration_ws/hand_eye_result_ak.yaml', 'w') as file:
            yaml.safe_dump(data, file)
        self.get_logger().info("Simulated hand-eye calibration results saved.")
        #print(f"Rotation matrix: {R.flatten().tolist()}")
        #print(f"Translation matrix: {t.flatten().tolist()}")
        print(f"Rotation matrix: {R}")
        print(f"Translation matrix: {t}")


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
