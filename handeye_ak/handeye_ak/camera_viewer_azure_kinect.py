"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
import cv2
import numpy as np
from pyk4a import PyK4A, Config, ColorResolution, DepthMode

# Pose name setting
POSE_NAME = 'cali_10'

def main():
    # Load camera with specific configuration
    k4a = PyK4A(
        Config(
            color_resolution=ColorResolution.RES_720P,
            depth_mode=DepthMode.OFF,  # Disable depth capture
            synchronized_images_only=False,  # No need for synchronization as depth is not used
        )
    )

    # Start the camera
    k4a.start()

    try:
        while True:
            # Get the next capture (only color frame)
            capture = k4a.get_capture()

            if capture.color is not None:
                # Color image is a numpy array
                color_image = capture.color[..., :3]  # Get BGRA color and strip alpha channel

                # Display the color image
                cv2.imshow('Azure Kinect', color_image)

                # Check for user input to close the program
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break

        # Save the last captured image
        cv2.imwrite(f"{POSE_NAME}.jpg", color_image)

    finally:
        # Stop the camera
        k4a.stop()

if __name__ == "__main__":
    main()
