"""
Author: Shengyang Zhuang
Date Created: 2024-09-07

Copyright © 2024 Shengyang Zhuang. All rights reserved.
This script is part of the "Multi-Robot System Prototyping for Cooperative Control in Robot-Assisted Spine Surgery" project and is authored solely by Shengyang Zhuang.

Project Website: https://shengyangzhuang.github.io/mres_thesis/
Shengyang Zhuang Personal Website: https://shengyangzhuang.github.io/

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:
    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

# Note: Please retain this header in derivative works.

BibTeX:
@mastersthesis{zhuang2024multirobot,
  author    = {Zhuang, Shengyang},
  title     = {Multi-Robot System Prototyping for Cooperative Control in Robot-Assisted Spine Surgery},
  school    = {Imperial College London},
  year      = {2024},
}
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
