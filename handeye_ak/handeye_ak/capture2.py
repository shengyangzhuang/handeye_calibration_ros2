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


from pyk4a import PyK4A, Config
from PIL import Image
import numpy as np
import logging

# Setup basic configuration for logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


def init_kinect():
    # Set the configuration with a valid color resolution
    config = Config(
        color_resolution=1080,  # Ensure this is a valid resolution as per SDK documentation
    )
    k4a = PyK4A(config=config)
    k4a.start()
    return k4a

def capture_color_image(k4a):
    """Capture a single color image from the given Kinect device."""
    capture = k4a.get_capture()
    if capture.color is not None:
        return Image.fromarray(capture.color[:, :, 0:3])  # Discard alpha channel
    else:
        raise RuntimeError("Failed to capture color image")

def save_image(image, filename):
    """Save the image to a file."""
    image.save(filename, "JPEG")
    logging.info(f"Image successfully saved as {filename}")

def main(output_filename):
    try:
        k4a = init_kinect()
        try:
            img = capture_color_image(k4a)
            save_image(img, output_filename)
        finally:
            k4a.stop()
    except Exception as e:
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    main("output2.jpg")
