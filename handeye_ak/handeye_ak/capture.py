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


from pyk4a import PyK4A
from PIL import Image
import numpy as np

def capture_jpeg_image(output_filename):
    # Load the Azure Kinect device
    k4a = PyK4A()
    k4a.start()

    # Capture a single image
    capture = k4a.get_capture()
    if capture.color is not None:
        img = Image.fromarray(capture.color[:,:,0:3])  # Convert to PIL Image, discard alpha channel
        img.save(output_filename, "JPEG")  # Save the image as a JPEG file
        print(f"Image successfully saved as {output_filename}")
    else:
        print("Failed to capture image")

    # Cleanup
    k4a.stop()

if __name__ == "__main__":
    capture_jpeg_image("output2.jpg")
