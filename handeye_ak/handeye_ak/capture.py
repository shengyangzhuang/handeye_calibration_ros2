"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
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
