"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
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
