"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
import cv2
import numpy as np
import pyrealsense2 as rs

#POSE_NAME = 'shifted_1'
#POSE_NAME = 'shifted'
#POSE_NAME = 'original'
POSE_NAME = 'cali_10'

def main():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    # different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Show images
            cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

        # Save the last captured image
        cv2.imwrite(f"{POSE_NAME}.jpg", color_image)

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    main()
