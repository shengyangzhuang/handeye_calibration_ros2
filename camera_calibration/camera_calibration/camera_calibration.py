"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
from __future__ import print_function # Python 2/3 compatibility 
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
import glob # Used to get retrieve files that have a specified pattern
  
# Chessboard dimensions
number_of_squares_X = 12 # Number of chessboard squares along the x-axis
number_of_squares_Y = 9  # Number of chessboard squares along the y-axis
nX = number_of_squares_X - 1 # Number of interior corners along x-axis
nY = number_of_squares_Y - 1 # Number of interior corners along y-axis
square_size = 0.025 # Size, in meters, of a square side 
  
# Set termination criteria. We stop either when an accuracy is reached or when
# we have finished a certain number of iterations.
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 
 
# Define real world coordinates for points in the 3D coordinate frame
# Object points are (0,0,0), (1,0,0), (2,0,0) ...., (5,8,0)
object_points_3D = np.zeros((nX * nY, 3), np.float32)  
  
# These are the x and y coordinates                                              
object_points_3D[:,:2] = np.mgrid[0:nY, 0:nX].T.reshape(-1, 2) 
 
object_points_3D = object_points_3D * square_size
 
# Store vectors of 3D points for all chessboard images (world coordinate frame)
object_points = []
  
# Store vectors of 2D points for all chessboard images (camera coordinate frame)
image_points = []
  
def main():
      
  # Get the file path for images in the current directory
  images = glob.glob('*.jpg')
  print(f"Found images: {images}")  # This will print the list of found images

    # If no images are found, exit the function
  if not images:
    print("No images found. Check the file path and image format.")
    return
      
  # Go through each chessboard image, one by one
  for image_file in images:
   
    # Load the image
    image = cv2.imread(image_file)  
  
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
  
    # Find the corners on the chessboard
    success, corners = cv2.findChessboardCorners(gray, (nY, nX), None)
      
    # If the corners are found by the algorithm, draw them
    if success == True:
  
      # Append object points
      object_points.append(object_points_3D)
  
      # Find more exact corner pixels       
      corners_2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)       
        
      # Append image points
      image_points.append(corners_2)
  
      # Draw the corners
      cv2.drawChessboardCorners(image, (nY, nX), corners_2, success)
  
      # Display the image. Used for testing.
      cv2.imshow("Image", image) 
      
      # Display the window for a short period. Used for testing.
      cv2.waitKey(1000) 
                                                                                                                      
  # Perform camera calibration to return the camera matrix, distortion coefficients, rotation and translation vectors etc 
  ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, 
                                                    image_points, 
                                                    gray.shape[::-1], 
                                                    None, 
                                                    None)
 
  # Save parameters to a file
  cv_file = cv2.FileStorage('calibration_chessboard_azure_kinect.yaml', cv2.FILE_STORAGE_WRITE)
  cv_file.write('K', mtx)
  cv_file.write('D', dist)
  cv_file.release()
  
  # Load the parameters from the saved file
  cv_file = cv2.FileStorage('calibration_chessboard_azure_kinect.yaml', cv2.FILE_STORAGE_READ) 
  mtx = cv_file.getNode('K').mat()
  dst = cv_file.getNode('D').mat()
  cv_file.release()
   
  # Display key parameter outputs of the camera calibration process
  print("Camera matrix:") 
  print(mtx) 
  
  print("\n Distortion coefficient:") 
  print(dist) 
    
  # Close all windows
  cv2.destroyAllWindows() 
      
if __name__ == '__main__':
  print(__doc__)
  main()