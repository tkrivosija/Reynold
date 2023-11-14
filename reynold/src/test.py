#!/usr/bin/env python3
import cv2

# Load the BMP image using OpenCV
image_path = '/home/developer/catkin_ws/src/sphero_simulation/sphero_stage/resources/maps/empty_3x3/empty_3x3.bmp'  # Replace with your BMP image path
bmp_image = cv2.imread(image_path)

if bmp_image is not None:
    # Convert the image to a matrix representation
    image_matrix = bmp_image.tolist()  # Convert the image to a nested list (matrix)

    # Print the size of the matrix representation
    print(f"Matrix Size: {len(image_matrix)} rows x {len(image_matrix[0])} columns")

    # Access pixel values (e.g., print the pixel value at row 0, column 0)
    print(f"Pixel value at (0, 0): {image_matrix[0][0]}")
    #for row in image_matrix:
    #    for pixel in row:
    #        print(pixel)
else:
    print("Failed to load the image.")
