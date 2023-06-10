# Ball Detection Using OpenCV
This code demonstrates object tracking using OpenCV. It captures video from a webcam and performs the following steps for each frame:
1. Apply background subtraction using the MOG2 algorithm to obtain the foreground mask.
2. Perform morphological opening to remove noise and small details from the foreground mask.
3. Convert the frame from BGR color space to LAB color space.
4. Define a color range for yellow objects in LAB color space and create a mask by thresholding.
5. Apply the color mask to the foreground mask using bitwise AND operation.
6. Reduce noise by applying Gaussian blur to the foreground mask.
7. Erode the image to remove small details and noise.
8. Dilate the image to enhance the object and remove gaps between areas of points.
9. Find contours in the thresholded image.
10. Determine the largest contour based on contour area.
11. If a contour is found and its area is greater than zero:
       - Calculate the enclosing circle and its center.
       - Draw the circle and center on the frame.
12.Display the original frame and the foreground mask.
