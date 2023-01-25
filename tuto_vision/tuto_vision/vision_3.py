import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node


# Initialize camera
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of orange and black colors in HSV
    orange_lower = (5, 180, 180)
    orange_upper = (30, 255, 255)
    black_lower = (25, 25, 25)
    black_upper = (45, 60, 60)
    color_info = (0, 0, 250)

    # Threshold the frame to get only orange and black colors
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    black_mask = cv2.inRange(frame, black_lower, black_upper)

    # Bitwise-AND mask and original frame
    orange_res = cv2.bitwise_and(frame, frame, mask=orange_mask)
    black_res = cv2.bitwise_and(frame, frame, mask=black_mask)

    # Detect shapes in the frames
    orange_cnts = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    black_cnts = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Draw the contours on the frames
    for c in orange_cnts:
        M = cv2.moments(c)
        if (M["m00"] != 0):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>100:
             cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
             cv2.rectangle(frame, (cX, cY), (cX+10, cY+15),color_info, 2)
             cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
             cv2.line(frame, (cX, cY), (cX+150, cY), color_info, 2)
             cv2.putText(frame, "Orange Bottle !!!", (cX+10, cY-10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

    for c in black_cnts:
        M = cv2.moments(c)
        if (M["m00"] != 0):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>100:
             cv2.drawContours(black_res, [c], -1, (0, 255, 0), 2)
             cv2.circle(black_res, (cX, cY), 7, (255, 255, 255), -1)
             cv2.line(frame, (cX, cY), (cX+150, cY), color_info, 2)
             cv2.putText(frame, "Black Bottle !!!", (cX+10, cY-10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

    # Show the frames
    cv2.imshow('Orange', orange_res)
    cv2.imshow('Black', black_res)
    cv2.imshow('Camera', frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera
cap.release()
cv2.destroyAllWindows()