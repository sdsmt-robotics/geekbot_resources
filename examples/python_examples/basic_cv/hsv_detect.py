#!/usr/bin/env python2.7
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# KNOWN BUG: ROS uses multithreading to make sure all the pub-sub mumbo jumbo is working
# on the system. There is a known deadlocking bug in the as-distributed-by-kinetic version 
# of the python2.7 threading library which has not been and will not be fixed. This program
# will crash often and for seemingly no reason. Maybe this deadlock could be avoided
# if the object detection was not contained in a class. Probably not though. 
# If you would like to debug further please do so!
# More info can be found here:  https://github.com/mkorpela/pabot/issues/146
#                               https://bugs.python.org/issue10394
 

class object_tracker:

  def nothing(self, placeholder):
    pass

  def __init__(self):
    # Connect ROS to OpenCV, set the minimum tracked object size
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/geekbot/webcam/image_raw/compressed", CompressedImage, self.callback)
    self.min_px_area = (100*100)

  def callback(self,data):
    # Start by grabbing a new image from the stream
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Should NOT be placed in the callback, but is a workaround for 
    # completely different OpenCV bug
    cv2.namedWindow("Parameters", cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar('Low H','Parameters',0,180,self.nothing)
    cv2.createTrackbar('Low S','Parameters',0,255,self.nothing)
    cv2.createTrackbar('Low V','Parameters',0,255,self.nothing)
    cv2.createTrackbar('High H','Parameters',0,180,self.nothing)
    cv2.createTrackbar('High S','Parameters',0,255,self.nothing)
    cv2.createTrackbar('High V','Parameters',0,255,self.nothing)

    # Update the threshold values
    low_h = cv2.getTrackbarPos('Low H','Parameters')
    low_s = cv2.getTrackbarPos('Low S','Parameters')
    low_v = cv2.getTrackbarPos('Low V','Parameters')
    high_h = cv2.getTrackbarPos('High H','Parameters')
    high_s = cv2.getTrackbarPos('High S','Parameters')
    high_v = cv2.getTrackbarPos('High V','Parameters')

    # Set HSV bounds
    low_color = np.array([low_h, low_s, low_v])
    high_color = np.array([high_h, high_s, high_v])

    # Convert stream image from RBG to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Pull boolean mask of everything between HSV bounds
    mask = cv2.inRange(hsv_image, low_color, high_color)
    # Filter out some noise
    mask = self.denoise_mask(mask)
    # Use the mask to mask the original RGB image for display
    thresholded = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # In the HSV mask from above find the connected contours/blobs. If
    # there is at least one contour, grab all bounding boxes of those contours
    # larger than min_px_area. Find the biggest box, draw it and a circle on center. 
    # This can also be reworked to use contours instead of bounding boxes. 
    cont_img,contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
      #largest = self.largest_contour(contours)
      #cv2.drawContours(thresholded, largest, -1, (0,255,0), 3)
      #centroid_x,centroid_y = self.contour_centroid(largest)
      boxes = self.get_bounding_boxes(contours)
      if(len(boxes)) > 1:
        x,y,w,h = self.xywh_largest_box(boxes)
        cv2.rectangle(thresholded,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.circle(thresholded, (x+w/2, y+h/2), 10, [0, 0, 255])
        # At this point you know the position of the object in frame. 
        # Maybe turn the robot left or right if the object is too far left/right?

    # Update the displayed images
    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
    cv2.imshow("Original", cv_image)

    cv2.imshow("Parameters", thresholded)
    # Let the images update
    cv2.waitKey(5)

  # Find the larges bounding box given a list of boxes in tuple (x,y,w,h) form,
  # return (x,y,w,h) tuple of larges box
  def xywh_largest_box(self, boxes):
    largest_area = 0
    largest_index = 0
    for i in range(len(boxes)):
      xywh = boxes[i]
      height = xywh[3]
      width = xywh[2]
      area = width*height
      if area > largest_area:
        largest_area = area
        largest_index = i
    return boxes[largest_index]
      
  # Given a list of contours, return all bounding boxes larger than min_px_area
  def get_bounding_boxes(self, contours):
    rects = []
    for i in range(len(contours)):
      rect = cv2.boundingRect(contours[i])
      rect_area = rect[2]*rect[3]
      if rect_area > self.min_px_area:
        rects.append(tuple(rect))
    return rects

  # Find the centroid of the given contour. Watch out for /0
  def contour_centroid(self, contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return cx, cy

  # Given a list of contours, return the largest
  def largest_contour(self, contours):
    largest = contours[0]
    largest_index = 0
    for i in range(len(contours)):
      current = contours[i]
      largest = contours[largest_index]
      if cv2.contourArea(current) > cv2.contourArea(largest):
        largest_index = i
    return contours[largest_index]

  # Erode small objects, inflate them again, return the denoised mask
  # These are ballparked values, change as necessary
  def denoise_mask(self, input):
    temp = cv2.erode(input, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)), iterations=1)
    temp = cv2.dilate(input, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1)
    #temp = cv2.dilate(input, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1)
    #temp = cv2.erode(input, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1)
    return temp

def main(args):
  ot = object_tracker()
  rospy.init_node('object_tracker', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
