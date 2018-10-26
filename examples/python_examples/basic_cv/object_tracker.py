#!/usr/bin/env python2.7
import numpy as np
import sys
import rospy
import cv2
from time import sleep
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from Tkinter import *

class object_tracker:

  def __init__(self):
    self.tk_init()
    # Connect ROS to OpenCV, set the minimum tracked object size
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/geekbot/webcam/image_raw/compressed", CompressedImage, self.callback)
    self.centroid_pub = rospy.Publisher("/tracker/centroid", Point, queue_size=10)
    self.min_px_area = (50*50) 

  def update_gui(self):
      # Update Tk things
      self.root.update()
      # Update the threshold values
      self.l_h = self.low_h.get()
      self.l_s = self.low_s.get()
      self.l_v = self.low_v.get()
      self.h_h = self.high_h.get()
      self.h_s = self.high_s.get()
      self.h_v = self.high_v.get()

  def tk_init(self):
    self.root = Tk()
    self.window = Canvas(self.root, width=640, height=480+160, bd = 10, bg = 'white')
    Label(self.root, text="Low  Hue:").grid(column=0, row=1)
    self.low_h = Scale(self.root, from_=0, to=180, length=480, orient=HORIZONTAL)
    self.low_h.grid( column=1, row=1)
    Label(self.root, text="Low  Sat:").grid(column=0, row=2)
    self.low_s = Scale(self.root, from_=0, to=255, length=480, orient=HORIZONTAL)
    self.low_s.grid( column=1, row=2)
    Label(self.root, text="Low  Val:").grid(column=0, row=3)
    self.low_v = Scale(self.root, from_=0, to=255, length=480, orient=HORIZONTAL)
    self.low_v.grid( column=1, row=3)
    Label(self.root, text="High Hue:").grid(column=0, row=4)
    self.high_h = Scale(self.root, from_=0, to=180, length=480, orient=HORIZONTAL)
    self.high_h.grid( column=1, row=4)
    Label(self.root, text="High Sat:").grid(column=0, row=5)
    self.high_s = Scale(self.root, from_=0, to=255, length=480, orient=HORIZONTAL)
    self.high_s.grid( column=1, row=5)
    Label(self.root, text="High Val:").grid(column=0, row=6)
    self.high_v = Scale(self.root, from_=0, to=255, length=480, orient=HORIZONTAL)
    self.high_v.grid( column=1, row=6)
    # Initialize these values in case the callback gets pulled before Tk updates
    self.l_h = self.low_h.get()
    self.l_s = self.low_s.get()
    self.l_v = self.low_v.get()
    self.h_h = self.high_h.get()
    self.h_s = self.high_s.get()
    self.h_v = self.high_v.get()

  def callback(self,data):
    # Start by grabbing a new image from the stream
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Set HSV bounds
    low_color = np.array([self.l_h, self.l_s, self.l_v])
    high_color = np.array([self.h_h, self.h_s, self.h_v])

    # Convert stream image from RBG to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Pull boolean mask of everything between HSV bounds
    mask = cv2.inRange(hsv_image, low_color, high_color)
    # Filter out some noise
    mask = self.denoise_mask(mask)
    # Use the mask to mask the original RGB image for display
    thresholded = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # In the HSV mask from above find the connected contours/blobs. If
    # there is at least one contour, grab all contours larger than 
    # min_px_area. Find the biggest contour, draw it and a circle on centroid.  
    cont_img,contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
      largest = self.largest_contour(contours)
      if largest != None:
        cv2.drawContours(thresholded, largest, -1, (0,255,0), 3)
        centroid_x,centroid_y = self.contour_centroid(largest)
        cv2.circle(thresholded, (int(centroid_x), int(centroid_y)), 10, [0, 0, 255])
        centroid = Point()
        centroid.x = centroid_x
        centroid.y = centroid_y
        self.centroid_pub.publish(centroid)

    # Update the displayed images
    cv2.namedWindow("Thresholded", cv2.WINDOW_NORMAL)
    cv2.imshow("Thresholded", thresholded)
    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
    cv2.imshow("Original", cv_image)
    # Wait a bit for user input
    cv2.waitKey(10)

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
    if cv2.contourArea(contours[largest_index]) < self.min_px_area: #None larger than min_px_area
      return None
    else:
      return contours[largest_index]

  # Erode small objects, inflate them again, return the denoised mask
  # These are ballparked values, change as necessary
  def denoise_mask(self, input):
    temp = cv2.erode(input, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)), iterations=1)
    temp = cv2.dilate(input, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1)
    return temp

def main(args):
  rospy.init_node('object_tracker', anonymous=True)
  ot = object_tracker()
  while not rospy.is_shutdown():
    ot.update_gui()
    sleep(0.016) # Let the OS do other things, update @ ~60Hz
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
