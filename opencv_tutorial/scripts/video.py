#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('opencv_tutorial')
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
            
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)
    
    # imgContour = cv_image.copy()
    imgGray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # imgBlur = cv2.GaussianBlur(imgGray, (7, 7), 1)
    imgCanny = cv2.Canny(imgGray, 50, 50)
    imgResult = getContours(imgCanny, cv_image)
    # imgResult = imgContour.copy()

    cv2.imshow("Image window", imgResult)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def getContours(imgRead, imgOrg):
    _, contours, _ = cv2.findContours(imgRead, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(area)
        # to check the area of contour and remove the noises
        if area > 500:
            # drawing the contour
            cv2.drawContours(imgOrg, cnt, -1, (255, 0, 0), 3)  #changed
            peri = cv2.arcLength(cnt, True)
            # print(peri)

            # approximating the number of edges based on the arc
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(len(approx))
            # object corner
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            if objCor == 3:
                objectType = "Triangle"
            elif objCor == 4:
                aspRatio = w / float(h)
                if aspRatio > 0.95 and aspRatio < 1.05:
                    objectType = "Square"
                else:
                    objectType = "Rectangle"
            elif objCor > 4:
                objectType = "Circle"
            else:
                objectType = "None"

            cv2.putText(imgOrg, objectType, (x + (w // 2) - 10, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 0, 0), 2)

            # useful to get the middle point
            cv2.rectangle(imgOrg, (x, y), (x + w, y + h), (0, 255, 0), 2)  #changed
    return imgOrg

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
    
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
