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

#blue and red
myColors = [[90,0,64,162,255,255], #[100,150,0,140,255,255]
            [0,116,71,82,255,255]] 

#BGR format (blue and red)
myColorValues = [[255,0,0],
                 [0,0,255]]

class image_converter:
            
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera2/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    
    imgResult = findColor(cv_image, myColors, myColorValues)
    
    # imgContour = cv_image.copy()
    # imgGray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # imgBlur = cv2.GaussianBlur(imgGray, (7, 7), 1)
    # imgCanny = cv2.Canny(imgGray, 50, 50)
    # imgResult = getContours(imgCanny, cv_image)
    # imgResult = imgContour.copy()

    cv2.imshow("Camera 2 Feed", imgResult)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def findColor(img, myColors, myColorValues):
    imgOrg = img
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # count = 0
    # newPoints=[]
    for color in myColors:
        lower = np.array(color[0:3])
        upper = np.array(color[3:6])
        mask = cv2.inRange(imgHSV, lower, upper)
        imgResult = getContours(mask, imgOrg)
        # cv2.circle(imgResult, (x,y), 15, myColorValues[count], cv2.FILLED)
        # if x!=0 and y!=0:
        #     newPoints.append([x,y,count])
        # count +=1
        # cv2.imshow(str(color[0]), mask)
    return imgResult


def getContours(imgRead, imgOrg):
    _, contours, _ = cv2.findContours(imgRead, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        print(area)
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
                objectType = "Foreign Object"  #changed
            else:
                objectType = "None"

            cv2.putText(imgOrg, objectType, (x + (w // 2) - 10, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (255, 255, 255), 2)

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
