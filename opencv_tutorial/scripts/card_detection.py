#!/usr/bin/env python
# license removed for brevity
import cv2
# to detect face

faceCascade = cv2.CascadeClassifier(r"/home/user/catkin_ws/src/opencv_ros_tutorial/haarcascades/haarcascade_frontalface_default.xml")
img = cv2.imread(r"/home/user/catkin_ws/src/opencv_ros_tutorial/lena.png")
imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

faces = faceCascade.detectMultiScale(imgGray, 1.1, 4)

for (x, y, w, h) in faces:
    cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
    cv2.circle(img, ( (x+x+w)//2, (y+y+h)//2 ), 1, (255,0,0), 5, -1)
    cv2.putText(img, "C:(" + str((x+x+w)//2) + "," + str((y+y+h)//2) + ")", ( (x+x+w)//2 - 30, (y+y+h)//2 - 8), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,0,0), 1)

cv2.imshow("Result", img)
cv2.waitKey(0)