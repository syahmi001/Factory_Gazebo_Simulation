import cv2
import numpy as np

frameWidth = 600
frameHeight = 440
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10,150)

#blue, yellow, orange
myColors = [[109,50,0,122,146,255], #109,122,50,146,0,255  85,140,17,119,0,255
            [20,54,127,50,254,255], #20,50,54,254,127,255
            [0,101,141,10,255,255]] #0,10,101,255,141,255

#BGR format
myColorValues = [[153,76,0],
                 [0,255,255],
                 [0,128,255]]

myPoints = [] #[x, y, colorId]


def findColor(img, myColors, myColorValues):
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    count = 0
    newPoints=[]
    for color in myColors:
        lower = np.array(color[0:3])
        upper = np.array(color[3:6])
        mask = cv2.inRange(imgHSV, lower, upper)
        x,y = getContours(mask)
        cv2.circle(imgResult, (x,y), 15, myColorValues[count], cv2.FILLED)
        if x!=0 and y!=0:
            newPoints.append([x,y,count])
        count +=1
        # cv2.imshow(str(color[0]), mask)
    return newPoints

def getContours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    x,y,w,h = 0,0,0,0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # to check the area of contour and remove the noises
        if area > 500:
            # drawing the contour
            # cv2.drawContours(imgResult, cnt, -1, (255, 0, 0), 3)
            #perimeter
            peri = cv2.arcLength(cnt, True)

            # approximating the number of edges based on the arc
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)

            # finding center to draw
        return x+w//2, y

def drawOnCanvas(myPoints, myColorValues):
    for point in myPoints:
        cv2.circle(imgResult, (point[0], point[1]), 10, myColorValues[point[2]], cv2.FILLED)

while True:
    success, img = cap.read()
    imgResult = img.copy()
    newPoints = findColor(img, myColors, myColorValues)
    if len(newPoints)!=0:
        for newP in newPoints:
            myPoints.append(newP)
    if len(myPoints)!=0:
        drawOnCanvas(myPoints, myColorValues)

    cv2.imshow("Video", imgResult)
    if cv2.waitKey(1) and 0xFF ==ord('q'):
        break