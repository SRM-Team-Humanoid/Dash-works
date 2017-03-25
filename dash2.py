#!/usr/bin/env python
import cv2
import time
import numpy as np
import rospy
from std_msgs.msg import String

def cvis():
    msg = "standby"
    global msg
    cap = cv2.VideoCapture(1)
    width, height, zoom = cap.get(3),cap.get(4),cap.get(5)
    print width, height, zoom
    #cap.set(5,30)
    sides = 150
    g1,g2 = 60,80
    r1,r2 = 90,130
    compare = 2
    maxSize = 500
    dist_thresh = 200
    green1 = np.array([g1,100,100])
    green2 = np.array([g2,255,255])
    red1=np.array([r1,120,100])
    red2=np.array([r2,255,255])

    _,f = cap.read()
    cv2.circle(f,(int(width/2),int(height/2)),5,(0,0,255),-1)
    f = cv2.flip(f,1)
    blur = cv2.medianBlur(f,7)
    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, red1, red2)
    erode = cv2.erode(mask,None,iterations = 1)
    dilate = cv2.dilate(erode,None,iterations = 10)
    mask_outer = cv2.inRange(hsv,green1,green2)
    erode = cv2.erode(mask_outer,None,iterations = 1)
    dilate_outer = cv2.dilate(erode,None,iterations = 10)


    im2,contours2,hierarchy2 = cv2.findContours(dilate_outer,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    im,contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) + len(contours2) == 0:
        # cv2.imshow("c",f)
        return
        # if cv2.waitKey(25) == 27:
        #     break
        # continue
    if len(contours) == 0:
        contourDisplay = None
        contourDisplay2 = max(contours2, key=cv2.contourArea)
    elif len(contours2) == 0:
        contourDisplay2 = None
        contourDisplay = max(contours, key=cv2.contourArea)
    else:
        contoursSorted = sorted(contours, key=cv2.contourArea , reverse=True)
        contoursSorted2 = sorted(contours2, key=cv2.contourArea, reverse=True)

        if len(contoursSorted) > compare:
            contoursSorted = contoursSorted[:compare]
        if len(contoursSorted2) > compare:
            contoursSorted2 = contoursSorted2[:compare]
        contourCenter = []
        contourCenter2 = []
        contours = []
        contours2 = []
        for c in contoursSorted:
            x, y, w, h = cv2.boundingRect(c)
            cxo, cyo = x + w / 2, y + h / 2
            if cv2.contourArea(c) > maxSize:
                contourCenter.append([cxo, cyo])
                contours.append(c)
        for c2 in contoursSorted2:
            x, y, w, h = cv2.boundingRect(c2)
            cxo, cyo = x + w / 2, y + h / 2
            if cv2.contourArea(c2) > maxSize:
                contourCenter2.append([cxo, cyo])
                contours2.append(c2)
        if (len(contours) == 0) or (len(contours2) == 0):
            # cv2.imshow("c", f)
            msg = "standby"
            return
            # if cv2.waitKey(25) == 27:
            #     break
            # continue
        minDis = 10000000
        minContour = 0
        minContour2 = 0

        for i in range(0, len(contourCenter)):
            c = contourCenter[i]
            for j in range(0, len(contourCenter2)):
                c2 = contourCenter2[j]
                dis = pow(pow(c[0] - c2[0], 2) + pow(c[1] - c2[1], 2), 0.5)
                if dis < minDis:
                    minDis = dis
                    minContour = i
                    minContour2 = j
        if minDis > dist_thresh:
            # cv2.imshow("c", f)
            msg = "standby"
            return
            # if cv2.waitKey(25) == 27:
            #     break
            #  continue

        contourDisplay = contours[minContour]
        contourDisplay2 = contours2[minContour2]
        centerx = (contourCenter[minContour][0]+contourCenter2[minContour2][0])/2
        center = width/2
        if centerx > (center + sides):
            msg = "left"
        elif centerx < (center - sides):
            msg = "right"

        else:
            msg = "center"
    # if contourDisplay != None:
    #     x, y, w, h = cv2.boundingRect(contourDisplay)
    #     cx, cy = x + w / 2, y + h / 2
    #     if r1 < hsv.item(cy, cx, 0) < r2:
    #         cv2.rectangle(f, (x, y), (x + w, y + h), [0, 0, 255], 2)
    #         cxo, cyo = x + w / 2, y + h / 2
    #         cv2.circle(f, (cxo, cyo), 5, (0, 0, 0), -1)
    #     cx, cy = x + w / 2, y + h / 2
    # if contourDisplay2 != None:
    #     x1, y1, w1, h1 = cv2.boundingRect(contourDisplay2)
    #     cx1, cy1 = x1 + w1 / 2, y1 + h1 / 2
    #     if g1 < hsv.item(cy1, cx1, 0) < g2:
    #         cv2.rectangle(f, (x1, y1), (x1+w1, y1+h1), [0, 255, ], 2)
    #         cxo1, cyo1 = x1 + w1 / 2, y1 + h1 / 2
    #         cv2.circle(f, (cxo1, cyo1), 5, (0, 0, 0), -1)
    #     cx1, cy1 = x1 + w1 / 2, y1 + h1 / 2
    # cv2.imshow("c",f)
    cv2.waitKey(1)

msg = "standby"

def vision_pub():
    pub = rospy.Publisher('vision_data', String, queue_size=1)
    rospy.init_node('Vision', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cvis()
        print msg
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        vision_pub()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        cap.release()
        pass

