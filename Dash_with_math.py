import cv2
import time
import numpy as np
#import serial
#ser = serial.Serial("COM5",9600)
#prev = 0
#cxg,cyg,cxo,cyo = 0,0,0,0
'''cap = cv2.VideoCapture(2)
w,h,z = cap.get(3),cap.get(4),cap.get(5)
print w,h,z
#cap.set(5,30)
green1 = np.array([60,100,100])
green2 = np.array([80,255,255])
red1=np.array([0,100,100])
red2=np.array([7,255,255])'''
def cvis():
    cap = cv2.VideoCapture(1)
    w,h,z = cap.get(3),cap.get(4),cap.get(5)
    print w,h,z
    #cap.set(5,30)
    g1,g2 = 60,71
    r1,r2 = 160,180
    green1 = np.array([g1,20,20])
    green2 = np.array([g2,190,190])
    red1=np.array([r1,100,100])
    red2=np.array([r2,255,255])
    while(1):

        _,f = cap.read()
        f = cv2.flip(f,1)
        blur = cv2.medianBlur(f,1)
        hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, red1, red2)
        erode = cv2.erode(mask,None,iterations = 1)
        dilate = cv2.dilate(erode,None,iterations = 10)
        mask_outer = cv2.inRange(hsv,green1,green2)
        erode = cv2.erode(mask_outer,None,iterations = 1)
        dilate_outer = cv2.dilate(erode,None,iterations = 10)


        im2,contours2,hierarchy2 = cv2.findContours(dilate_outer,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        im,contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            cx, cy = x + w / 2, y + h / 2
            if r1 < hsv.item(cy,cx,0) < r2:
                cv2.rectangle(f,(x,y),(x+w,y+h),[0,0,255],2)
                cxo, cyo = x + w / 2, y + h / 2
                cv2.circle(f, (cxo, cyo), 5, (0,0,0), -1)
            cx, cy = x + w / 2, y + h / 2

        if len(contours2):
            c2 = max(contours2,key = cv2.contourArea)
            x1, y1, w1, h1 = cv2.boundingRect(c2)
            cx1, cy1 = x1 + w1 / 2, y1 + h1 / 2
            if g1 < hsv.item(cy1,cx1,0) < g2:
                cv2.rectangle(f,(x1,y1),(x1+w1,y1+h1),[0,255,],2)
                cxo1, cyo1 = x1 + w1 / 2, y1 + h1 / 2
                cv2.circle(f, (cxo1, cyo1), 5, (0,0,0), -1)
            cx1, cy1 = x1 + w1 / 2, y1 + h1 / 2
        cv2.imshow("c",f)
        if cv2.waitKey(25) == 27:
            break
    cv2.destroyAllWindows()
    cap.release()

cvis()
