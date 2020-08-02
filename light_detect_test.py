#!/usr/bin/env python3
from __future__ import print_function
import cv2
import numpy as np
import statistics
import sys
import rospy
from std_msgs.msg import String,Float64,Int64,Int8
import roslib
roslib.load_manifest('usb_cam')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

lower_red = np.array([0, 50, 150])
upper_red = np.array([5, 255, 255])

lower_green = np.array([80,0,40])
upper_green = np.array([190,255,255])

#lower_green = np.array([103,0,100])
#upper_green = np.array([125,255,255])

lower_yellow = np.array([20, 50,70])
upper_yellow = np.array([50, 100, 255])

#######################################
lower_yellow1 = np.array([20, 50,110])
upper_yellow1 = np.array([50, 100, 170])

lower_white = np.uint8([0, 200, 0])
upper_white = np.uint8([255, 255, 255])

Servo_morter = 0
Servo_morter1 = 0

slope = 1
intercept = 1
xpointbot = 0
xpointtop = 0 
xpointmid = 0
x_C = [0, 0, 0]
x_C1 = [0, 0, 0]

B1 = [0,0]
B2 = [0,0]
B3 = [0,0]
B4 = [0,0]

# Multiplier from full y-axis -- for white line
multiply_y = [0.9,0.8,0.7]
Size_crop = [0,0]
blobs = [0, 0, 0]
blob_c = [0, 0, 0]
keypoint = [0, 0, 0]
# For yellow line
multiply_y1 = [0.9,0.8,0.7]
Size_crop1 = [0,0]
blobs1 = [0, 0, 0]
blob_c1 = [0, 0, 0]
keypoint1 = [0, 0, 0]

kernel = np.ones((3,3), np.uint8)
MODE_COLOR = 1 #0 yellow, 1 white
trig_mode=0 #0 left, 1 right


def intitial():
    #rospy.Subscriber("/usb_cam/image_raw",Image,trafficgreen)
    rospy.Subscriber("/usb_cam/image_raw",Image,Getcam)


# def trafficgreen(data):
#   cv = bridge.imgmsg_to_cv2(data, "bgr8")
#   global run
#   y1=70
#   y2=120
#   x1=370
#   x2=420
#   kernel_traff = np.ones((5,5),np.float32)/25
#   crop_frame_traff = frame[12:57 , 152:253]
#   res = cv2.filter2D(frame,-1,kernel)
#   hsv = cv2.cvtColor(crop_frame_traff, cv2.COLOR_BGR2HSV) 
#   mask_traff = cv2.inRange(hsv, lower_red, upper_red)
#   mask2_traff = cv2.inRange(hsv, lower_green, upper_green)        
#   #mask = cv2.bitwise_or(mask_red,mask_green)
#   #masked = cv2.bitwise_and(crop_frame ,crop_frame, mask = mask)
  
#   red_traffic = cv2.bitwise_and(crop_frame_traff,crop_frame_traff,mask = mask_traff)
#   green_traffic = cv2.bitwise_and(crop_frame_traff,crop_frame_traff,mask= mask2_traff)
#   #rospy.loginfo(np.sum(red_traffic)/255)
#   #print(np.sum(red_traffic)/255)
#   if (np.sum(red_traffic)/255) > 500 :
#     run = 0
#     result_traff = red_traffic
#   elif (np.sum(green_traffic)/255) > 100 :
#     run = 1
#     result_traff = green_traffic
#   else :
#     run = 2
#     result_traff = hsv
#   cv2.imshow('normal', frame)
#   cv2.imshow('light', result)
#   #cv2.waitKey(3)
#   print(run)

# def send_angle(Servo_morter):
#     pub.publish(Servo_morter)
#     rospy.loginfo(Servo_morter)


def Getcam(data): #WHITE
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    global x_mid
    global y_bot
    global y_top
    global y_mid
    global main_Y
    global x_mid1
    global y_bot1
    global y_top1
    global y_mid1
    global main_Y1
    global MODE_COLOR
    global run
    point(cv_image)
    #pts1 = np.float32([[B1], [B2], [B3], [B 4]])
    #pts1 = np.float32([[67,87],[157,92],[0,180],[163,180]])
    #cv2.imshow('raw',cv_image)

    # PERSPECTIVE POINT
    pts1 = np.float32([[int(66),int(196)],[int(440),int(185)],[0,int(220)],[int(490),int(208)]]) #tl,tr,bl,br
    pts2 = np.float32([[0, 0], [Size_crop[0], 0], [0, Size_crop[1]], [Size_crop[0], Size_crop[1]]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    result = cv2.warpPerspective(cv_image, matrix, (Size_crop[0], Size_crop[1]))
    # result1 = cv2.warpPerspective(cv_image, matrix, (Size_crop[0], Size_crop[1]))
    cv2.imshow('perspect',result)
    # cv2.imshow('perspect1',result1)

    # Crop to cal
    result1 = Crop_to_Cal1(result)
    result = Crop_to_Cal(result)
    #cv2.imshow('kkk',result)
    result = cv2.resize(result, (0,0), fx=4, fy=4) #left
    result1 = cv2.resize(result1, (0,0), fx=4, fy=4) #right

    # MODE COLOR
    hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV) #left=white
    mask = cv2.inRange(hsv, lower_yellow1, upper_yellow1)

    hsv1 = cv2.cvtColor(result1, cv2.COLOR_BGR2HSV) #right=yellow
    mask1 = cv2.inRange(hsv1, lower_yellow, upper_yellow)

    # MASK COLOR
    crop = cv2.bitwise_and(result, result, mask=mask)
    crop = cv2.morphologyEx(crop, cv2.MORPH_OPEN, kernel)
    crop = cv2.dilate(crop,kernel,iterations = 1)
    crop = cv2.morphologyEx(crop, cv2.MORPH_CLOSE, kernel)

    crop1 = cv2.bitwise_and(result1, result1, mask=mask1)
    crop1 = cv2.morphologyEx(crop1, cv2.MORPH_OPEN, kernel)
    crop1 = cv2.dilate(crop1,kernel,iterations = 1)
    crop1 = cv2.morphologyEx(crop1, cv2.MORPH_CLOSE, kernel)

    # BLOB WHITE PTS
    x_mid = int(result.shape[1]*0.5)
    y_bot = int(result.shape[0]*(multiply_y[0]+0.025))
    y_top = int(result.shape[0]*(multiply_y[2]+0.025))
    y_mid = int((y_bot+y_top)/2)
    main_Y = [result.shape[0]*(multiply_y[0]+0.025),
                result.shape[0]*(multiply_y[1]+0.025),
                result.shape[0]*(multiply_y[2]+0.025)]
    cv2.line(result, (int(result.shape[1]*0.5), y_bot),(int(result.shape[1]*0.5), y_top), (0, 0, 255), 1,  8)
    blobs[0], keypoint[0] = bloblane(crop, multiply_y[0],10)
    blobs[1], keypoint[1] = bloblane(crop, multiply_y[1],10)
    blobs[2], keypoint[2] = bloblane(crop, multiply_y[2],10)
    blobby(keypoint,0)
    blobby(keypoint,1)
    blobby(keypoint,2)
    BLOB=np.concatenate((blobs[2],blobs[1],blobs[0]),axis=0)
    Detect_Curve(result)
    Servo_morter0 = Cal_Servo(result,x_C[2])
    result =np.concatenate((result,BLOB),axis=0)
    crop = cv2.resize(BLOB, (0,0), fx=0.5, fy=0.5)
    result = cv2.resize(result, (0,0), fx=0.5, fy=0.5)

    # BLOB YELLOW PTS
    x_mid1 = int(result1.shape[1]*0.5)
    y_bot1 = int(result1.shape[0]*(multiply_y[0]+0.025)) #<----------------------
    y_top1 = int(result1.shape[0]*(multiply_y[2]+0.025))
    y_mid1 = int((y_bot1+y_top1)/2)
    main_Y1 = [result1.shape[0]*(multiply_y[0]+0.025),
                result1.shape[0]*(multiply_y[1]+0.025),
                result1.shape[0]*(multiply_y[2]+0.025)]
    cv2.line(result1, (int(result1.shape[1]*0.5), y_bot1),(int(result1.shape[1]*0.5), y_top1), (0, 0, 255), 1,  8)

    blobs1[0], keypoint1[0] = bloblane(crop1, multiply_y[0],10) #<-------------------
    blobs1[1], keypoint1[1] = bloblane(crop1, multiply_y[1],10)
    blobs1[2], keypoint1[2] = bloblane(crop1, multiply_y[2],10)
    blobby1(keypoint1,0)
    blobby1(keypoint1,1)
    blobby1(keypoint1,2)
    BLOB1=np.concatenate((blobs1[2],blobs1[1],blobs1[0]),axis=0)
    Detect_Curve1(result1)
    Servo_morter1 = Cal_Servo(result1,x_C1[2])
    result1 =np.concatenate((result1,BLOB1),axis=0)
    crop1 = cv2.resize(BLOB1, (0,0), fx=0.5, fy=0.5)
    result1 = cv2.resize(result1, (0,0), fx=0.5, fy=0.5)
    
    
    cv2.circle(cv_image,(int(66),int(196)),1,(0,0,255),2)#1
    cv2.circle(cv_image,(int(571),int(223)),1,(0,0,255),2)#2
    cv2.circle(cv_image,(0,int(220)),1,(0,0,255),2)#3
    cv2.circle(cv_image,(int(609),int(232)),1,(0,0,255),2)#4



    cv2.circle(cv_image,(int(465),int(4)),1,(200,50,100),2)
    cv2.circle(cv_image,(int(542),int(104)),1,(200,50,100),2)
    cv2.imshow('frame',cv_image[0:int(cv_image.shape[0]),0:int(cv_image.shape[1])])
    cv2.imshow('roi',result)
    cv2.imshow('roi1',result1)
    cv2.imshow('color',crop)
    cv2.imshow('color1',crop1)
    cv2.waitKey(1)
    pub = rospy.Publisher('/angle', Float64, queue_size=3)
    Servo_morter = (Servo_morter1) # mean value
    # print(Servo_morter)
    ###################################################
   
    pub.publish(Servo_morter)
    rospy.loginfo(Servo_morter)
    #global run

    ##################################################
    y1=70
    y2=120
    x1=370
    x2=420
    kernel_traff = np.ones((5,5),np.float32)/25
    #cv2.circle(cv_image,(int(cv_image.shape[1]*0.70),int(cv_image.shape[0]*0.26)),1,(0,0,255),2)
    crop_frame_traff = cv_image[4:104 , 465:542]
    res = cv2.filter2D(cv_image,-1,kernel_traff)
    hsv_traff = cv2.cvtColor(crop_frame_traff, cv2.COLOR_BGR2HSV) 
    mask_traff = cv2.inRange(hsv_traff, lower_red, upper_red)
    mask2_traff = cv2.inRange(hsv_traff, lower_green, upper_green)        
    #mask = cv2.bitwise_or(mask_red,mask_green)
    #masked = cv2.bitwise_and(crop_frame ,crop_frame, mask = mask)
  
    red_traffic = cv2.bitwise_and(crop_frame_traff,crop_frame_traff,mask = mask_traff)
    green_traffic = cv2.bitwise_and(crop_frame_traff,crop_frame_traff,mask= mask2_traff)
    #rospy.loginfo(np.sum(red_traffic)/255)
    
    print('green',np.sum(green_traffic)/255)
    print('red',np.sum(red_traffic)/255)
    if (np.sum(red_traffic)/255) >= 50 :
      run = 0
      result_traff1 = red_traffic
    if (np.sum(green_traffic)/255) > 1500 :
      run = 1
      result_traff1 = green_traffic
    #cv2.imshow('normal', cv_image)
    pub_traff = rospy.Publisher('/light', Int8, queue_size=1)
    cv2.imshow('light2', result_traff1)
    pub_traff.publish(run)
    return result,crop,result1,crop1


def point(cv_image):
    global B1
    global B2
    global B3
    global B4
    global Size_crop
    B1[0] = int(cv_image.shape[1]*0.3125)
    B2[0] = int(cv_image.shape[1]*0.6875)
    B3[0] = int(cv_image.shape[1]*0)
    B4[0] = int(cv_image.shape[1]*1)
    B1[1] = int(cv_image.shape[0]*0.45)
    B2[1] = int(cv_image.shape[0]*0.45)
    B3[1] = int(cv_image.shape[0]*0.694)
    B4[1] = int(cv_image.shape[0]*0.694) 
    Size_crop[0] = int(cv_image.shape[1]*0.4)
    Size_crop[1] = int(cv_image.shape[0])

def Crop_to_Cal(result): #left
    # global trig_modqe
    A1 = [0 , int(result.shape[0])]
    A2 = [int(result.shape[1]*0.20) , int(result.shape[0])]
    A3 = [int(result.shape[1]*0.9), int(result.shape[0]*0.85)]
    A4 = [int(result.shape[1]) , int(result.shape[0]*0.6)]
    crop_img = result[A3[1]:A1[1], A1[0]:A2[0]] #left
    return crop_img

def Crop_to_Cal1(result): #right
    # global trig_modqe
    A1 = [0 , int(result.shape[0])]
    A2 = [int(result.shape[1]*0.15) , int(result.shape[0])]
    A3 = [int(result.shape[1]*0.75), int(result.shape[0]*0.85)]
    A4 = [int(result.shape[1]) , int(result.shape[0]*0.6)]
    crop_img = result[A3[1]:A1[1], A3[0]:A4[0]] #right
    return crop_img

def bloblane(image, y_point,minblobs):
    if minblobs == 0:
        minblobs  = image.shape[1]*0.08
    xp =  int(image.shape[1])
    yp1 = int(image.shape[0] * y_point)
    yp2 = int(image.shape[0] * (y_point+0.05))
    crop = image[yp1:yp2, 0:xp]
    crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByInertia = False
    params.filterByConvexity = False
    params.filterByColor = False
    params.minArea = minblobs
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(crop)
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(crop, keypoints, blank, (0, 0, 255),
    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return blobs, keypoints


def Find_slope(point_X, point_Y, y_1, y_2):
    global slope
    global intercept
    global xpointbot
    global xpointtop
    global xpointmid
    sum_1 = 0
    sum_2 = 0    
    y_mid = (y_1+y_2)/2
    x_bar = statistics.mean(point_X)
    y_bar = statistics.mean(point_Y)    
    for i in range(3):
        sum_1 = sum_1 + ((point_X[i]-x_bar) * (point_Y[i]-y_bar))
        sum_2 = sum_2 + ((point_X[i]-x_bar)**2)
    if sum_2 != 0:
        slope = sum_1/sum_2
    if slope != 0:
        intercept = y_bar - (slope*x_bar)
        xpointbot = (y_1-intercept)/slope
        xpointtop = (y_2-intercept)/slope
        xpointmid = (y_mid-intercept)/slope
    return xpointbot, xpointtop, xpointmid


def Detect_Curve(result):
    global x_C
    x_C[0], x_C[1], x_C[2] = Find_slope(blob_c, main_Y, y_bot, y_top,)   
    cv2.line(result, (int(x_C[2]), y_mid),(int(result.shape[1]*0.5), y_mid), (180, 0, 255), 1,  8)
    cv2.line(result, (int(x_C[0]), y_bot),(int(x_C[1]), y_top), (20,255,150), 2,  8)

def Detect_Curve1(result):
    global x_C1
    x_C1[0], x_C1[1], x_C1[2] = Find_slope(blob_c1, main_Y1, y_bot1, y_top1,)   
    cv2.line(result, (int(x_C1[2]), y_mid1),(int(result.shape[1]*0.5), y_mid1), (180, 0, 255), 1,  8)
    cv2.line(result, (int(x_C1[0]), y_bot1),(int(x_C1[1]), y_top1), (20,255,150), 2,  8)    


def Cal_Servo(result,mid_point):
    Servo_morter = (mid_point/result.shape[1])*100
    if Servo_morter < 0:
        Servo_morter=0
    if Servo_morter > 100:
         Servo_morter=100
    return Servo_morter


def blobby(key_point,x):
    global blob_c
    if len(keypoint[x])==1:
        blob_c[x] = keypoint[x][0].pt[0]
    else :
        pass

def blobby1(key_point,x):
    global blob_c1
    if len(keypoint1[x])==1:
        blob_c1[x] = keypoint1[x][0].pt[0]
    else :
        pass



def main(args):
  rospy.init_node('light_node', anonymous=True)
  intitial()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)