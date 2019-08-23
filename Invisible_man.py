import cv2
import numpy as np
import time

print(cv2.__version__)
#activates the webcam in system
capture_video = cv2.VideoCapture(0)

#activates a secondary cam in your phone using IP Cam app
cap = cv2.VideoCapture('http://192.168.137.60:8080/video')

#give the camera to warm up
time.sleep(1) 
count = 0 
background = 0 

#capturing the background in range of 60

# we are reading from video 
while (capture_video.isOpened() and cap.isOpened()):
    
    # Numpy to convert into a array
    
    # Finally decode the array to OpenCV usable format ;) 
    return_val, background = cap.read()

    #resize your background stream so that both streams will be the same
    if return_val == True:
        background = cv2.resize(background,(480,360),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
    
    if not return_val :
        break
    
    background = np.flip(background , axis=1)

    #resize your front mask stream so that both streams will be the same
	
    return_val, img = capture_video.read()
    
    if return_val == True:
        img = cv2.resize(img,(480,360),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
    
    if not return_val :
        break 
    count = count + 1
    img = np.flip(img , axis=1)
	# convert the image - BGR to HSV
	# as we focused on detection of red color 
    hsv = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)
	# generating mask to detect red color
	# HSV
	# it should be mono-color cloth 
	# lower range
    lower_red = np.array([100, 40, 40])
    upper_red = np.array([100, 255, 255])
    mask1 = cv2.inRange(hsv,lower_red,upper_red)
    lower_red = np.array([155, 40, 40])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv,lower_red,upper_red)
    
    mask1 = mask1+mask2

    # Refining the mask corresponding to the detected red color
    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, np.ones((3,3),np.uint8),iterations=2)
    mask1 = cv2.dilate(mask1,np.ones((3,3),np.uint8),iterations = 1)
    mask2 = cv2.bitwise_not(mask1)

    # Generating the final output
    res1 = cv2.bitwise_and(background,background,mask=mask1)
    res2 = cv2.bitwise_and(img,img,mask=mask2)
    final_output = cv2.addWeighted(res1,1,res2,1,0)
    cv2.imshow("INVISIBLE MAN",final_output)
    k = cv2.waitKey(10)
    if k == 27:
        break






