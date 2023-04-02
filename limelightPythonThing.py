import cv2
import numpy as np

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    # convert the input image to the HSV color space
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # convert the hsv to a binary image by removing any pixels
    # that do not fall within the following HSV Min/Max values
    cone_threshold = cv2.inRange(img_hsv, (22, 210, 255), (22, 255, 255))

   # cone_threshold = cv2.inRange(img_hsv, (22, 210, 205), (22, 255, 255))
   
    # cone_threshold = cv2.inRange(img_hsv, (0, 0, 0), (255, 100, 100))

    # cone_threshold = cv2.inRange(img_hsv, (100, 150, 50), (120, 200, 250))

    # find contours in the new binary image
    cone_contours, _ = cv2.findContours(cone_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    cube_threshold = cv2.inRange(img_hsv, (110, 0, 0), (150, 255, 255))
    
      #  cube_threshold = cv2.inRange(img_hsv, (120, 100, 120), (128, 200, 205))

    # cube_threshold = cv2.inRange(img_hsv, (0, 0, 0), (0, 0, 0))

    # find contours in the new binary image
    cube_contours, _ = cv2.findContours(cube_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    coneContour = np.array([[]])
    cubeContour = np.array([[]])

    coneSize = 0
    cubeSize = 0

    # initialize an empty array of values to send back to the robot
    llpython = [0,0,0,0,0,0,0,0]

    # if contours have been detected, draw them
    if len(cone_contours) > 0:
        cv2.drawContours(image, cone_contours, -1, 255, 2)
        # record the largest contour
        coneContour = max(cone_contours, key=cv2.contourArea)

        # get the unrotated bounding box that surrounds the contour
        x,y,w,h = cv2.boundingRect(coneContour)
        coneSize = w*h
        # # draw the unrotated bounding box
        # cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)

        # # record some custom data to send back to the robot
        # llpython = [1,x,y,w,h,9,8,7]

    # if contours have been detected, draw them
    if len(cube_contours) > 0:
        cv2.drawContours(image, cube_contours, -1, 255, 2)
        # record the largest contour
        cubeContour = max(cube_contours, key=cv2.contourArea)

        # get the unrotated bounding box that surrounds the contour
        x,y,w,h = cv2.boundingRect(cubeContour)
        cubeSize = w*h


    if cubeSize > coneSize and len(cube_contours) and cubeSize > 400:
        x,y,w,h = cv2.boundingRect(cubeContour)
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
        llpython = [2,x,y,w,h,9,8,7]
        return cubeContour, image, llpython
    elif coneSize > cubeSize and len(cone_contours) and coneSize > 400:
        x,y,w,h = cv2.boundingRect(coneContour)
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
        llpython = [1,x,y,w,h,9,8,7]
        return coneContour, image, llpython
    else:
        return cubeContour, image, llpython
    #return the largest contour for the LL crosshair, the modified image, and custom robot data
