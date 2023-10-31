import cv2
import numpy as np
from time import sleep
import time
#import imutils
import serial
ser = serial.Serial('/dev/ttyACM0',115200,timeout=1)
ser.flush()
ser.write ("1#".encode('ascii'))
time.sleep(0.5)
cap = cv2.VideoCapture(0)
img1 = 0
lower_red = np.array([0,160,50])
upper_red = np.array([10,255,255])
lower_red2 = np.array([150,100,0])
upper_red2 = np.array([179,255,255])

lower_green = np.array([36, 36, 0])
upper_green = np.array([99, 255, 255])
last = "null"
dis = 0
first_distance = 1
while True:
   success, img = cap.read()
   if (img is not None):

      

      #croping HSV image for ROI
      img[380:500, 0:700] = 0

	  #taking images & rotation 180 degrees for raspberry pi
      img_rotated = cv2.rotate(img, cv2.ROTATE_180)
      
      #Bluring & Smoothening
      img_blur = cv2.GaussianBlur(img_rotated, (5, 5), 0)

      #convert original image from RGB to HSV
      img_hsv  = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

      # lower mask (0-10) for RED | mask 0
      
      mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

      # upper mask (170-180) for RED | mask 1
      
      mask1 = cv2.inRange(img_hsv, lower_red2, upper_red2)

      # join my masks | combining masks together
      mask = mask0+mask1

      #green mask
      
      mask_green = cv2.inRange(img_hsv,lower_green,upper_green)
      # set my output img to zero everywhere except my mask | time & storage saving
      output_img = img.copy()
      output_img[np.where(mask==0)] = 0
      output_img_green = img.copy()
      output_img_green[np.where(mask_green==0)]=0

      #image openning | morpholoy
      kernel = np.ones((5,5),np.uint8)

      output_img = cv2.dilate(output_img,kernel,iterations = 1)

      output_img = cv2.morphologyEx(output_img, cv2.MORPH_OPEN, kernel)

      output_img_green = cv2.dilate(output_img_green,kernel,iterations = 1)

      output_img_green = cv2.morphologyEx(output_img_green, cv2.MORPH_OPEN, kernel)

      #converting image to gray scale

      #RED RED
      imgray = cv2.cvtColor(output_img, cv2.COLOR_BGR2GRAY)
      #cv2.imwrite('imggray.jpg',imgray)

      ret, thresh = cv2.threshold(imgray, 255, 255 , 255)
      #cv2.imwrite('thresh.jpg',thresh)

      #GREEN GREEN
      imgray_green = cv2.cvtColor(output_img_green, cv2.COLOR_BGR2GRAY)
      #cv2.imwrite('imggray_green.jpg', imgray_green)

      ret_g, thresh_g = cv2.threshold(imgray_green, 255, 255, 255)
      #cv2.imwrite('thresh.jpg', thresh_g)

      #finding *RED* contours using cv2

      contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      #finding GREEN contours
      contours_green , hierarchy_green = cv2.findContours(thresh_g , cv2.RETR_TREE , cv2.CHAIN_APPROX_SIMPLE)
      Area = -1
      final_contour = None
      x_center = -1
      y_center = -1
      per = -1
      x_bounding = -1
      y_bounding = -1
      h_bounding = -1
      w_bounding = -1
      flag = "no color"
      for i, cnt in enumerate(contours):
         M = cv2.moments(cnt)
         if M['m00'] != 0.0:
            x1 = int(M['m10']/M['m00'])
            y1 = int(M['m01']/M['m00'])
         area_contour = cv2.contourArea(cnt)
         perimeter = cv2.arcLength(cnt, True)
         perimeter = round(perimeter, 4)
         x , y,w,h = cv2.boundingRect(cnt)
         area_rect = w*h
         if Area < (area_contour + area_rect)/2:
            Area = (area_contour + area_rect)/2
            final_contour = cnt
            x_center = x1
            y_center = y1
            per = perimeter
            x_bounding = x
            y_bounding = y
            h_bounding = h
            w_bounding = w
            flag = "RED"
      # Green contour processing
      for i, cntg in enumerate(contours_green):
         x, y, w, h = cv2.boundingRect(cntg)
         area_rect = w * h
         area_contour = cv2.contourArea(cntg)
         if (area_contour + area_rect)/2<Area:
            continue
         else:
            M = cv2.moments(cntg)
            if M['m00'] != 0.0:
               x1 = int(M['m10'] / M['m00'])
               y1 = int(M['m01'] / M['m00'])
               perimeter = cv2.arcLength(cntg, True)
               perimeter = round(perimeter, 4)

               if Area < (area_contour + area_rect) / 2:
                  Area = (area_contour + area_rect) / 2
                  final_contour = cntg
                  x_center = x1
                  y_center = y1
                  per = perimeter
                  x_bounding = x
                  y_bounding = y
                  h_bounding = h
                  w_bounding = w
                  flag = "GREEN"
      
      if flag=="RED":
          dis = 4532.2 * pow(Area, -0.532)
          if dis > 30 and last != 'R':
              ser.write("R#".encode('ascii'))
              print("R")
              last = 'R'
          if dis < 30 and last != 'r':
              ser.write("r#".encode('ascii'))
              print("r")
              last = 'r'
      elif flag=="GREEN":
          dis = 6165.4 * pow(Area, -0.562)
          if dis > 30 and last != 'G':
              ser.write("G#".encode('ascii'))
              print("G")
              last = 'G'
          if dis < 30 and last != 'g':
              ser.write("g#".encode('ascii'))
              print("g")
              last = 'g'
      elif flag=="no color" and last != 'N':
          ser.write("N#".encode('ascii'))
          last = "N"
          print("N")
    
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break
      #time.sleep(0.1)
      
cap.release()
cv2.destroyAllWindows()