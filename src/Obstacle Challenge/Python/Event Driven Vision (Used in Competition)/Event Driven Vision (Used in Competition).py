#import the required libraries [open computer visison (OpenCV/cv2) library, numpy, time, serial]
import cv2 
import numpy as np
import time
import serial

# Initialization of serial communication with arduino
ser = serial.Serial('/dev/ttyACM0',115200,timeout=1)
ser.flush()
ser.write("1#".encode('ascii'))# Send a handshake signal to ensure that the serial communication is ready
time.sleep(1)

#Initializing a video recording with the camera as the cap object 
cap = cv2.VideoCapture(0)
for i in range(0,10):# Take 10 images to ensure that the camera is correctly reading and processing the scene
    success,img = cap.read()

#### HSV values in terms of (numpy array ranges for (Hue, Saturation, Value)) for read and green 
lower_red = np.array([0,100,50]) 
upper_red = np.array([0,255,255]) 
lower_red2 = np.array([163,100,54])
upper_red2 = np.array([179,255,255])
lower_green = np.array([43, 50, 45])
upper_green = np.array([79, 255, 255])

data = "" #Current data. data variable will store the result of the frame processing, wich represents the detected pillars with the cooresponding colors.

Clock_Wise = False #Round Rotation Direction.
counter = 0
last_counter = 0
first_counter = True
colors_counter = 0
previous_direction = Clock_Wise
colors_array = ["","","",""]# The array in which the pillars information are stored.
ser.write(data.encode('ascii'))
def increase_contrast(image, alpha, beta):## This function is used to adjust the contrast.
    # Apply contrast and brightness adjustments
    adjusted_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return adjusted_image
while True:## This is the loop in which all image processing and communication with arduino is done.
    success,img = cap.read()
    if ser.in_waiting > 0 or first_counter:# If Raspberry Pi recieved a signal from arduino or this is the first section, then begin processing.
            img = increase_contrast(img, 1.2,10)# Adjust the contrast of the image.
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()## Read the incoming data from the arduino.
                previous_direction = Clock_Wise
                if line == "L":## Recived counter clockwise direction.
                    Clock_Wise = False
                elif line == "R":## Recived clockwise direction.
                    Clock_Wise = True
                if Clock_Wise != previous_direction:## In case the robot changed the direction of rotation, then reverse the array of pillars.
                    colors_array.reverse()
                    colors_array[0] = "".join(reversed(colors_array[0]))
                    colors_array[1] = "".join(reversed(colors_array[1]))
                    colors_array[2] = "".join(reversed(colors_array[2]))
                    colors_array[3] = "".join(reversed(colors_array[3]))

                print("line  ===")
                print(line)
            print('---------------------------------------')
            #Find countours of the red and green pillars.
            detected_g = 0
            detected_r = 0
            t = time.time()
            img= cv2.rotate(img, cv2.ROTATE_180)
            image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Convert the image from RGB to HSV

            #Find the masks in the HSV ranges for RED (mask1+mask0), Green (mask2). 
            mask0 = cv2.inRange(image, lower_red, upper_red) 
            mask1 = cv2.inRange(image, lower_red2, upper_red2)
            mask2 = cv2.inRange(image, lower_green, upper_green)

            mask1 = mask1 + mask0 #(+) represents "or" operation.
            

            #find the contours cooresponding to the masks 
            contours1,hierarchy1  = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contours2,hierarchy2  = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
          
            redContours = []
            redCentroids = []
            redAreas = []
            if len(contours1)!= 0:## In case the red contours are not empty (red contours loop).
                for contour in contours1:
                    if cv2.contourArea(contour) > 200:## If the area of contour is larger than a specific value, then process it(This will reduce the noise).
                        x, y , w, h = cv2.boundingRect(contour)
                        area_r = cv2.contourArea(contour)
                        dis_r = 4532.2*pow(area_r,-0.532)## This formula calculates the distance with respect to the contour's area.
                        m = cv2.moments(contour)
                        #calculate the centroid of the pillar
                        cx = int (m["m10"]/m["m00"])
                        cy = int (m["m01"]/m["m00"])
                        detected_r = 1;
                        redContours.append(contour)
                        redCentroids.append([cx, cy, "R", area_r, dis_r])## store the information of the contour (X & Y coordinates, color, area, distance).
                        redAreas.append(area_r)
           
            #print("Green contours")
            greenContours = []
            greenCentroids = []
            greenAreas = []
            if len(contours2)!= 0:#Same steps applied in red contours loop are applied here for green contours.
                for contour in contours2:
                    
                    if cv2.contourArea(contour) > 300:
                        x, y , w, h = cv2.boundingRect(contour)
                        area_g = cv2.contourArea(contour)
                        dis_g = 6165.4*pow(area_g,-0.562)
                        m = cv2.moments(contour)
                        cx = int (m["m10"]/m["m00"])
                        cy = int (m["m01"]/m["m00"])
                        detected_g = 1;
                        greenContours.append(contour)
                        greenCentroids.append([cx, cy, "G", area_g, dis_g])
                        greenAreas.append(area_g)        
                
                
            if (len(greenContours) + len(redContours)) == 3:#In case of detecting three pillars
                allpillars = greenCentroids + redCentroids# Merge the array of red and green contours
                allpillars_areas = sorted(allpillars, key=lambda x:x[3])## Sort the array of contours (pillars) based on area
                if Clock_Wise == False:## If the direction is CCW
                    #The pillar with largest area is assured to be the closest one and within the borders of the current section
                    #The other two may introduce errors in area calculation due to their large distance from the camera, thus the one that is closest to right is in the current section
                    if allpillars_areas[0][0] > allpillars_areas[1][0]:# if the pillar with index 0 is to the right, then ignore the other one and take it.
                        data = str(allpillars_areas[2][2]+allpillars_areas[0][2])
                    else:# if the pillar with index 1 is to the right, then ignore the other one and take it.
                        data = str(allpillars_areas[2][2]+allpillars_areas[1][2])
                else:# Same logic in CCW but in CW this time (Take the one which is closest to the left)
                    if allpillars_areas[0][0] > allpillars_areas[1][0]: #Three Pillars : clock_wise 
                        data = str(allpillars_areas[2][2]+allpillars_areas[1][2])
                    else:
                        data = str(allpillars_areas[2][2]+allpillars_areas[0][2])

            elif (len(greenContours) + len(redContours)) == 2:#In case of detecting two pillars
                allpillars = greenCentroids + redCentroids# Merge the array of red and green contours
                allpillars = sorted(allpillars, key=lambda x:x[0])## Sort the array of contours (pillars) based on X-coordinates
                if min(allpillars[0][4],allpillars[1][4]) < 110:#if there is a pillar in the beginning of the current section - distance from it is less than 110 cm (for both CW and CCW). Then both pillars are in the current section (Take them both).
                    if allpillars[0][3] > allpillars[1][3]:
                        data = str(allpillars[0][2]+allpillars[1][2])
                    else:
                        data = str(allpillars[1][2]+allpillars[0][2])

                else:#if there is no a pillar in the beginning of the current section (for both CW and CCW). Then only take one (This because it is impossible for two pillars to exist in the same section without at least one of them being in the beginning)
                    if Clock_Wise == False:
                        if allpillars[0][0] < allpillars[1][0]:# take the second pillar if it is closer to right than the other
                            data = str(allpillars[1][2])

                        else:
                            data = str(allpillars[0][2])

                    else:
                        if allpillars[0][0] < allpillars[1][0]:# take the second pillar if it is closer to left than the other
                            data = str(allpillars[0][2])
                            print("ftt fia")
                        else:
                            data = str(allpillars[1][2])

            elif len(greenContours)==1 and len(redContours)==0:#One pillar
                data = "G"

            elif len(greenContours)==0 and len(redContours)==1:#One pillar
                data = "R"
        
            if first_counter==False and colors_counter < 4:## In case the robot completed first lap, then use the old stored values
                colors_array[(colors_counter)%4]  = data
                   
               
            print("colors counter = " + str(colors_counter))
            ser.flush()
            if colors_counter < 4:## In case the robot is still in the first lap, then use the old stored values
                signal = data + "#"
            else:## In case the robot completed first lap, then use the old stored values
                print(colors_array)
                signal = colors_array[colors_counter%4] + "#"
            
            ser.write(signal.encode('ascii'))## Send the final decision
            if first_counter==False:
                colors_counter+=1
            time.sleep(0.001)
            
            k = cv2.waitKey(5)
            if k == 27:
                break
            first_counter = False## The robot is not in the first section anymore
cap.release()
cv2.destroyAllWindows()



