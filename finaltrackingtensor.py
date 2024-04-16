############################################################
# Loading all libraries
############################################################
print("Loading libraries...")
import cv2
import numpy as np
import time
import torch
from ultralytics import YOLO
import sys
import serial
from functions import *
print("Loading libraries complete!!!")

############################################################
# Function to calculate delta X and delta
############################################################
def delta_xy(imgcenter_x, imgcenter_y, center_x, center_y, classID):
    delta_x = imgcenter_x - center_x
    delta_y = imgcenter_y - center_y
 
    rounded_delta_x = round(delta_x, 3)
    rounded_delta_y = round(delta_y, 3)
 
    if delta_x > 0.0:
        direction = "L"
    else:
        direction = "R"

    return direction, rounded_delta_x, rounded_delta_y

############################################################
# Loading the model and camera here
############################################################

print("Loading model...")
model = YOLO("weights/mergedGatePics.pt", "v8").cuda() 
print("Loading model complete!!!")

#ARDUINO
print("Loading arduino...")
#arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
print("Loading arduino complete")

# for using webcam
cam_port = 0
#cap = cv2.VideoCapture(cam_port)

# for video input
cap = cv2.VideoCapture("data/videos/7.mp4")
 
if not cap.isOpened():
    print("Cannot open camera")
    exit()
 
prev_frame_time = 0
new_frame_time = 0
count = 0
RATIO = 52

############################################################
# Main loop begins here
############################################################
while True:
    ret, frame = cap.read()
    
    #frame = cv2.flip(frame, 1)
    image_height, image_width = frame.shape[0], frame.shape[1]
    #print("current image_height, image_width:", image_height, image_width)
    frame = cv2.resize(frame, (frame.shape[1]//2, frame.shape[0]//2))
    
    image_height, image_width = frame.shape[0], frame.shape[1]
    #print("new image_height, image_width:", image_height, image_width)
    
    imgcenter_x = image_width / 2
    imgcenter_y = image_height / 2
 
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    new_frame_time = time.time()
    # Convert frame to PyTorch tensor
    # frame_tensor = torch.from_Loading model complete!!!
                                                         
    
    detect_params = model.predict(source=frame, conf=0.3, save=False)
    
    
    # Drawing quadrants
    cv2.line(frame, (int(imgcenter_x), int(0)), (int(imgcenter_x), int(image_height)), (250, 250, 250), 2)
    cv2.line(frame, (int(0), int(imgcenter_y)), (int(image_width), int(imgcenter_y)), (250, 250, 250), 2)
   
    if len(detect_params[0]) != 0:
        for i in range(len(detect_params[0])):
            boxes = detect_params[0].boxes
            box = boxes[i]
            
            clsID = box.cls.cpu().numpy()[0]
            conf = box.conf.cpu().numpy()[0]
            bb = box.xyxy.cpu().numpy()[0]
            print(clsID)
            
            cv2.rectangle(frame, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), (0, 255, 0), 3)
            
            center_x = (bb[0] + bb[2]) / 2
            center_y = (bb[1] + bb[3]) / 2
            
            
            cv2.line(frame, (int(center_x), int(center_y)), (int(imgcenter_x), int(imgcenter_y)), (255, 255, 255), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 2, (0, 0, 255), -1)
            cv2.circle(frame, (int(imgcenter_x), int(imgcenter_y)), 2, (0, 0, 255), -1)
            
            #direction, diff = delta_xy(imgcenter_x, imgcenter_y, center_x, center_y, clsID)
            direction, diffX, diffY = delta_xy(imgcenter_x, imgcenter_y, center_x, center_y, clsID)        
            delX = diffX
            
            val = (delX*150)/(bb[0] + bb[2])

            #printing the distance
            if((bb[0] + bb[2]) < 400):
                distance = (bb[0] + bb[2])/RATIO
                print("Distance", distance)            
            
            count = count + 1
            timer = 20 #int(sys.argv[1])
            if(count%timer == 0):
                if (val/25 >= 1):
                    print("Go Diagonal Left")
                    arr = diagonal_L(0)
                    msg = ",".join(str(ele) for ele in arr)
                    msg = msg + '\n'
                    #arduino.write(msg.encode('utf-8'))
                    print(arr)
                    
                elif (val/25 <= -1):
                    print("Go Diagonal Right")
                    arr = diagonal_R(0)
                    msg = ",".join(str(ele) for ele in arr)
                    msg = msg + '\n'
                    #arduino.write(msg.encode('utf-8'))
                    print(arr)
                else:
                    print("Go straight")
                    arr = straight(0)
                    msg = ",".join(str(ele) for ele in arr)
                    msg = msg + '\n'
                    #arduino.write(msg.encode('utf-8'))
                    print(arr)

            print("DelX:",delX)
            # Display class name and confidence
            # font = cv2.FONT_HERSHEY_COMPLEX
            
            ''' 
            cv2.putText(
                frame, str(diff),
                (int(center_x + 25), int(center_y)), font, 0.5, (0, 255, 0), 1
            )
            ''' 
            cv2.putText(
                frame, direction,
                (int(bb[0]), int(bb[1]) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1
            )
            '''
            if(direction == "L"):
                arduino.write(bytes('YES\n','utf-8'))
            elif(direction == "R"):
                arduino.write(bytes('YES\n','utf-8'))
            '''
    else:
        print("Go straight")
        arr = straight(0)
        msg = ",".join(str(ele) for ele in arr)
        msg = msg + '\n'
        #arduino.write(msg.encode('utf-8'))
        print(arr)
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    
    fps = int(fps)
    fps = str(fps)
    
    cv2.putText(
        frame, fps,
        (15, 60), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2
    )
    # Display the resulting frame
    cv2.imshow("ObjectDetection", frame)
    
    #cv2.imshow("ObjectDetection", frame)
    
    # Terminate run when "Q" pressed
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()

