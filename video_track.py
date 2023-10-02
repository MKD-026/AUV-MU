import cv2
import numpy as np
from ultralytics import YOLO

def delta_xy(imgcenter_x, imgcenter_y, center_x, center_y):
    delta_x = center_x - imgcenter_x
    delta_y = center_y - imgcenter_y
    rounded_delta_x = round(delta_x, 3)
    rounded_delta_y = round(delta_y, 3)
    return rounded_delta_x, rounded_delta_y


# load a pretrained YOLOv8n model
model = YOLO("yolov8n.pt", "v8")

# Vals to resize video frames | small frame optimise the run
frame_wid = 640
frame_hyt = 480

#for using webcam
# cap = cv2.VideoCapture(0)

#for video input
cap = cv2.VideoCapture("walk.mp4")

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    image_height, image_width = frame.shape[0], frame.shape[1]
    imgcenter_x = image_width / 2
    imgcenter_y = image_height / 2
    # if frame is read correctly ret is True

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Predict on image
    detect_params = model.predict(source=[frame], conf=0.45, save=False)

    # Convert tensor array to numpy
    DP = detect_params[0].numpy()
    print(DP)

    if len(DP) != 0:
        for i in range(len(detect_params[0])):
            print(i)

            boxes = detect_params[0].boxes
            box = boxes[i]  # returns one box
            clsID = box.cls.numpy()[0]
            conf = box.conf.numpy()[0]
            bb = box.xyxy.numpy()[0]

            if clsID == 0.0: #2.0: #check coco list for indexes
                cv2.rectangle(frame, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), (0, 255, 0), 3)
                center_x = (bb[0] + bb[2]) / 2
                center_y = (bb[1] + bb[3]) / 2

                cv2.line(frame, (int(center_x), int(center_y)), (int(imgcenter_x), int(imgcenter_y)), (255, 255, 255), 5)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                cv2.circle(frame, (int(imgcenter_x), int(imgcenter_y)), 5, (0, 0, 255), -1)

                diff = delta_xy(imgcenter_x, imgcenter_y, center_x, center_y)

                # Display class name and confidence
                font = cv2.FONT_HERSHEY_COMPLEX
                cv2.putText(
                    frame, class_list[int(clsID)] + " " + str(round(conf, 3)) + "%",
                    (int(bb[0]), int(bb[1]) - 10), font, 1, (255, 255, 255), 2
                )

                cv2.putText(
                    frame, str(diff),
                    (int(center_x + 25), int(center_y)), font, 0.5, (0, 255, 0), 1
                )

    # Display the resulting frame
    cv2.imshow("ObjectDetection", frame)

    # Terminate run when "Q" pressed
    if cv2.waitKey(1) == ord("q"):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
