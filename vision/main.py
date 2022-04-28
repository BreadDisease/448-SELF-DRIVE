import cv2
import time
import serial
from robotvision import RobotVision
import matplotlib.pyplot as plt

frame_rate = 2
prev = time.time()
startTime = time.time()
total_time = 100000 # 5 minutes

robot = RobotVision(detection=False, debugMode=False)
ser = serial.Serial("/dev/ttyS0", 115200) # Open serial port to Arduino

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('level_1.mov')

try:
    while True:
        time_elapsed = time.time() - prev
        seconds = int(time.time() - startTime)
        res, image = cap.read()

        if time_elapsed > 1.0/frame_rate:
            prev = time.time()

            #frame = robot.resizeImage(image, 50)
            #cv2.imshow('original', image) # frame)

            isValid, output, img = robot.perceive(image,50)
            cv2.imshow('Frame1', img)

            msg = str(isValid) + "," + "{:.2f}".format(output) + "\n"
            #print(msg)
            ser.write(msg.encode())

        if seconds > total_time:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except:
    print("Input ends")

# After the loop release the cap object
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()


