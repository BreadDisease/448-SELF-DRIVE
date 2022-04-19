import cv2
import time
import serial
from robotvision import RobotVision
import matplotlib.pyplot as plt

frame_rate = 60
prev = time.time()
startTime = time.time()
total_time = 300 # 5 minutes

robot = RobotVision(detection=False, debugMode=False)
ser = serial.Serial("/dev/ttyS0", 115200) # Open serial port to Arduino

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('roadtest1.mp4')

meanOut = 0
counter = 0
while True:
    time_elapsed = time.time() - prev
    seconds = int(time.time() - startTime)
    res, image = cap.read()

    if time_elapsed > 1.0/frame_rate:
        prev = time.time()

        #frame = robot.resizeImage(image, 20)
        #cv2.imshow('original', frame)

        isValid, output, _ = robot.perceive(image)
        #cv2.imshow('Frame1', img)

        # if isValid == 1:
        #     # Expected output
        #     #meanOut = (meanOut * counter + output) / (counter + 1)
        #     meanOut = meanOut + (output - meanOut) / (counter + 1)
        #     counter = counter + 1

        msg = str(isValid) + "," + "{:.2f}".format(output) + "\n"
        ser.write(msg.encode())

    if seconds > total_time:
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()



