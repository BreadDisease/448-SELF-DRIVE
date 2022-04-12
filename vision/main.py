import cv2
import time
from robotvision import RobotVision
import matplotlib.pyplot as plt

def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # change the pic to the gray one and give name as linepic for using later to draw line.

    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # make the pic to be smooth and east to draw
    canny_img = cv2.Canny(blur, 50, 150)
    return canny_img

frame_rate = 60
prev = time.time()
startTime = time.time()
total_time = 300

robot = RobotVision(detection=False)

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('roadtest1.mp4')

while True:
    time_elapsed = time.time() - prev
    seconds = int(time.time() - startTime)
    res, image = cap.read()

    if time_elapsed > 1.0/frame_rate:
        prev = time.time()

        frame = robot.resizeImage(image, 50)
        cv2.imshow('original', frame)

        # canny_pic = canny(frame)
        # cv2.imshow('canny', canny_pic)

        tmpt, img = robot.perceive(image, 50)
        cv2.imshow('Frame1', img)
        #cv2.imshow('Temp', tmpt) 

    if seconds > total_time:
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()



