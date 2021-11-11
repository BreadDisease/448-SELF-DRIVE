import cv2
import time
from robotvision import RobotVision

frame_rate = 10
prev = 0

robot = RobotVision()

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('level_1.mov')

while True:

    time_elapsed = time.time() - prev
    res, image = cap.read()

    if time_elapsed > 1.0/frame_rate:
        prev = time.time()

        # Do something with your image here.
        #process_image()
        # cv2.imshow(image)
        frame = robot.resizeImage(image)
        cv2.imshow('original', frame)

        img = robot.perceive(image)

        cv2.imshow('Frame1', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()

