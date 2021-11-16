import cv2
import time
from robotvision import RobotVision

frame_rate = 10
prev = 0

robot = RobotVision()

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('level_1.mov')
img_array = []
while True:

    time_elapsed = time.time() - prev
    res, image = cap.read()

    if time_elapsed > 1.0/frame_rate:
        prev = time.time()

        # Do something with your image here.
        #process_image()
        # cv2.imshow(image)
        try:
            frame = robot.resizeImage(image)
            cv2.imshow('original', frame)
            img = robot.perceive(image, detection=False)
            cv2.imshow('Frame1', img)
        except:
            print("An exception occurred")
            break     

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()

# height, width, layers = img_array[0].shape
# vidsize = (width,height)

# out = cv2.VideoWriter('output_level1.mov',cv2.VideoWriter_fourcc(*'DIVX'), 15, vidsize)
 
# for i in range(len(img_array)):
#     out.write(img_array[i])   
# out.release()

