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

def region_of_interest_right(rg_image):
    h = rg_image.shape[0]
    poly = np.array([ [(450,220),(450,380),(950,380),(950,220)]])
    #poly = np.array([ [(450,200),(450,500),(950,500),(950,200)]])
    #according to the carmera data, these number still need to change
    mask = np.zeros_like(rg_image)
    cv2.fillPoly(mask, poly, 255)
    masked_image = cv2.bitwise_and(rg_image, mask)
    return masked_image

def region_of_interest_left(rg_image):
    h = rg_image.shape[0]
    poly = np.array([ [(0,220),(0,380),(450,380),(450,220)]])
    #poly = np.array([ [(0,200),(0,500),(450,500),(450,200)]])
    #according to thecarmera data, these number still need to change
    mask = np.zeros_like(rg_image)
    cv2.fillPoly(mask, poly, 255)
    masked_image = cv2.bitwise_and(rg_image, mask)
    return masked_image    

frame_rate = 60
prev = time.time()
startTime = time.time()
total_time = 300

robot = RobotVision(detection=False)

#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('roadtest1.mp4')

# while True:
#     time_elapsed = time.time() - prev
#     seconds = int(time.time() - startTime)
#     res, image = cap.read()

#     if time_elapsed > 1.0/frame_rate:
#         prev = time.time()

        
#         frame = robot.resizeImage(image)
#         cv2.imshow('original', frame)

#         img = robot.perceive(image)
#         cv2.imshow('Frame1', img)  

#     if seconds > 60:
#         break
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

try:
    while True:
        time_elapsed = time.time() - prev
        seconds = int(time.time() - startTime)
        res, image = cap.read()

        if time_elapsed > 1.0/frame_rate:
            prev = time.time()
                                    
            # Do something with your image here.
            #process_image()
            # cv2.imshow('original', image)
            #img_array.append(image)

            frame = robot.resizeImage(image)
            cv2.imshow('original', frame)

            canny_pic = canny(frame)
            cv2.imshow('canny', canny_pic)

            # region_right = region_of_interest_right(canny_pic)
            # cv2.imshow('Right', region_right)

            # region_left = region_of_interest_left(canny_pic)
            # cv2.imshow('Left', region_left)

            tmpt, img = robot.perceive(image)
            cv2.imshow('Frame1', img)
            cv2.imshow('Temp', tmpt)

            # try:
            #     frame = robot.resizeImage(image)
            #     cv2.imshow('original', frame)
            #     print("Aload")
            #     img = robot.perceive(image)
            #     cv2.imshow('Frame1', img)
            # except:
            #     print("An exception occurred")
            #     break     

        if seconds > total_time:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except:
    print("Streaming closed")

# After the loop release the cap object
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()

# height, width, layers = img_array[0].shape
# vidsize = (width,height)

# out = cv2.VideoWriter('output_vid.mov',cv2.VideoWriter_fourcc(*'DIVX'), 15, vidsize)
 
# for i in range(len(img_array)):
#     out.write(img_array[i])   
# out.release()


