# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

TIME_LIMIT = 60 # In seconds
FRAME_PER_SECOND = 60

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = FRAME_PER_SECOND
rawCapture = PiRGBArray(camera, size=(640, 480))

#Store frames of live video
img_array = []

# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
prev = 0

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	# show the frame
	cv2.imshow("Frame", image)
	img_array.append(image)
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
	time_elapsed = time.time() - prev
	if time_elapsed >= TIME_LIMIT:
		break


height, width, layers = img_array[0].shape
vidsize = (width,height)

out = cv2.VideoWriter('Pi_Video.mov',cv2.VideoWriter_fourcc(*'DIVX'), 15, vidsize)
 
for i in range(len(img_array)):
    out.write(img_array[i])   
out.release()