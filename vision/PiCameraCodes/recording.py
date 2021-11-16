import time
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 60


camera.start_recording("testVid.mov")

time.sleep(10)

camera.stop_recording()