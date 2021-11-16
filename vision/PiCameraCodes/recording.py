import time
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 60