from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import sys
import numpy as np
from genPos import gameScheme

camera = PiCamera()
camera.resolution = (1308, 1000)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(1308, 1008))
 
time.sleep(0.1)
 
def captureFrame():
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        picture = camera.capture("img/game1.png")
        time.sleep(1)
        game = gameScheme()
        return game.getState()

