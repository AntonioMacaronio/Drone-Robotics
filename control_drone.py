from threading import Thread
from djitellopy import Tello
import cv2, math, time
import os


tello = Tello()
tello.connect()#
tello.streamon()
#print(tello.get_udp_video_address())
#print(type(tello.get_udp_video_address()))
#cap = cv2.VideoCapture(tello.get_udp_video_address())
tello.takeoff()
tello.move_up(80)
try:
    while True:
        img = tello.get_frame_read().frame#
        cv2.imshow('frame', img)
        cv2.waitKey(1)#
except KeyboardInterrupt:
    exit(1)
finally:
    print("fin")