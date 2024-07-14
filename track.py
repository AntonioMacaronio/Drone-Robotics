#!/usr/bin/python
import json
import math
from threading import Thread
from djitellopy import Tello
import cv2, math, time
import os
import numpy as np

MAP_SIZE_COEFF = 5.14
CENTER_X = 128 # screen is 256x256
CENTER_Y = 128

def move_towards_center(bbox_center_x, bbox_center_y):
    # Calculate offsets
    offset_x = bbox_center_x - CENTER_X
    offset_y = bbox_center_y - CENTER_Y

    # Define thresholds for movement (to avoid small jittery movements)
    threshold_x = 20
    threshold_y = 20

    # Move drone based on offsets
    if abs(offset_x) > threshold_x:
        if offset_x > 0:
            tello.move_right(20)
        else:
            tello.move_left(20)

    if abs(offset_y) > threshold_y:
        if offset_y > 0:
            tello.move_down(20)
        else:
            tello.move_up(20)

if __name__ == '__main__':
    tello = Tello()
    tello.connect()
    tello.streamon()

    tello.takeoff()
    tello.rotate_clockwise(90)
    try:
        while True:
            img = tello.get_frame_read().frame # np array

            # Generate bounding boxes from YOLO
            bounding_boxes = [
                (50, 50, 200, 150),
                (300, 200, 100, 100)
            ]

            bbox_center_x = []
            bbox_center_y = []

            for (x, y, w, h) in bounding_boxes:
                bbox_center_x.append(x + w // 2)
                bbox_center_y.append(y + h // 2)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            cv2.imshow('frame', img)            
            for x, y in zip(bbox_center_x, bbox_center_y):
                cv2.circle(img, (x,y), 5, (0, 0, 255), -1)
                move_towards_center(x, y)
                cv2.imshow('frame', img)            

            # TODO: generate path waypoints to add inputs for tello
                # integrate model + get annotations for faces
                # figure out where to move from faces + relative location
                # add those inputs

            

    except KeyboardInterrupt:
        exit(1)
    finally:
        print("fin")
