#!/usr/bin/env python3

import cv2
import sys
import math
import time
import numpy as np
import traceback
# import jetson.inference
# import jetson.utils

from datetime import datetime
# from utils.mtcnn import TrtMtcnn
from threading import Thread
from djitellopy import Tello
import argparse

# Video recoding frame rate (Max 15 on Jetson Nano)
video_fps = 15

# Land if battery is below percentage
low_bat = 15

# Enable video recoder
enableVideoRecoder = True

# Max velocity settings (Keep in mind the delay!)
max_yaw_velocity = 50
max_up_down_velocity = 40
max_forward_backward_velocity = 40


parser = argparse.ArgumentParser()
parser.add_argument(
    '--input', help='Path to image or video. Skip to capture frames from camera')
parser.add_argument('--thr', default=0.2, type=float,
                    help='Threshold value for pose parts heat map')
parser.add_argument('--width', default=368, type=int,
                    help='Resize input to specific width.')
parser.add_argument('--height', default=368, type=int,
                    help='Resize input to specific height.')

args = parser.parse_args()


BODY_PARTS = {"Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
              "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
              "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
              "LEye": 15, "REar": 16, "LEar": 17, "Background": 18}

POSE_PAIRS = [["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
              ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
              ["Neck", "RHip"], ["RHip", "RKnee"], [
    "RKnee", "RAnkle"], ["Neck", "LHip"],
    ["LHip", "LKnee"], ["LKnee", "LAnkle"], [
    "Neck", "Nose"], ["Nose", "REye"],
    ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"]]

inWidth = args.width
inHeight = args.height


def limitVelocity(velocity, max_velocity):
    return min(max_velocity, max(-max_velocity, velocity))


def videoRecorder():
    global frame_read
    now = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video/video-' + now + '.avi',
                            cv2.VideoWriter_fourcc(*'XVID'), video_fps, (width, height))

    while keepRecording:
        start = time.time()
        currentFrame = frame_read.frame
        video.write(currentFrame)
        end = time.time()
        elapsed = end - start
        time.sleep(max(0, 1 / video_fps - elapsed))
        end = time.time()
    video.release()


def main():
    global net, detection, frame_read, keepRecording, low_bat, max_yaw_velocity, max_up_down_velocity, max_forward_backward_velocity

    currentFrame = False
    keepRecording = True

    # Init model
    net = cv2.dnn.readNetFromTensorflow("graph_opt.pb")

    tello = Tello()
    tello.connect()

    battery = tello.get_battery()
    print("Battery: ", tello.get_battery())

    if battery < low_bat:
        print("Battery low")
        exit()

    tello.streamon()
    frame_read = tello.get_frame_read()
    currentFrame = frame_read.frame

    if enableVideoRecoder:
        recorder = Thread(target=videoRecorder)
        recorder.start()

    tello.send_rc_control(0, 0, 0, 0)
    tello.takeoff()
    tello.move_up(50)

    winname = "Tello"
    running = True
    detection = False
    while running:

        battery = tello.get_battery()
        if battery < low_bat:
            print("Battery low")
            running = False

        currentFrame = frame_read.frame
        frameWidth = currentFrame.shape[1]
        frameHeight = currentFrame.shape[0]

        net.setInput(cv2.dnn.blobFromImage(currentFrame, 1.0, (368, 368),
                                           (127.5, 127.5, 127.5), swapRB=True, crop=False))

        out = net.forward()
        # MobileNet output [1, 57, -1, -1], we only need the first 19 elements
        out = out[:, :19, :, :]

        assert (len(BODY_PARTS) == out.shape[1])

        points = []
        for i in range(len(BODY_PARTS)):
            # Slice heatmap of corresponging body's part.
            heatMap = out[0, i, :, :]
            # print("Heatmap: ", heatMap)

            # Originally, we try to find all the local maximums. To simplify a sample
            # we just find a global one. However only a single pose at the same time
            # could be detected this way.
            _, conf, _, point = cv2.minMaxLoc(heatMap)
            x = (frameWidth * point[0]) / out.shape[3]
            y = (frameHeight * point[1]) / out.shape[2]
            # Add a point if it's confidence is higher than threshold.
            points.append((int(x), int(y)) if conf > args.thr else None)

        print("points: ", points)
        for pair in POSE_PAIRS:
            partFrom = pair[0]
            # print("Partfrom:", partFrom)
            partTo = pair[1]
            assert (partFrom in BODY_PARTS)
            assert (partTo in BODY_PARTS)

            idFrom = BODY_PARTS[partFrom]
            idTo = BODY_PARTS[partTo]

            if points[idFrom] and points[idTo]:
                cv2.line(currentFrame, points[idFrom],
                         points[idTo], (0, 255, 0), 3)
                cv2.ellipse(currentFrame, points[idFrom], (3, 3),
                            0, 0, 360, (0, 0, 255), cv2.FILLED)
                cv2.ellipse(currentFrame, points[idTo], (3, 3), 0,
                            0, 360, (0, 0, 255), cv2.FILLED)

        h, w, c = currentFrame.shape
        yaw_velocity = 0
        left_right_velocity = 0
        up_down_velocity = 0
        forward_backward_velocity = 0
        if len(points) > 0:
            # detection = getDetectedPerson(detections, detection)
            if points[0] != None:
                noseX, noseY = points[0][0], points[0][1]
                color = (0, 255, 0)
                # np_img = cv2.rectangle(np_img, (int(detection.Left), int(
                #     detection.Top)), (int(detection.Right), int(detection.Bottom)), color, 2)
                # np_img = cv2.circle(np_img, (int(detection.Center[0]), int(
                #     detection.Center[1])), 5, color, 5, cv2.LINE_AA)
                detectX = int(noseX)
                centerX = w / 2
                yaw_velocity = int((detectX - centerX) /
                                   7)  # proportional only
                up_down_velocity = int((30 - noseY) / 2)
                forward_backward_velocity = int(
                    (h - noseY - 30) / 2)
                yaw_velocity = limitVelocity(yaw_velocity, max_yaw_velocity)
                up_down_velocity = limitVelocity(
                    up_down_velocity, max_up_down_velocity)
                forward_backward_velocity = limitVelocity(
                    forward_backward_velocity, max_forward_backward_velocity)
                left_right_velocity = 0
                # print(w, h, detection.Top, detection.Bottom,
                #       forward_backward_velocity, up_down_velocity)

            else:
                print("No Person detected")
        else:
            print("No detection")

        # Tello remote control by object detection
        tello.send_rc_control(
            left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)

        # scale image for small screen
        scale_percent = 50  # percent of original size
        width = int(currentFrame.shape[1] * scale_percent / 100)
        height = int(currentFrame.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(currentFrame, dim, interpolation=cv2.INTER_AREA)

        # Show image on screen (Needs running X-Server!)
        cv2.namedWindow(winname)
        cv2.moveWindow(winname, 0, 0)
        cv2.imshow(winname, resized)

        keyCode = cv2.waitKey(1) & 0xFF
        if keyCode == 27:
            break

    # Goodby
    cv2.destroyAllWindows()
    tello.send_rc_control(0, 0, 0, 0)
    battery = tello.get_battery()
    tello.land()
    if enableVideoRecoder:
        recorder.join()
        keepRecording = False


if __name__ == "__main__":
    # Execute if run as a script
    main()
