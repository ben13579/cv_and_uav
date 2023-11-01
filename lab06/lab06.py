from dis import dis
from turtle import up
from zlib import Z_PARTIAL_FLUSH
import cv2
from cv2 import drawChessboardCorners
import numpy as np
import time
from djitellopy import Tello
from pyimagesearch.pid import PID
from keyboard_djitellopy import keyboard
import math


def clamp(x, max_speed_threshold=50):
    if x > max_speed_threshold:
        x = max_speed_threshold
    elif x < -max_speed_threshold:
        x = -max_speed_threshold

    return x




drone = Tello()
drone.connect()
drone.streamon()
frame_read = drone.get_frame_read()
f = cv2.FileStorage("calib_new.xml.xml", cv2.FILE_STORAGE_READ)
intrinsic = f.getNode("intrinsic").mat()
distortion = f.getNode("distortion").mat()
f.release()

x_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
y_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
z_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.1)

x_pid.initialize()
y_pid.initialize()
z_pid.initialize()
yaw_pid.initialize()

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
while True:
    frame = frame_read.frame
    frame = frame.frame
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imshow("", frame)
    key = cv2.waitKey(1)
    h, w = frame.shape[:2]
    markerCorners, markerids, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if markerids is not None:
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, intrinsic, distortion)
        # if a == 0:
        #     print(tvec)
        #     a+=1
        for i in range(rvec.shape[0]):
            id = markerids[i][0]
            if id != 0:
                continue
            rotM = np.zeros(shape=(3, 3))
            cv2.Rodrigues(rvec[i], rotM, jacobian=0)
            ypr = cv2.RQDecomp3x3(rotM)[0]
            yaw_update = ypr[1] * 1.2
            x_update = tvec[0, 0, 0] - 10
            y_update = -(tvec[0, 0, 1] - (-20))
            z_update = tvec[0, 0, 2] - 150
            x_update = clamp(x_pid.update(x_update, sleep=0))
            y_update = clamp(y_pid.update(y_update, sleep=0))
            z_update = clamp(z_pid.update(z_update, sleep=0))
            yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))
            print(x_update, y_update, z_update, yaw_update)

            drone.send_rc_control(0, int(z_update // 2), int(y_update), int(yaw_update))

            frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerids)
            frame = cv2.drawFrameAxes(frame, intrinsic, distortion, rvec[i, :, :], tvec[i, :, :], 10)
            cv2.putText(frame, "x = " + str(round(tvec[0, 0, 0], 2)) + ", y = " + str(
                round(tvec[0, 0, 1], 2)) + ", z = " + str(round(tvec[0, 0, 2], 2)), (0, 64), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("", frame)
        key = cv2.waitKey(1)
    else:
        drone.send_rc_control(0, 0, 0, 0)
    if key != -1:
        keyboard(drone, key)