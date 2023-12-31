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


# def keyboard(self, key):
#     # global is_flying
#     print("key:", key)
#     fb_speed = 50
#     lf_speed = 50
#     ud_speed = 50
#     degree = 30
#     if key == ord('1'):
#         self.takeoff()
#         # is_flying = True
#     if key == ord('2'):
#         self.land()
#         # is_flying = False
#     if key == ord('3'):
#         self.send_rc_control(0, 0, 0, 0)
#         print("stop!!!!")
#     if key == ord('w'):
#         self.send_rc_control(0, fb_speed, 0, 0)
#         print("forward!!!!")
#     if key == ord('s'):
#         self.send_rc_control(0, (-1) * fb_speed, 0, 0)
#         print("backward!!!!")
#     if key == ord('a'):
#         self.send_rc_control((-1) * lf_speed, 0, 0, 0)
#         print("left!!!!")
#     if key == ord('d'):
#         self.send_rc_control(lf_speed, 0, 0, 0)
#         print("right!!!!")
#     if key == ord('z'):
#         self.send_rc_control(0, 0, ud_speed, 0)
#         print("down!!!!")
#     if key == ord('x'):
#         self.send_rc_control(0, 0, (-1) * ud_speed, 0)
#         print("up!!!!")
#     if key == ord('c'):
#         self.send_rc_control(0, 0, 0, degree)
#         print("rotate!!!!")
#     if key == ord('v'):
#         self.send_rc_control(0, 0, 0, (-1) * degree)
#         print("counter rotate!!!!")
#     if key == ord('5'):
#         height = self.get_height()
#         print(height)
#     if key == ord('6'):
#         battery = self.get_battery()
#         print(battery)


# Calib - --------------
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

f = cv2.FileStorage("calib_new.xml", cv2.FILE_STORAGE_READ)
intrinsic = f.getNode("intrinsic").mat()
distortion = f.getNode("distortion").mat()
f.release()

# Drone - --------------

drone = Tello()
drone.connect()
drone.streamon()

frame_read = drone.get_frame_read()

# PID - --------------

x_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
y_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
z_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.1)

x_pid.initialize()
y_pid.initialize()
z_pid.initialize()
yaw_pid.initialize()

while True:
    frame = frame_read.frame
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    cv2.imshow('drone', frame)
    key = cv2.waitKey(1)
    h, w = frame.shape[:2]
    markerCorners, markerids, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)


    if markerids is not None:
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, intrinsic, distortion)

        for i in range(rvec.shape[0]):
            id = markerids[i][0]
            print(id)
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
            # print(x_update, y_update, z_update, yaw_update)

            drone.send_rc_control(0, int(z_update // 2), int(y_update), int(yaw_update))

            frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerids)
            frame = cv2.drawFrameAxes(frame, intrinsic, distortion, rvec[i, :, :], tvec[i, :, :], 10)
            text = f'x = {tvec[0, 0, 0]}, y = {tvec[0, 0, 1]}, z = {tvec[0, 0, 2]}'
            cv2.putText(frame, text, (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('drone', frame)
        key = cv2.waitKey(1)
    # elif key != -1:
    #     keyboard(drone, key)
    #     time.sleep(0.3)
    else:
        drone.send_rc_control(0, 0, 0, 0)

    if key != -1:
        keyboard(drone, key)
