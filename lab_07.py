import cv2
import time

import cv2
import numpy as np
from djitellopy import Tello

from pyimagesearch.pid import PID


# from keyboard_djitellopy import keyboard


def clamp(x, max_speed_threshold=50):
    if x > max_speed_threshold:
        x = max_speed_threshold
    elif x < -max_speed_threshold:
        x = -max_speed_threshold

    return x


def keyboard(self, key):
    # global is_flying
    print("key:", key)
    fb_speed = 50
    lf_speed = 50
    ud_speed = 60
    degree = 30
    if key == ord('1'):
        self.takeoff()
        # is_flying = True
    if key == ord('2'):
        self.land()
        # is_flying = False
    if key == ord('3'):
        self.send_rc_control(0, 0, 0, 0)
        print("stop!!!!")
    if key == ord('w'):
        self.send_rc_control(0, fb_speed, 0, 0)
        print("forward!!!!")
    if key == ord('s'):
        self.send_rc_control(0, (-1) * fb_speed, 0, 0)
        print("backward!!!!")
    if key == ord('a'):
        self.send_rc_control((-1) * lf_speed, 0, 0, 0)
        print("left!!!!")
    if key == ord('d'):
        self.send_rc_control(lf_speed, 0, 0, 0)
        print("right!!!!")
    if key == ord('z'):
        self.send_rc_control(0, 0, ud_speed, 0)
        print("up!!!!")
    if key == ord('x'):
        self.send_rc_control(0, 0, (-1) * ud_speed, 0)
        print("down!!!!")
    if key == ord('c'):
        self.send_rc_control(0, 0, 0, degree)
        print("rotate!!!!")
    if key == ord('v'):
        self.send_rc_control(0, 0, 0, (-1) * degree)
        print("counter rotate!!!!")
    if key == ord('5'):
        height = self.get_height()
        print(height)
    if key == ord('6'):
        battery = self.get_battery()
        print(battery)


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
z_pid = PID(kP=0.35, kI=0.0001, kD=0.2)
yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.1)

x_pid.initialize()
y_pid.initialize()
z_pid.initialize()
yaw_pid.initialize()

# Calib - --------------
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)  ##
parameters = cv2.aruco.DetectorParameters()


stage = 0
frame = None

# while frame is not None:
#     frame = frame_read.frame
# time.sleep(2)

while True:
    # while frame is not None:
    frame = frame_read.frame
    # frame = frame_read.frame
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imshow("", frame)

    key = cv2.waitKey(1)
    h, w = frame.shape[:2]
    markerCorners, markerids, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    x_update = 0
    y_update = 0
    z_update = 0
    yaw_update = 0

    if markerids is not None:
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, intrinsic, distortion)
        for i in range(rvec.shape[0]):
            id = markerids[i][0]

            if stage == 1:
                if id == 1:
                    # print(tvec[i,0,:])
                    rotM = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvec[i], rotM, jacobian=0)
                    ypr = cv2.RQDecomp3x3(rotM)[0]
                    yaw_update = ypr[1]
                    x_update = tvec[i, 0, 0]
                    y_update = -(tvec[i, 0, 1]) + 20
                    z_update = z_err = tvec[i, 0, 2] - 80

                    x_update = clamp(x_pid.update(x_update, sleep=0))
                    # x_update = 0
                    y_update = clamp(y_pid.update(y_update, sleep=0))
                    z_update = clamp(z_pid.update(z_update, sleep=0))
                    yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))
                    if abs(z_update) <= 4 and abs(z_err) < 15:
                        drone.send_rc_control(0, 0, 45, 0)
                        time.sleep(1.8)
                        drone.send_rc_control(0, 35, 0, 0)
                        time.sleep(2)

                        # drone.send_rc_control(0, 0, -45, 0)
                        # time.sleep(3.5)

                        y_pid.initialize()
                        while abs(drone.get_distance_tof() - 105) > 15 or abs(y_update) > 10:
                            print(drone.get_distance_tof())
                            y_update = -(drone.get_distance_tof() - 105)
                            y_update = clamp(y_pid.update(y_update, sleep=0))
                            drone.send_rc_control(0, 0, int(y_update), 0)

                        x_pid.initialize()
                        y_pid.initialize()
                        z_pid.initialize()
                        yaw_pid.initialize()

                        drone.send_rc_control(0, 0, 0, 0)
                        stage = 2
                        break
                    break
            elif stage == 2:
                if id == 2:
                    rotM = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvec[i], rotM, jacobian=0)
                    ypr = cv2.RQDecomp3x3(rotM)[0]
                    yaw_update = ypr[1] * 1.2
                    x_update = tvec[i, 0, 0]
                    y_update = -(tvec[i, 0, 1]) + 10
                    z_update = z_err = tvec[i, 0, 2] - 60

                    x_update = clamp(x_pid.update(x_update, sleep=0))
                    y_update = clamp(y_pid.update(y_update, sleep=0))
                    z_update = clamp(z_pid.update(z_update, sleep=0))
                    yaw_update = 0

                    if abs(z_err) <= 10:
                        drone.send_rc_control(0, 0, -55, 0)
                        time.sleep(1.5)
                        drone.send_rc_control(0, 50, 0, 0)
                        time.sleep(3)
                        drone.send_rc_control(0, 0, 0, 0)

                        stage = 0
                        break
                    break
            elif stage == 0:
                if id == 0:
                    rotM = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvec[i], rotM, jacobian=0)
                    ypr = cv2.RQDecomp3x3(rotM)[0]
                    yaw_update = ypr[1] * 1.8
                    x_update = tvec[0, 0, 0]
                    y_update = -(tvec[0, 0, 1]) + 10
                    z_update = tvec[0, 0, 2] - 100
                    x_update = clamp(x_pid.update(x_update, sleep=0))
                    y_update = clamp(y_pid.update(y_update, sleep=0))
                    z_update = clamp(z_pid.update(z_update, sleep=0))
                    yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))
                    break
                elif id == 3:
                    stage = 3
            elif stage == 3:
                if id == 3:
                    rotM = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvec[i], rotM, jacobian=0)
                    ypr = cv2.RQDecomp3x3(rotM)[0]
                    yaw_update = ypr[1] * 1.2
                    x_update = tvec[0, 0, 0]
                    y_update = -(tvec[0, 0, 1]) + 10
                    z_update = z_err = tvec[0, 0, 2] - 60

                    x_update = clamp(x_pid.update(x_update, sleep=0))
                    y_update = clamp(y_pid.update(y_update, sleep=0))
                    z_update = clamp(z_pid.update(z_update, sleep=0))
                    yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))

                    if (abs(z_err)) <= 10:
                        drone.send_rc_control(0, 0, 0, 20)
                        time.sleep(2)
                        stage = 4
                        break
            elif stage == 4:
                if id == 4:
                    rotM = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvec[i], rotM, jacobian=0)
                    ypr = cv2.RQDecomp3x3(rotM)[0]
                    yaw_update = ypr[1] * 1.2
                    x_update = tvec[0, 0, 0] - 10
                    y_update = -(tvec[0, 0, 1] )+20
                    z_update = tvec[0, 0, 2] - 40
                    if (abs(z_update)) <= 10 and yaw_update <5:
                        drone.send_rc_control(-50, 0, 0, 0)
                        time.sleep(3)
                        break

            # elif id == 0:
            #     rotM = np.zeros(shape=(3, 3))
            #     cv2.Rodrigues(rvec[i], rotM, jacobian=0)
            #     ypr = cv2.RQDecomp3x3(rotM)[0]
            #     yaw_update = ypr[1] * 1.75
            #     x_update = tvec[i,0,0] - 10
            #     y_update = -(tvec[i,0,1] + 10) * 2
            #     z_update = tvec[i,0,2] - 85
            #     x_update = clamp(x_pid.update(x_update, sleep=0))
            #     y_update = clamp(y_pid.update(y_update, sleep=0))
            #     z_update = clamp(z_pid.update(z_update, sleep=0))
            #     yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))
            #     break
            # elif id == 5:
            #     rotM = np.zeros(shape=(3, 3))
            #     cv2.Rodrigues(rvec[i], rotM, jacobian=0)
            #     ypr = cv2.RQDecomp3x3(rotM)[0]
            #     yaw_update = ypr[1] * 1.5
            #     x_update = tvec[i,0,0] - 10
            #     y_update = -(tvec[i,0,1] - (-10))
            #     z_update = (tvec[i,0,2] - 50) * 0.75
            #     if z_update <= 27:
            #         hasRead5 = True
            #         drone.send_rc_control(-35, 0, 0, 0)
            #         time.sleep(0.5)
            #         drone.send_rc_control(0, 0, 0, 0)
            #     x_update = clamp(x_pid.update(x_update, sleep=0))
            #     y_update = clamp(y_pid.update(y_update, sleep=0))
            #     z_update = clamp(z_pid.update(z_update, sleep=0))
            #     yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))
            #     break
            # elif id == 3 and hasRead5:
            #     rotM = np.zeros(shape=(3, 3))
            #     cv2.Rodrigues(rvec[i], rotM, jacobian=0)
            #     ypr = cv2.RQDecomp3x3(rotM)[0]
            #     yaw_update = ypr[1] * 1.5
            #     x_update = tvec[i,0,0] + 10
            #     y_update = -(tvec[i,0,1] - (-20))
            #     z_update = (tvec[i,0,2] - 50) * 0.75
            #     if z_update <= 27:
            #         drone.send_rc_control(40, 0, 0, 0)
            #         time.sleep(1)
            #         drone.send_rc_control(0, 40, 0, -40)
            #         time.sleep(1.5)
            #         drone.send_rc_control(0, 0, 0, 0)

            #     break
            # elif id == 4:
            #     rotM = np.zeros(shape=(3, 3))
            #     cv2.Rodrigues(rvec[i], rotM, jacobian=0)
            #     ypr = cv2.RQDecomp3x3(rotM)[0]
            #     yaw_update = ypr[1] * 1.2
            #     x_update = tvec[i,0,0] + 8
            #     y_update = -(tvec[i,0,1]-10)
            #     z_update = (tvec[i,0,2] - 75) * 0.5
            #     if abs(z_update) <= 10 and abs(x_update) <= 20:
            #         drone.send_rc_control(0, 0, 0, 0)
            #         drone.land()
            #         drone.send_rc_control(0, 0, 0, 0)
            #     break

        # x_update = clamp(x_pid.update(x_update, sleep=0))
        # y_update = clamp(y_pid.update(y_update, sleep=0))
        # z_update = clamp(z_pid.update(z_update, sleep=0))
        # yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))
        # print(y_update)
        drone.send_rc_control(int(x_update), int(z_update), int(y_update), int(yaw_update))

        frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerids)
        # frame = cv2.aruco.drawAxis(frame, intrinsic, distortion, rvec[i,:,:], tvec[i,:,:], 10)
        frame = cv2.drawFrameAxes(frame, intrinsic, distortion, rvec[i, :, :], tvec[i, :, :], 10)
        cv2.putText(frame,
                    "x = " + str(round(tvec[i, 0, 0], 2)) + ", y = " + str(round(tvec[i, 0, 1], 2)) + ", z = " + str(
                        round(tvec[i, 0, 2], 2)), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("", frame)
        key = cv2.waitKey(33)
    elif drone.is_flying:
        drone.send_rc_control(0, 0, 0, 0)
    if key != -1:
        keyboard(drone, key)
        time.sleep(0.1)

# frame_read = drone.get_frame_read()

# # PID - --------------


# while True:
#     frame = frame_read.frame
#     frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

#     cv2.imshow('drone', frame)
#     key = cv2.waitKey(1)
#     h, w = frame.shape[:2]
#     markerCorners, markerids, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
#     if markerids is not None:
#         rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, intrinsic, distortion)

#         for i in range(rvec.shape[0]):
#             id = markerids[i][0]
#             if id != 0:
#                 continue
#             rotM = np.zeros(shape=(3, 3))
#             cv2.Rodrigues(rvec[i], rotM, jacobian=0)
#             ypr = cv2.RQDecomp3x3(rotM)[0]

#             yaw_update = ypr[1] * 1.2
#             x_update = tvec[0, 0, 0] - 10
#             y_update = -(tvec[0, 0, 1] - (-20))
#             z_update = tvec[0, 0, 2] - 150
#             x_update = clamp(x_pid.update(x_update, sleep=0))
#             y_update = clamp(y_pid.update(y_update, sleep=0))
#             z_update = clamp(z_pid.update(z_update, sleep=0))
#             yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))
#             print(x_update, y_update, z_update, yaw_update)

#             drone.send_rc_control(0, int(z_update // 2), int(y_update), int(yaw_update))

#             frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerids)
#             frame = cv2.drawFrameAxes(frame, intrinsic, distortion, rvec[i, :, :], tvec[i, :, :], 10)
#             text = f'x = {tvec[0, 0, 0]}, y = {tvec[0, 0, 1]}, z = {tvec[0, 0, 2]}'
#             cv2.putText(frame, text, (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
#         cv2.imshow('drone', frame)
#         key = cv2.waitKey(1)
#     elif key != -1:
#         keyboard(drone, key)
#         time.sleep(0.3)
#     else:
#         drone.send_rc_control(0, 0, 0, 0)


# -------------------------------------------------------
# lab06
# # Calib - --------------
# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
# parameters = cv2.aruco.DetectorParameters()

# f = cv2.FileStorage("calib_new.xml", cv2.FILE_STORAGE_READ)
# intrinsic = f.getNode("intrinsic").mat()
# distortion = f.getNode("distortion").mat()
# f.release()

# # Drone - --------------

# drone = Tello()
# drone.connect()
# drone.streamon()

# frame_read = drone.get_frame_read()

# # PID - --------------

# x_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
# y_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
# z_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
# yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.1)

# x_pid.initialize()
# y_pid.initialize()
# z_pid.initialize()
# yaw_pid.initialize()

# while True:
#     frame = frame_read.frame
#     frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

#     cv2.imshow('drone', frame)
#     key = cv2.waitKey(1)
#     h, w = frame.shape[:2]
#     markerCorners, markerids, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
#     if markerids is not None:
#         rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, intrinsic, distortion)

#         for i in range(rvec.shape[0]):
#             id = markerids[i][0]
#             if id != 0:
#                 continue
#             rotM = np.zeros(shape=(3, 3))
#             cv2.Rodrigues(rvec[i], rotM, jacobian=0)
#             ypr = cv2.RQDecomp3x3(rotM)[0]
#             yaw_update = ypr[1] * 1.2
#             x_update = tvec[0, 0, 0] - 10
#             y_update = -(tvec[0, 0, 1] - (-20))
#             z_update = tvec[0, 0, 2] - 150
#             x_update = clamp(x_pid.update(x_update, sleep=0))
#             y_update = clamp(y_pid.update(y_update, sleep=0))
#             z_update = clamp(z_pid.update(z_update, sleep=0))
#             yaw_update = clamp(yaw_pid.update(yaw_update, sleep=0))
#             print(x_update, y_update, z_update, yaw_update)

#             drone.send_rc_control(0, int(z_update // 2), int(y_update), int(yaw_update))

#             frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerids)
#             frame = cv2.drawFrameAxes(frame, intrinsic, distortion, rvec[i, :, :], tvec[i, :, :], 10)
#             text = f'x = {tvec[0, 0, 0]}, y = {tvec[0, 0, 1]}, z = {tvec[0, 0, 2]}'
#             cv2.putText(frame, text, (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
#         cv2.imshow('drone', frame)
#         key = cv2.waitKey(1)
#     elif key != -1:
#         keyboard(drone, key)
#         time.sleep(0.3)
#     else:
#         drone.send_rc_control(0, 0, 0, 0)
