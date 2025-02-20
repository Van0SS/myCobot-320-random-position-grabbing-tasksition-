#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Vision-based Follow, Pick & Place Demo
----------------------------------------
流程：
  1. go back to zero, open graper
  2. using camera to detect a blue object（only 2d coordination），calculate coordination based on robot（fixed z=165）。
     —— cancel all delay when following，to achieve realtime follow，send every five frames
  3. press enter，robot starts grabbing ，go up 50 units（z: 165→215）。
  4. move to pre-defined position，Open graper and place，Then go back to zero。
"""

import cv2
import numpy as np
import time
import json
from pymycobot import MyCobot320Socket


def getIpConfig():
    # Open and read the JSON file
    with open('env/ipconfig.json', 'r') as file:
        data = json.load(file)

    # read the ip and port info
    ip_address = data['ip']
    netport = data['port']

    return ip_address, netport


def convert_camera_to_robot(camera_coord, H):
    u, v = camera_coord
    point_h = np.array([u, v, 1.0])
    robot_h = H.dot(point_h)
    robot_h /= robot_h[2]
    return (robot_h[0], robot_h[1])


def follow_blue_object(mc, H, pick_z, pick_orientation, speed):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Unable to open camera")
        exit()

    print("\n--- Now following the blue object. Move the blue object within the camera view.")
    print("    Press Enter to confirm the position and pick the object.")

    pick_coords = None
    frame_count = 0  # 帧计数器

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Unable to read frame")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_coord = None

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2
                center_y = y + h // 2
                detected_coord = (center_x, center_y)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"({center_x}, {center_y})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if detected_coord is not None:
            robot_xy = convert_camera_to_robot(detected_coord, H)
            pick_coords = [robot_xy[0], robot_xy[1], pick_z] + pick_orientation

            if frame_count % 1 == 0:
                mc.send_coords(pick_coords, speed, 1)
        frame_count += 1

        cv2.imshow("Blue Object Following", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            pick_coords = None
            break
        elif key == 13:
            print("Position confirmed.")
            break

    cap.release()
    cv2.destroyAllWindows()
    return pick_coords


def main():
    # wifi

    ROBOT_IP, ROBOT_PORT = getIpConfig()
    mc = MyCobot320Socket(ROBOT_IP, ROBOT_PORT)
    time.sleep(1)
    mc.focus_all_servos()
    time.sleep(1)

    print("\n---> set_gripper_mode(0) => pass-through")
    ret_mode = mc.set_gripper_mode(0)
    print("     Return code:", ret_mode)
    time.sleep(1)

    home_angles = [0, 0, 0, 0, 0, 0]
    print("\n---> Move to home position:", home_angles)
    mc.send_angles(home_angles, 30)
    time.sleep(3)

    print("\n---> Open gripper")
    mc.set_gripper_state(0, 100)
    time.sleep(2)

    pick_z = 165
    pick_orientation = [-179.46, -6.69, 95.57]
    speed = 30

    H = np.array([
        [6.60782927e-04, 2.48469514e+00, -5.96091742e+02],
        [3.82506417e-01, 4.06164160e-01, -2.18163280e+02],
        [9.21284300e-05, -5.55189057e-03, 1.00000000e+00]
    ])

    pick_coords = follow_blue_object(mc, H, pick_z, pick_orientation, speed)
    if pick_coords is None:
        print("did not detect blue item, or not following. Break")
        return

    print("Final pick coordinates:", pick_coords)

    print("\n---> Close gripper to grasp object")
    mc.set_gripper_state(1, 100)
    time.sleep(2)

    pick_coords_ascend = [pick_coords[0], pick_coords[1], pick_coords[2] + 50] + pick_orientation
    print("\n---> Ascend after grasping (z + 50)")
    mc.send_coords(pick_coords_ascend, speed, 1)
    time.sleep(3)

    place_coords = [-329.1, 104.6, 179.1, -179.46, -6.69, 95.57]
    print("\n---> Move to place coordinates")
    mc.send_coords(place_coords, speed, 1)
    time.sleep(3)

    print("\n---> Open gripper to release object")
    mc.set_gripper_state(0, 100)
    time.sleep(2)

    print("\n---> Return to home position")
    mc.send_angles(home_angles, 30)
    time.sleep(3)

    print("\n---> Close gripper (final state)")
    mc.set_gripper_state(1, 100)
    time.sleep(2)

    print("\nPick & Place sequence completed.\n")


if __name__ == "__main__":
    main()
