#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Vision-based Follow, Pick & Place Demo with Multi-threading and Jog-like Increment

流程：
  1. 机器人回零位，并将抓夹设置为透传模式后打开抓夹。
  2. 启动一个线程持续读取摄像头，检测蓝色物体，转换为机器人坐标（固定 z=165），
     并更新全局目标坐标（不直接发送运动指令）。
  3. 主线程每隔 10 毫秒检查最新目标坐标，如果与上次发送的坐标差异超过一定阈值，则计算
     小步增量（受限于最大步长），调用 send_coords 发送新的绝对坐标，实现“Jog 点动”跟随。
  4. 当用户按下回车（或 q）后，结束跟随，使用最后目标作为最终抓取点，执行抓取放置流程：
     闭合抓夹、升高、移动到放置点、打开抓夹放置、返回零位、最终闭合抓夹。
"""

import cv2
import numpy as np
import threading
import time
from pymycobot import MyCobot320Socket

# 全局变量（线程间共享）
global_target = None       # 机器人坐标系下的 (x, y) 目标位置（浮点数）
confirm_follow = False     # 跟随结束标志
target_lock = threading.Lock()

def convert_camera_to_robot(camera_coord, H):
    """
    将相机坐标系下的二维点转换为机器人坐标系下的二维点。
    """
    u, v = camera_coord
    point_h = np.array([u, v, 1.0])
    robot_h = H.dot(point_h)
    robot_h /= robot_h[2]
    return (robot_h[0], robot_h[1])

def image_processing_thread(H):
    """
    图像处理线程：持续读取摄像头，检测蓝色物体，并更新全局目标坐标。
    当用户按下回车或 'q' 键时结束跟随。
    """
    global global_target, confirm_follow
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Unable to open camera")
        return

    while not confirm_follow:
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
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0,0,255), -1)
                cv2.putText(frame, f"({center_x}, {center_y})", (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        # 如果检测到目标，转换为机器人坐标并更新全局变量
        if detected_coord is not None:
            robot_xy = convert_camera_to_robot(detected_coord, H)
            with target_lock:
                global_target = robot_xy

        cv2.imshow("Blue Object Following", frame)
        cv2.imshow("Mask", mask)
        key = cv2.waitKey(1) & 0xFF
        if key == 13 or key == ord('q'):
            confirm_follow = True

    cap.release()
    cv2.destroyAllWindows()

def main():
    global global_target, confirm_follow

    # 机器人连接参数
    ROBOT_IP = "192.168.43.94"
    ROBOT_PORT = 9000
    mc = MyCobot320Socket(ROBOT_IP, ROBOT_PORT)


    time.sleep(1)
    mc.focus_all_servos()
    time.sleep(1)
    mc.set_fresh_mode(1)
    time.sleep(1)

    # 在使用抓夹之前，设置为透传模式
    print("\n---> set_gripper_mode(0) => pass-through")
    ret_mode = mc.set_gripper_mode(0)
    print("Return code:", ret_mode)
    time.sleep(1)

    # 回零位
    home_angles = [0, 0, 0, 0, 0, 0]
    print("\n---> Move to home position:", home_angles)
    mc.send_angles(home_angles, 30)
    time.sleep(3)

    # 打开抓夹
    print("\n---> Open gripper")
    mc.set_gripper_state(0, 100)
    time.sleep(2)

    # 预设参数
    pick_z = 165
    pick_orientation = [-179.46, -6.69, 95.57]  # 固定姿态
    speed = 80

    # 标定得到的转换矩阵 H（请根据实际标定数据进行替换）
    H = np.array([
        [6.60782927e-04,  2.48469514e+00, -5.96091742e+02],
        [3.82506417e-01,  4.06164160e-01, -2.18163280e+02],
        [9.21284300e-05, -5.55189057e-03,  1.00000000e+00]
    ])

    # 启动图像处理线程
    img_thread = threading.Thread(target=image_processing_thread, args=(H,))
    img_thread.start()

    # 主线程：利用 Jog 点动方式进行小步跟随
    last_sent = None   # 记录上一次发送的 (x, y)
    threshold = 1    # mm：坐标变化阈值
    max_step = 100     # mm：每次最大增量

    while not confirm_follow:
        with target_lock:
            current_target = global_target
        if current_target is not None:
            if last_sent is None:
                # 初始时直接设置 last_sent 为当前目标，并发送初始坐标
                last_sent = current_target
                coords = [last_sent[0], last_sent[1], pick_z] + pick_orientation
                mc.send_coords(coords, speed, 1)
            else:
                dx = current_target[0] - last_sent[0]
                dy = current_target[1] - last_sent[1]
                if abs(dx) >= threshold or abs(dy) >= threshold:
                    # 限制每次最大增量
                    inc_x = max(-max_step, min(max_step, dx))
                    inc_y = max(-max_step, min(max_step, dy))
                    new_x = last_sent[0] + inc_x
                    new_y = last_sent[1] + inc_y
                    new_coords = [new_x, new_y, pick_z] + pick_orientation
                    mc.send_coords(new_coords, speed, 1)
                    last_sent = (new_x, new_y)
        time.sleep(0.01)  # 10 毫秒循环一次

    img_thread.join()

    if last_sent is None:
        print("No valid target detected. Exiting.")
        return

    final_target = last_sent
    print("Final target (robot xy):", final_target)
    time.sleep(1)

    # 构造最终抓取点
    pick_coords = [final_target[0], final_target[1], pick_z] + pick_orientation
    print("Pick coordinates:", pick_coords)
    time.sleep(1)

    # 执行抓取流程
    print("\n---> Close gripper to grasp object")
    mc.set_gripper_state(1, 100)
    time.sleep(2)

    # 升高50单位（z: 165 -> 215）
    pick_coords_ascend = [pick_coords[0], pick_coords[1], pick_coords[2] + 50] + pick_orientation
    print("\n---> Ascend after grasping (z + 50)")
    mc.send_coords(pick_coords_ascend, speed, 1)
    time.sleep(3)

    # 预设放置点（固定坐标）
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
