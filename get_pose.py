#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Demo of 'release -> manual drag -> focus -> read pose' for MyCobot 320 via Socket/Wi-Fi.

1) Connect to the robot over Wi-Fi.
2) Check connectivity by attempting a basic get_angles().
3) Release all servos so user can manually position the robot.
4) Re-focus (power on) all servos.
5) Read & print the new angles/coords.
"""

import time
from pymycobot import MyCobot320Socket

def main():
    #---------------------------------------------------
    # 1) Build WiFi Connection
    #---------------------------------------------------
    mc = MyCobot320Socket("192.168.43.94", 9000)

    #---------------------------------------------------
    # 2) Test
    #---------------------------------------------------
    print(">> Test communication: if wrong: -1")
    test_angles = mc.get_angles()
    print("   current angle:", test_angles)

    if test_angles == -1:
        print("\n[Warning] If return -1, may fail\n")
    else:
        print("   Communication correct ...")

    time.sleep(1)

    #---------------------------------------------------
    # 3) Release control
    #    Different from power_off() ，release_all_servos() stay connected
    #---------------------------------------------------
    print("\n>>  (release_all_servos) ...")
    mc.release_all_servos()
    time.sleep(0.5)

    input("Move robot manually，then press enter to continue...")

    #---------------------------------------------------
    # 4) focus server again
    #---------------------------------------------------
    print(">>  (focus_all_servos) ...")
    mc.focus_all_servos()
    time.sleep(1)

    #---------------------------------------------------
    # 5) Read and prin current coordination
    #---------------------------------------------------
    angles = mc.get_angles()
    coords = mc.get_coords()

    print("\n===== Current Pose Information =====")
    print("Joint Angles (j1 ~ j6):", angles)
    print("Coordinates [x, y, z, rx, ry, rz]:", coords)
    print("====================================\n")

    if angles == -1 or coords == -1:
        print("[note] if remains -1，")
        print("       could be other problems")
    else:
        print("[success] 。")

if __name__ == "__main__":
    main()
