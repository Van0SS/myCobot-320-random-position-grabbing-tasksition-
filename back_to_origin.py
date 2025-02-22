import time
import json
import yaml
from pymycobot import MyCobot320Socket


def get_ip_config():
    # 读取 YAML 文件
    with open('env/configs.yaml', 'r') as file:
        data = yaml.safe_load(file)

    # 读取 IP 和端口信息
    ip_address = data['ip']
    netport = data['port']

    return ip_address, netport


def main():
    # config wifi setting
    ROBOT_IP, ROBOT_PORT = get_ip_config()
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
    mc.send_angles(home_angles, 10)
    time.sleep(3)

    print("\n---> Close gripper")
    mc.set_gripper_state(1, 100)
    time.sleep(2)

    print("\n---> Open gripper")
    mc.set_gripper_state(0, 100)
    time.sleep(2)

if __name__ == "__main__":
    main()