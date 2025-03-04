from pymycobot import MyCobot320Socket
import time
import threading
import keyboard
import yaml

def get_ip_config():
    # read yaml file
    with open('env/configs.yaml', 'r') as file:
        data = yaml.safe_load(file)

    # load ip address and port
    ip_address = data['ip']
    netport = data['port']

    return ip_address, netport

ip_address, netport = get_ip_config()
mc = MyCobot320Socket(ip_address, netport)
# mc.send_angles([0, 0, 0, 0, 0, 0], 40)
# mc.release_all_servos()
mc.send_angles([0, 0, 0, 0, 0, 0], 40)
ret_mode = mc.set_gripper_mode(0)

time.sleep(1)
mc.set_gripper_state(0, 100) # get_gripper_value: 91
mc.set_gripper_state(254, 100)
gripper_open_close_threshold = mc.get_gripper_value() // 2

record_list = []
recording = True
mc.release_all_servos(1)
def _record():
    while recording:
        angles = mc.get_encoders()
        speeds = mc.get_servo_speeds()
        # # print(angles)
        # gripper = mc.get_gripper_value()
        # gripper = 1 if gripper <= gripper_open_close_threshold else 0

        if angles:
            record_list.append(angles + speeds )
            time.sleep(0.042)
print("start recording action. Press s to stop recording...")
record_t = threading.Thread(target=_record, daemon=True)
record_t.start()
keyboard.wait('s')
recording = False
record_t.join()
print("stop recording action.")
print(len(record_list), record_list)
mc.set_fresh_mode(0)
print("start replay action...")
for index, angles in enumerate(record_list):
    # print(index, angles[:-1])
    mc.set_encoders_drag(angles[:6],angles[6:])
    time.sleep(0.055)
    # mc.set_gripper_state(angles[-1],40)
    # time.sleep(0.1)
print("action sequence: ",record_list)