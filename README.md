# myCobot-320-random-position-grabbing-task-YOLOv5s

Demo video: https://www.youtube.com/watch?v=ElcOFTFr0Zg&ab_channel=dalong

Using myCobot-320 to grab an object in the area of camera detection

jog_following is an updated version of following.py, using dual thread, avoided camera lag and slow tracking.

following.py is for lettting the end factor to follow the blue item in the camera.

get_pose.py is for getting the current pose info of the robot, you have to manully move the robot to the Calibration Points.

matrix.py is for calculating the Homography Transform

robot control.ipynb is for letting the robot grab the object with random position

robot_control_with_Qwen.ipynb is for letting the robot grab objects and move to a random position by your instruction.

jog_follow_new.py is upadated using fresh mode, which will make the process more efficient.

Voice.py is voice based control

test.py is for quickly test your prompt

## Before the Start:
### Set up the Wi-Fi Hotspot
Create a Wi-Fi hotspot with the following SSID and password. Make sure the Wi-Fi band is **2.4G**
* SSID: **MyCobotWiFi2.4G**
* password: **mycobot123**

### Set up the ```configs.json``` file
1. Create a folder named as ```env``` under the ```root```.
3. In the ```configs.json``` file, type the following information:
```
ip: <IP address shown on the MyCobot>
port: 9000

qwen_apikey: <Your Qwen model API_KEY>
```

### Install related python libraries
```
pip install pymycobot
```
There are other required libraries in the ```requirements.txt``` file.
