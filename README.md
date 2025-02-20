# myCobot-320-random-position-grabbing-task-YOLOv5s
Using myCobot-320 to grab an object in the area of camera detection

following.py is for lettting the end factor to follow the blue item in the camera.

get_pose.py is for getting the current pose info of the robot, you have to manully move the robot to the Calibration Points.

matrix.py is for calculating the Homography Transform

robot control.ipynb is for letting the robot grab the object with random position

## Before the Start:
### Set up the Wi-Fi Hotspot
Create a Wi-Fi hotspot with the following SSID and password. Make sure the Wi-Fi band is **2.4G**
* SSID: **MyCobotWiFi2.4G**
* password: **mycobot123**

### Set up the ```ipconfig.json``` file
1. Create a folder named as ```env``` under the ```root```.
2. In the ```env``` folder, create a new JSON file named as ```ipconfig.json```.
3. In the ```ipconfig.json``` file, type the following information:
```
{
    "ip": "<IP address shown on the MyCobot>",
    "port": 9000
}
```

### Install related python libraries
```
pip install pymycobot
```
There are other required libraries in the ```requirements.txt``` file.