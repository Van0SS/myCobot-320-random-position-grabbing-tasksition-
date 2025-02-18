import numpy as np
import cv2


# camera_points：
camera_points = np.array([
    [440, 378],
    [423, 323],
    [299, 324],
    [280, 375],
    [551, 412],
    [68, 425],
    [366, 347]
], dtype=np.float32)

# robot_points：
robot_points = np.array([
    [-333.6, -105.8],
    [-273.3, -97.4],
    [-270.7, -40.7],
    [-320.0, -35.6],
    [-340.9, -127.9],
    [-338.4, 14.1],
    [-295.2, -64.2]
], dtype=np.float32)

# could be more than 4, but least 4
H, status = cv2.findHomography(camera_points, robot_points)

print("pairs needed：", len(camera_points))
print("matrix H:")
print(H)
