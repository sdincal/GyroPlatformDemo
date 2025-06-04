# gyro_controller.py
# A Webots Python Controller: oscillates the GYRO’s yaw around 180°
# with fixed roll = 5°, pitch = 3° (in radians).

from controller import Supervisor
import math

# Instantiate a Supervisor so we can set transforms directly
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Reference the GYRO_TRANSFORM node (DEF GYRO_TRANSFORM in Test1.wbt)
gyro_tf = robot.getFromDef("GYRO_TRANSFORM")
rot_field = gyro_tf.getField("rotation")  # we’ll update this field each timestep

# Fixed roll and pitch (in radians)
roll  = math.radians(5.0)
pitch = math.radians(3.0)
alpha = math.radians(180.0)  # static mounting offset

while robot.step(timestep) != -1:
    t = robot.getTime()
    # Let yaw oscillate ±10° around the 180° base
    yaw = alpha + math.radians(10.0) * math.sin(2.0 * t)

    # Build the 3×3 ZYX rotation matrix from (yaw, pitch, roll)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)

    # Row‐major 3×3 matrix
    rot_mat = [
        cos_y * cos_p,
        cos_y * sin_p * sin_r - sin_y * cos_r,
        cos_y * sin_p * cos_r + sin_y * sin_r,

        sin_y * cos_p,
        sin_y * sin_p * sin_r + cos_y * cos_r,
        sin_y * sin_p * cos_r - cos_y * sin_r,

        -sin_p,
        cos_p * sin_r,
        cos_p * cos_r
    ]

    # Convert the 3×3 matrix to axis‐angle form for Webots
    axis, angle = robot.getRotationFromMatrix(rot_mat)
    rot_field.setSFRotation([axis[0], axis[1], axis[2], angle])
