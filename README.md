# Robot-Trick-or-Treat
Robot Trick or Treat
Hardware List
NVIDIA Jetson Nano Developer Kit 5V2A USB

Trossenrobotics WidowX 250 S 12V/5A DC

Intel RealSense Depth Camera D415

Rechargeable Battery 12V/5A 5V/2A (Amazon.com)

Any Keyboard (Bluetooth or USB) and Monitor (with HDMI or DP port) 
Software List
Ubuntu 20.04, https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image

ROS2 Galactic, https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html#raspberry-pi-4b-arm64-architecture

$ sudo apt install curl

$ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/rpi4/xsarm_rpi4_install.sh' > xsarm_rpi4_install.sh

$ chmod +x xsarm_rpi4_install.sh

$ ./xsarm_rpi4_install.sh -d galactic

librealsense SDK, https://jetsonhacks.com/2019/12/22/install-realsense-camera-in-5-minutes-jetson-nano/

$ git clone https://github.com/JetsonHacksNano/installLibrealsense

$ cd installLibrealsense

$ ./buildLibrealsense.sh

# if with error of no pyrealsesnse2 module, try uninstall and reinstall pyrealsesnse2

$ pip3 uninstall pyrealsense2

$ pip3 install pyrealsense2

Lunch Robot Arm
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=mobile_wx250s

If use the simulator 

ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=mobile_wx250s use_sim:=true

In another terminal 

python3 <the following code>.py

Code for Trick or Treat
#!/usr/bin/env python3

import sys



from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import numpy as np

import time  # Make sure to import the time module

import random  # Ensure the random module is imported

import cv2

import pyrealsense2 as rs



def fun(bot): #Cobra model

    bot.arm.set_single_joint_position(joint_name='wrist_angle', position=+np.pi / 3)

    bot.arm.set_single_joint_position(joint_name='shoulder', position=-1 * np.pi / 10.0)

    #bot.arm.set_ee_pose_components(x=0.1, z=0.6, y=0)

    #bot.arm.set_single_joint_position(joint_name='waist', position=np.pi / 2.0)

    #bot.arm.set_ee_pose_components(x=0.1, z=0.5, y=0)

    #bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi / 2.0)

    bot.arm.set_ee_pose_components(x=0.1, z=0.6, y=0)

    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi / 2.0)

    bot.arm.set_ee_pose_components(x=0.1, z=0.5, y=0)

    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi / 2.0)



def getCandy(bot):

    bot.arm.set_ee_pose_components(x=0.2, z=0.4)

    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi / 2.0)

    bot.arm.set_single_joint_position(joint_name='elbow', position=1 * np.pi / 8.0)

    bot.arm.set_single_joint_position(joint_name='wrist_angle', position=+np.pi / 4)

    bot.arm.set_single_joint_position(joint_name='shoulder', position=1 * np.pi / 15.0)  #

    bot.gripper.release()

    bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=np.pi / 2)

    bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=-1 * np.pi / 2)

    bot.arm.set_single_joint_position(joint_name='elbow', position=1 * np.pi / 7.0)

    bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=0 * np.pi / 2)

    bot.gripper.grasp()



def putCandy(bot,min_dis):

    bot.arm.set_single_joint_position(joint_name='wrist_angle', position=+np.pi / 3)

    bot.arm.set_single_joint_position(joint_name='shoulder', position=-1 * np.pi / 10.0)

    bot.arm.set_ee_pose_components(x=0.3, z=0.4)

    while determinDistance() > min_dis:

        print('waiting')

    #time.sleep(2)  # Wait for 2 seconds

    bot.gripper.release()



def determinDistance():

    frameset = pipe.wait_for_frames()

    depth_frame = frameset.get_depth_frame()

    depth = np.asanyarray(depth_frame.get_data())

    height, width = depth.shape[:2]

    expected = 300

    aspect = width / height

    crop_start = round(expected * (aspect - 1) / 2)

    xmin = 0.25

    xmax = .75

    ymin = 0.00 # upper

    ymax = .55  

    className = 'center'

    scale = height / expected

    xmin_depth = int((xmin * expected + crop_start) * scale)

    ymin_depth = int((ymin * expected) * scale)

    xmax_depth = int((xmax * expected + crop_start) * scale)

    ymax_depth = int((ymax * expected) * scale)

    xmin_depth, ymin_depth, xmax_depth, ymax_depth

    # Crop depth data:

    depth = depth[xmin_depth:xmax_depth, ymin_depth:ymax_depth].astype(float)

    # Get data scale from the device and convert to meters

    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    depth = depth * depth_scale

    dist, _, _, _ = cv2.mean(depth)

    print("Detected a {0} {1:.3} meters away.".format(className, dist))

    return dist



def main():

    bot = InterbotixManipulatorXS(

        robot_model='mobile_wx250s',

        group_name='arm',

        gripper_name='gripper'

    )



    if (bot.arm.group_info.num_joints < 5):

        bot.core.get_logger().fatal('This demo requires the robot to have at least 5 joints!')

        bot.shutdown()

        sys.exit()

    bot.arm.set_ee_pose_components(x=0.2, z=0.4)

    return bot

    #bot.shutdown()



if __name__ == '__main__':

    bot=main()



    # Setup:

    pipe = rs.pipeline()

    cfg = rs.config()

    # cfg.enable_device_from_file("../object_detection.bag")

    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipe.start(cfg)



    for i in range(100):

        getCandy(bot)

        rand_num = random.randint(1, 10)

        if rand_num % 2 == 0: # enter Cobra model

            fun(bot)

        putCandy(bot,0.1)

    bot.arm.go_to_sleep_pose()

    bot.shutdown()

    pipe.stop()

