#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse

import rospy


from std_msgs.msg import Header
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import intera_interface
import intera_external_devices
import numpy as np
from intera_interface import CHECK_VERSION
from intera_interface import gripper as robot_gripper
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import tf2_ros
import tf
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose

ball_position = "START"
box_position = None
endpt = None


def pickup_ball(ball_position, right_gripper):

    pixel_world_scalar = 1900

    final_cam_pos_x, final_cam_pos_y = ball_position[0] - 293.61285400390625 , ball_position[1] - 246.72952270507812
    final_world_x_offset, final_world_y_offset = final_cam_pos_x/pixel_world_scalar, final_cam_pos_y/pixel_world_scalar

    end_position = [endpt['position'][0] - final_world_y_offset,
                    endpt['position'][1] - final_world_x_offset,
                    endpt['position'][2] - 0.05,
                    endpt['orientation'][0],
                    endpt['orientation'][1],
                    endpt['orientation'][2],
                    endpt['orientation'][3]]

    ik_service_client(end_position)

    right_gripper.open()
    rospy.sleep(3.0)

    pickup_position = [endpt['position'][0] - final_world_y_offset,
                    endpt['position'][1] - final_world_x_offset,
                    endpt['position'][2] - 0.20,
                    endpt['orientation'][0],
                    endpt['orientation'][1],
                    endpt['orientation'][2],
                    endpt['orientation'][3]]

    ik_service_client(pickup_position)

    print('Closing...')
    right_gripper.close()
    rospy.sleep(1.0)

    ik_service_client(end_position)

    # Close the right gripper

def move_to_position(x, y, z):
   

    arm_group = MoveGroupCommander("right_arm")
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    # Set orientation if needed
    # pose_target.orientation.w = 1.0

    arm_group.set_pose_target(pose_target)
    plan = arm_group.go(wait=True)
    arm_group.execute(plan,wait=True)

    
def throw_to_box(box_position, right_gripper):

    pixel_world_scalar = 1900

    final_cam_pos_x, final_cam_pos_y = ball_position[0] - 293.61285400390625 , ball_position[1] - 246.72952270507812
    final_world_x_offset, final_world_y_offset = final_cam_pos_x/pixel_world_scalar, final_cam_pos_y/pixel_world_scalar

    end_position = [box_position['position'][0] - final_world_y_offset,
                    endpt['position'][1] - final_world_x_offset,
                    endpt['position'][2] - 0.05,
                    endpt['orientation'][0],
                    endpt['orientation'][1],
                    endpt['orientation'][2],
                    endpt['orientation'][3]]

    ik_service_client(end_position)

    # right_gripper.open()
    # rospy.sleep(3.0)

    # pickup_position = [endpt['position'][0] - final_world_y_offset,
    #                 endpt['position'][1] - final_world_x_offset,
    #                 endpt['position'][2] - 0.19,
    #                 endpt['orientation'][0],
    #                 endpt['orientation'][1],
    #                 endpt['orientation'][2],
    #                 endpt['orientation'][3]]

    # ik_service_client(pickup_position)

    # print('Closing...')
    # right_gripper.close()
    # rospy.sleep(1.0)

    # ik_service_client(end_position)

    # Close the right gripper


def ik_service_client(end_position):

    rospy.wait_for_service('compute_ik')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    

    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
        
    # Set the desired orientation for the end effector HERE
    print("Curr Position: ", endpt["position"])
    print("Attempting to move to: ", end_position[:3])

    request.ik_request.pose_stamped.pose.position.x = end_position[0]
    request.ik_request.pose_stamped.pose.position.y = end_position[1]
    request.ik_request.pose_stamped.pose.position.z = end_position[2]
    request.ik_request.pose_stamped.pose.orientation.x = end_position[3]
    request.ik_request.pose_stamped.pose.orientation.y = end_position[4]
    request.ik_request.pose_stamped.pose.orientation.z = end_position[5]
    request.ik_request.pose_stamped.pose.orientation.w = end_position[6]
    
    try:
        # Send the request to the service
        response = compute_ik(request)
        
        # Print the response HERE
        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Plan IK
        plan = group.plan()
        group.execute(plan[1])
        # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
        
        # # Execute IK if safe
        # if user_input == 'y':
        #     group.execute(plan[1])
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def throw(side):
    limb = intera_interface.Limb(side)
    
    right_gripper = robot_gripper.Gripper('right_gripper')
    
    right_gripper.open()


    gripper = intera_interface.Gripper(side + '_gripper')
    has_gripper = True

    joints = limb.joint_names()

    def set_j(limb, joint_name, delta, speed):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        print("Executing" + str(joint_command))
        # default 0.3
        limb.set_joint_position_speed(speed)
        # default 0.1
        limb.set_joint_positions(joint_command)

    #Custom Tuck Position
    TUCK_POSITION = {}
    TUCK_POSITION['right_j0'] = 0
    TUCK_POSITION['right_j1'] = -0.5
    TUCK_POSITION['right_j2'] = 0
    TUCK_POSITION['right_j3'] = 1.5
    TUCK_POSITION['right_j4'] = 0
    TUCK_POSITION['right_j5'] = -1
    TUCK_POSITION['right_j6'] = 1.7


    limb.set_joint_position_speed(0.3)
    curr = limb.joint_angles()

    while not np.allclose(list(curr.values()), list(TUCK_POSITION.values()), atol=0.05):
        limb.set_joint_positions(TUCK_POSITION)
        time.sleep(0.001)
        curr = limb.joint_angles()

    print("DETECT BALL RESULT" , detect_ball("right"))
    global ball_position
    if ball_position == None:
        return

    #Set to Position to Pick up ball:
    #0 -0.5 0 1.5 0 -1 1.7
    PICKUP_POSITION = {}
    PICKUP_POSITION['right_j0'] = 0
    PICKUP_POSITION['right_j1'] = -0.5
    PICKUP_POSITION['right_j2'] = 0
    PICKUP_POSITION['right_j3'] = 1.5
    PICKUP_POSITION['right_j4'] = 0
    PICKUP_POSITION['right_j5'] = 0.5
    PICKUP_POSITION['right_j6'] = 1.7


    limb.set_joint_position_speed(0.3)
    curr = limb.joint_angles()

    while not np.allclose(list(curr.values()), list(PICKUP_POSITION.values()), atol=0.05):
        limb.set_joint_positions(PICKUP_POSITION)
        time.sleep(0.001)
        curr = limb.joint_angles()


    global endpt
    endpt = limb.endpoint_pose()

    #TODO PICKUP BALL

    pickup_ball(ball_position, right_gripper)




    ### SET ROBOT TO INITIAL THROWING POSITION ###
    c = input("Give Thrower Initial Configuration (7 angles) or press enter to use default values: ")
    
    if c: 
        c = [float(i) for i in c.split(" ")]
        if len(c)==7:
            print("inside")
            
            d = {}
            d['right_j0'] = c[0]
            d['right_j1'] = c[1]
            d['right_j2'] = c[2]
            d['right_j3'] = c[3]
            d['right_j4'] = c[4]
            d['right_j5'] = c[5]
            d['right_j6'] = c[6]
        else:
            d = {}
            d['right_j0'] = 0.5 
            d['right_j1'] = -0.4 
            d['right_j2'] = 0 
            d['right_j3'] = 1.8 
            d['right_j4'] = 0 
            d['right_j5'] = 2.4
            d['right_j6'] = 1.7
    
    else:
        d = {}
        d['right_j0'] = 0.5 
        d['right_j1'] = -0.4 
        d['right_j2'] = 0 
        d['right_j3'] = 1.8 
        d['right_j4'] = 0 
        d['right_j5'] = 2.4
        d['right_j6'] = 1.7


    print(d)
    r = rospy.Rate(10) # 10hz

    limb.set_joint_position_speed(0.3)
    curr = limb.joint_angles()

    while not np.allclose(list(curr.values()), list(d.values()), atol=0.05):
        limb.set_joint_positions(d)
        time.sleep(0.001)
        curr = limb.joint_angles()
    detect_ball("right")
    box_position = ball_position

    ### THROW BALL ###
    input2 = input("Give Joint End Positions (3 Angles [5,3,1]) or press enter to use default: ")
    #Set Throwing Velocities
    if input2:
        input2 = [float(i) for i in input2.split(" ")]
        d['right_j5'] = input2[0]
        d['right_j3'] = input2[1]
        d['right_j1'] = input2[2]
    else:
        d['right_j5'] = 0
        d['right_j3'] = 1
        d['right_j1'] = -1
    r = rospy.Rate(10) # 10hz
    master_vel = -50
    velocity = {}
    for i in d:
        if i=='right_j5':
            velocity[i] = master_vel * 3
        elif i == 'right_j3':
            velocity[i] = master_vel * 0.7
        elif i=='right_j1':
            velocity[i] = master_vel * 0.025
        else:
            velocity[i]=0

    #Perform Throw
    count = 0
    limb.set_joint_position_speed(1.0)
    while count<330:
        limb.set_joint_velocities(velocity)
        time.sleep(0.001)
        count+=1
    toss_position = limb.endpoint_pose()['position']
    toss_velocity  =limb.endpoint_pose()['orientation']
    right_gripper.open()
    rospy.sleep(3.0)
    ball_position = "FINISHED"
    detect_box("right")
    # print(box_position) #730, 446
    
    # send_toss_data(toss_position, toss_velocity)
    # except:
    #     has_gripper = False
    #     rospy.loginfo("The electric gripper is not detected on the robot.")
    #     print("hi")
        
        
        # # detect_box("right")
        # # global box_position
        # # if box_position == None:
        # #     return

        # d = {}
        # d['right_j0'] = 0.5 
        # d['right_j1'] = -0.4 
        # d['right_j2'] = 0 
        # d['right_j3'] = 1.8 
        # d['right_j4'] = 0 
        # d['right_j5'] = 2.4
        # d['right_j6'] = 1.7



        # r = rospy.Rate(10) # 10hz

        # limb.set_joint_position_speed(0.3)
        # curr = limb.joint_angles()

        # while not np.allclose(list(curr.values()), list(d.values()), atol=0.05):
        #     limb.set_joint_positions(d)
        #     time.sleep(0.001)
        #     curr = limb.joint_angles()

    # detect_box("right")
    # print(box_position) #730, 446
    #     # move_to_position(ball_position[0],ball_position[1],0.1)

        

def detect_box(msg):
    sub = rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, image_callback_box)
    rospy.sleep(3.0)
    sub.unregister()
    return box_position


def detect_ball(side):

    sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", Image, image_callback)
    rospy.sleep(3.0)
    sub.unregister()
    return ball_position

def image_callback_box(msg):
    try:
        # Convert the ROS image to OpenCV format
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
   
    # Convert the image to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the HSV color range for the ball (adjust these values depending on the ball's color)
    lower = np.array([90,150,50])  # Lower bound of color (adjust as needed)
    upper = np.array([130, 255, 255])  # Upper bound of color (adjust as needed)
    lower = np.array([35,50,50])  # Lower bound of color (adjust as needed)
    upper = np.array([85, 255, 255])  # Upper bound of color (adjust as needed)
    
   

    # Create a binary mask using the color range
    
    mask = cv2.inRange(hsv, lower, upper)
    red_regions = cv2.bitwise_and(img, img, mask=mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("Red Regions", red_regions)
    global box_position 
    box_position = None
    # Loop through each contour and find the bounding rectangle and center
    for contour in contours:
        # Skip small contours (you can adjust the area threshold)
        if cv2.contourArea(contour) < 1000:  # Adjust the area threshold as needed
            continue

        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)

        # Calculate the center of the rectangle
        center_x = x + w // 2
        center_y = y + h // 2

        # Draw the bounding rectangle and the center point
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)  # Red center

        # Optionally, print the center position
        print(f"Center of bounding box: ({center_x}, {center_y})")
        box_position = (center_x, center_y)
        

    # Display the result
    cv2.imshow("Original Image with Bounding Boxes and Centers", img)
    cv2.waitKey(0)

    
    

    # Wait for the user to press a key to close the image windows
    cv2.waitKey(1)

def image_callback(msg):
    try:
        # Convert the ROS image to OpenCV format
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
   
    # Convert the image to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the HSV color range for the ball (adjust these values depending on the ball's color)
    lower = np.array([0, 0, 0])  # Lower bound of color (adjust as needed)
    upper = np.array([180, 255, 50])  # Upper bound of color (adjust as needed)

    # Create a binary mask using the color range
    
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    num_balls = 0
    global ball_position
    ball_position = None

    for contour in contours:
        # Approximate the contour with a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # If the contour area is too small, skip it
        if cv2.contourArea(contour) < 100:
            continue
        
        
        # Get the enclosing circle of the contour
        (x, y), radius = cv2.minEnclosingCircle(contour)

        # If the radius is in a reasonable range (you can adjust these thresholds)
        if 30 < radius < 100:
            # Draw the circle on the image
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            # cv2.circle(img, (int(x), int(y)), 2, (0, 0, 255), 3)

            # Log the position of the ball
            rospy.loginfo(f"Ball detected at ({x}, {y})")
            num_balls +=1

            
            ball_position  = (x,y)

    # Display the image with detected balls
    cv2.imshow('Ball Detection', img)
    cv2.waitKey(1)
    
    print("Total number of balls detected: ", num_balls)

def send_toss_data(position,velocity):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    msg = Twist()
    msg.linear = position
    msg.angular = velocity
    pub.publish(msg)




def main():
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    while ball_position == "START" or ball_position == "FINISHED":
        throw(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
