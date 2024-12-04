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

ball_position = None
def throw(side):
    right_gripper = robot_gripper.Gripper('right_gripper')
    limb = intera_interface.Limb(side)

    try:
        gripper = intera_interface.Gripper(side + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("The electric gripper is not detected on the robot.")
    else:
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

    #USE IK WITH THE BALL POSITION TO FIGURE OUT WHERE TO GO FROM HERE TODO



    ik_position_move(None)




    ### SET ROBOT TO INITIAL THROWING POSITION ###
    c = [float(i) for i in input("Give Thrower Initial Configuration (7 angles): ").split(" ")]
    joint5 = 0.0
    if c and len(c) == 7: 
        d = {}
        d['right_j0'] = c[0]
        d['right_j1'] = c[1]
        d['right_j2'] = c[2]
        d['right_j3'] = c[3]
        d['right_j4'] = c[4]
        d['right_j5'] = c[5]
        d['right_j6'] = c[6]
        r = rospy.Rate(10) # 10hz

        limb.set_joint_position_speed(0.3)
        curr = limb.joint_angles()

        while not np.allclose(list(curr.values()), list(d.values()), atol=0.05):
            limb.set_joint_positions(d)
            time.sleep(0.001)
            curr = limb.joint_angles()
            

    right_gripper.open()
    rospy.sleep(3.0)

    # Close the right gripper
    print('Closing...')
    right_gripper.close()
    rospy.sleep(1.0)

    ### THROW BALL ###
    input2 = [float(i) for i in input("Give Joint End Positions (3 Angles [5,3,1]): ").split(" ")]
    #Set Throwing Velocities
    if input2:
        d['right_j5'] = input2[0]
        d['right_j3'] = input2[1]
        d['right_j1'] = input2[2]
        
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

        right_gripper.open()



def detect_ball(side):

    sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", Image, image_callback)
    rospy.sleep(3.0)
    sub.unregister()
    return ball_position


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
    lower = np.array([0, 0, 200])  # Lower bound of color (adjust as needed)
    upper = np.array([180, 50, 255])  # Upper bound of color (adjust as needed)

    # Create a binary mask using the color range
    
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    num_balls = 0

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
        if 1 < radius < 100:
            # Draw the circle on the image
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            # cv2.circle(img, (int(x), int(y)), 2, (0, 0, 255), 3)

            # Log the position of the ball
            rospy.loginfo(f"Ball detected at ({x}, {y})")
            num_balls +=1

            global ball_position
            ball_position  = (x,y)

    # Display the image with detected balls
    cv2.imshow('Ball Detection', img)
    cv2.waitKey(1)
    
    print("Total number of balls detected: ", num_balls)

def ik_position_move(position):
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.5
        request.ik_request.pose_stamped.pose.position.y = 0.5
        request.ik_request.pose_stamped.pose.position.z = 0.0        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

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
    throw(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
