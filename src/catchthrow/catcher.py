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
from ultralytics import YOLO

ball_position = "START"
endpt = None

def pickup_ball(ball_position, ball_velocity, right_gripper):

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


    
def catch(msg):

    # MOVE TO CATCH INIT POSITION
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

    
    start_position = msg.linear
    start_velocity = msg.angular
    catch_ball(start_position, start_velocity, right_gripper)



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
    rospy.init_node('catcher', anonymous=True)
  
    rospy.Subscriber("/cmd_vel", Twist,catch) # TODO: what are we subscribing to here?
  
    rospy.spin()
    print("Done.")


# def predict(chosen_model, img, classes=[], conf=0.5):
# if classes:
# results = chosen_model.predict(img, classes=classes, conf=conf)
# else:
# results = chosen_model.predict(img, conf=conf)
# return results
# def predict_and_detect(chosen_model, img, classes=[], conf=0.5, rectangle_thickness=2,
# text_thickness=1):
# results = predict(chosen_model, img, classes, conf=conf)
# for result in results:
# for box in result.boxes:
# cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
# (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0),
# rectangle_thickness)
# cv2.putText(img, f"{result.names[int(box.cls[0])]}",
# (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
# cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), text_thickness)
# return img, results
# # defining function for creating a writer (for mp4 videos)
# def create_video_writer(video_cap, output_filename):
# # grab the width, height, and fps of the frames in the video stream.
# frame_width = int(video_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# frame_height = int(video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# fps = int(video_cap.get(cv2.CAP_PROP_FPS))
# # initialize the FourCC and a video writer object
# fourcc = cv2.VideoWriter_fourcc(*’MP4V’)
# writer = cv2.VideoWriter(output_filename, fourcc, fps,
# (frame_width, frame_height))
# return writer
# model = YOLO("yolo11x.pt")
# output_filename = "output.mp4"
# video_path = r"balls.mp4"
# cap = cv2.VideoCapture(video_path)
# writer = create_video_writer(cap, output_filename)
# while True:
# success, img = cap.read()
# if not success:
# break
# result_img, _ = predict_and_detect(model, img, classes=[], conf=0.5)
# writer.write(result_img)
# cv2.imshow("Image", result_img)
# cv2.waitKey(1)
# writer.release()



if __name__ == '__main__':
    main()
