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


def map_keyboard(side):
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
    print(joints)

    def set_j(limb, joint_name, delta, speed):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        print("Executing" + str(joint_command))
        limb.set_joint_position_speed(speed)
        # default 0.1
        limb.set_joint_positions(joint_command)

        

    def set_g(action):
        if has_gripper:
            if action == "close":
                gripper.close()
            elif action == "open":
                gripper.open()
            elif action == "calibrate":
                gripper.calibrate()

    bindings = {
        '1': (set_j, [limb, joints[0], 0.1], joints[0]+" increase"),
        'q': (set_j, [limb, joints[0], -0.1], joints[0]+" decrease"),
        '2': (set_j, [limb, joints[1], 0.1], joints[1]+" increase"),
        'w': (set_j, [limb, joints[1], -0.1], joints[1]+" decrease"),
        '3': (set_j, [limb, joints[2], 0.1], joints[2]+" increase"),
        'e': (set_j, [limb, joints[2], -0.1], joints[2]+" decrease"),
        '4': (set_j, [limb, joints[3], 0.1], joints[3]+" increase"),
        'r': (set_j, [limb, joints[3], -0.1], joints[3]+" decrease"),
        '5': (set_j, [limb, joints[4], 0.1], joints[4]+" increase"),
        't': (set_j, [limb, joints[4], -0.1], joints[4]+" decrease"),
        '6': (set_j, [limb, joints[5], 0.1], joints[5]+" increase"),
        'y': (set_j, [limb, joints[5], -0.1], joints[5]+" decrease"),
        '7': (set_j, [limb, joints[6], 0.1], joints[6]+" increase"),
        'u': (set_j, [limb, joints[6], -0.1], joints[6]+" decrease")
     }
    if has_gripper:
        bindings.update({
        '8': (set_g, "close", side+" gripper close"),
        'i': (set_g, "open", side+" gripper open"),
        '9': (set_g, "calibrate", side+" gripper calibrate")
        })
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    c = [float(i) for i in input("Give: ").split(" ")]
    joint5 = 0.0
    print(c)
    if c and c in ['\x1b', '\x03']:
        done = True
        rospy.signal_shutdown("Example finished.")
    if c and len(c) == 7:
        
        d = {}
        d['right_j0'] = c[0]
        d['right_j1'] = c[1]
        d['right_j2'] = c[2]
        d['right_j3'] = c[3]
        d['right_j4'] = c[4]
        d['right_j5'] = c[5]
        d['right_j6'] = c[6]
        joint5 = float(c[5])
        r = rospy.Rate(10) # 10hz

        limb.set_joint_position_speed(0.3)
        curr = limb.joint_angles()
        # for i in range(100):
        #     limb.set_joint_positions(d)
        #     time.sleep(0.01)
        
        while not np.allclose(list(curr.values()), list(d.values()), atol=0.05):
            limb.set_joint_positions(d)
            time.sleep(0.01)
            curr = limb.joint_angles()
            

    right_gripper.open()
    rospy.sleep(2.0)
    print('Done!')

    # Close the right gripper
    print('Closing...')
    right_gripper.close()
    rospy.sleep(1.0)

    input2 = [float(i) for i in input("Give 5 and 3: ").split(" ")]
    if input2 and input2 in ['\x1b', '\x03']:
        done = True
        rospy.signal_shutdown("Example finished.")
    if input2:
        d['right_j5'] = input2[0]
        d['right_j3'] = input2[1]
        
        r = rospy.Rate(10) # 10hz
        print("input2: ", input2)
        print("c5: ", joint5)
        print("float: ", joint5)
        
        #angle_diff = abs(joint5 - float(input2))
        #d['right_j5'] = float(input2)
        gripper_opened = False

        limb.set_joint_position_speed(80)
        cvals = list(limb.joint_angles().values())
        dvals = list(d.values())

        while not np.allclose(cvals, dvals, atol=0.05):
            limb.set_joint_positions(d)
            time.sleep(0.001)
            cvals = list(limb.joint_angles().values())
            max_diff = max([abs(cvals[j]-dvals[j]) for j in range(len(cvals))])
            if max_diff < 0.5:
                right_gripper.open()

        

def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
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
    map_keyboard(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
