#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading

class CameraDisplay:
    def __init__(self):
        rospy.init_node('camera_display', anonymous=True)

        self.bridge = CvBridge()
        self.current_topic = "/io/internal_camera/head_camera/image_raw"
        self.subscriber = None
        self.lock = threading.Lock()

        self.running = True

    def switch_camera(self, new_topic):
        with self.lock:
            if self.subscriber:
                self.subscriber.unregister()  # Unsubscribe from the current topic

            self.current_topic = new_topic
            self.subscriber = rospy.Subscriber(self.current_topic, Image, self.image_callback)
            rospy.loginfo(f"Switched to camera topic: {self.current_topic}")

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        # Display the image
        cv2.imshow("Camera Feed", cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            rospy.signal_shutdown("User requested shutdown.")

    def run(self):
        # Start with the default topic
        self.switch_camera(self.current_topic)

        while self.running:
            # Check for user input to switch cameras
            key = cv2.waitKey(1) & 0xFF
            if key == ord('h'):  # Switch to head camera
                self.switch_camera("/io/internal_camera/head_camera/image_raw")
            elif key == ord('r'):  # Switch to right-hand camera
                self.switch_camera("/io/internal_camera/right_hand_camera/image_raw")
            elif key == ord('q'):  # Quit
                self.running = False
                rospy.signal_shutdown("User requested shutdown.")

        # Cleanup
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera_display = CameraDisplay()
    try:
        camera_display.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera display node terminated.")
    finally:
        cv2.destroyAllWindows()