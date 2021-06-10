#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

class ImageResizer:
    def __init__(self):
        self.image_pub = rospy.Publisher('/camera/depth/image_rect_raw_resized', Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.init_node('image_resize_node')
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.image_cb)
        rospy.spin()

    def image_cb(self, msg):
        try:
            orig = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            size = (640, 480)
            img = cv2.resize(orig, size)
            pub_msg = self.bridge.cv2_to_imgmsg(img, "16UC1")
            self.image_pub.publish(pub_msg)
            # cv2.imshow('image', img)
            # cv2.waitKey(1)
        except Exception as err:
            print err

if __name__ == '__main__':
    try:
        ir = ImageResizer()
    except rospy.ROSInterruptException:
        pass

