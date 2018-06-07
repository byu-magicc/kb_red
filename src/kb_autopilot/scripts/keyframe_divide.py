#!/usr/bin/env python
import rospy
from relative_nav.msg import Keyframe
from sensor_msgs.msg import Image

pub = rospy.Publisher('depth', Image, queue_size=10)

def callback(data):
    pub.publish(data.depth)

def listener():
    rospy.init_node('keyframe_devide', anonymous=True)

    rospy.Subscriber("keyframe", Keyframe, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
