#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)



if __name__ == '__main__':
    rospy.init_node('vision_rec', anonymous=True)
    rospy.Subscriber('vision_data', String, callback)
    rospy.spin()
