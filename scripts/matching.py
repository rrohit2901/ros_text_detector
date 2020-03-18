#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from text_detector.msg import sample
from std_msgs.msg import String

rospy.init_node("matching", anonymous = True)
final_pub = rospy.Publisher("final_result", sample, queue_size = 1)
rate = rospy.Rate(10)
def matching_callback(mess):
    a = b'\xd1\x82\xd1\x80\xd1\x83\xd1\x82\xd0\xb5\xd0\xbd\xd1\x8c'
    text = mess.data
    text1 = text.decode("utf-8")
    flag = "different"
    a = a.decode("utf-8")
    if(text1 == a):
        flag = "same"
    rospy.loginfo("These texts are %s", flag)
    f_mess = sample()
    f_mess.x = mess.x
    f_mess.y = mess.y
    f_mess.data = flag
    rospy.loginfo("Co-ordinates of center are (%d,%d)", mess.x, mess.y)
    final_pub.publish(f_mess)
    rate.sleep()

def main():
    rospy.Subscriber("detection_result", sample, matching_callback)
    rospy.spin()

if __name__ == '__main__':
    main()