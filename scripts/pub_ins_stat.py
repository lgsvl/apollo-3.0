#!/usr/bin/env python

import rospy
import pb_msgs.msg

def pub_ins_stat():
    pub = rospy.Publisher('/apollo/sensor/gnss/ins_stat', pb_msgs.msg.InsStat, queue_size = 10)
    rospy.init_node('pub_ins_stat', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = pb_msgs.msg.InsStat()
        now = rospy.get_rostime()
        msg.header.timestamp_sec = now.secs
        msg.ins_status = 3
        msg.pos_type = 56
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        pub_ins_stat()
    except rospy.ROSInterruptException:
        pass
