#!/usr/bin/env python
# coding=utf-8
NAME = 'demo_test_py'
import rospy
from demo_test.msg import pos_data

def  callback(data):
    header = data.header
    timestamp = header.stamp.to_sec()
    print header.seq, "heard that %d body from Optitrack at %12f"%(data.num, timestamp)
    for i in range(data.num):
        print "=========================================================="
        print "   id : %d"%i
        print "   pos: %.2f %.2f %.2f "%(data.pos[i * 3],data.pos[i * 3 + 1],data.pos[i * 3 + 2])
        print "   q  : %.2f %.2f %.2f %.2f "%(data.q[i * 4],data.q[i*4 +1],data.q[i*4 +2],data.q[i*4 +3])

def listener_with_user_data():
    rospy.Subscriber("demo_udp",pos_data,callback)
    rospy.init_node(NAME,anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_with_user_data()
    except rospy.ROSInterruptException:
        pass
    print "exiting"
