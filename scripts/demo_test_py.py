#!/usr/bin/env python
# coding=utf-8
NAME = 'demo_test_py'
import threading
import rospy
from demo_test.msg import pos_data
class send_thread(threading.Thread):
    def __init__(self,stopevt = None,name = 'mavsend',th_num = 0):
        threading.Thread.__init__(self)
        self.stopevt = stopevt
        self.name = name
        self.th_num = th_num

    def thread_send_data(self):
        while not rospy.is_shutdown() and self.stopevt.isSet():
            print "num %d thread sending"%self.th_num
            rospy.sleep(1)


def  callback(data):
    header = data.header
    timestamp = header.stamp.to_sec()
    print header.seq, "heard that %d body from Optitrack at %12f"%(data.num, timestamp)
    print "=========================================================="
    for i in range(data.num):
        print "----------------------------------------------------------"
        print "   id : %d"%i
        print "   pos: %.2f %.2f %.2f "%(data.pos[i * 3],data.pos[i * 3 + 1],data.pos[i * 3 + 2])
        print "   q  : %.2f %.2f %.2f %.2f "%(data.q[i * 4],data.q[i*4 +1],data.q[i*4 +2],data.q[i*4 +3])
    print "=========================================================="

def thread_init():
    rospy.Subscriber("demo_udp",pos_data,callback)
    rospy.init_node(NAME,anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        thread_init()
    except rospy.ROSInterruptException:
        pass
    print "exiting"
