#!/usr/bin/env python
# coding=utf-8
NAME = 'demo_test_py2'
from apscheduler.schedulers.blocking import BlockingScheduler
import rospy
import threading
from geometry_msgs.msg import PoseStamped
from demo_test.msg import pos_data
global_num = 0
global_pos = []
global_q = []
mutexA = threading.Lock()
def sending_data():
    global global_num,global_pos,global_q,mutexA
    if mutexA.acquire():
        for i in range(global_num):
            pub = rospy.Publisher("/mavros{0}/mocap/pose".format(i+1),PoseStamped,queue_size=10)
            pose_stemp = PoseStamped()
            pose_stemp.pose.position.x = global_pos[i*3]
            pose_stemp.pose.position.y = global_pos[i*3+2]
            pose_stemp.pose.position.z = global_pos[i*3+1]
            pose_stemp.pose.orientation.x = global_q[i*4]
            pose_stemp.pose.orientation.y = global_q[i*4+1]
            pose_stemp.pose.orientation.z = global_q[i*4+2]
            pose_stemp.pose.orientation.w = global_q[i*4+3]
            pub.publish(pose_stemp)
        mutexA.release()

def  callback(data):
    global global_num,global_pos,global_q,mutexA
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
    if mutexA.acquire():
        global_num = data.num
        global_pos = data.pos[:]
        global_q = data.q[:]
        mutexA.release()
def thread_init():
    rospy.Subscriber("demo_udp",pos_data,callback)
    rospy.init_node(NAME,anonymous=True)
    scheduler = BlockingScheduler()
    scheduler.add_job(sending_data,'interval',seconds=0.02, id = 'sending_data_id')
    scheduler.start()
    rospy.spin()

if __name__ == '__main__':
    try:
        thread_init()
    except rospy.ROSInterruptException:
        pass
    print "exiting"
