#!/usr/bin/env python
# coding=utf-8

NAME = 'demo_test_py3'
freq_send = int(60)
import rospy
import threading
from geometry_msgs.msg import PoseStamped
from demo_test.msg import pos_data
global_pos_data = pos_data()
mutexA = threading.Lock()
def sending_data():
    global global_pos_data,mutexA
    send_rate = rospy.Rate(freq_send)
    while not rospy.is_shutdown():
        if mutexA.acquire():
            for i in range(global_pos_data.num):
                pub = rospy.Publisher("/mavros{0}/mocap/pose".format(i+1),PoseStamped,queue_size=10)
                pose_stemp = PoseStamped()
                pose_stemp.pose.position.x = global_pos_data.pos[i*3]
                pose_stemp.pose.position.y = global_pos_data.pos[i*3+2]
                pose_stemp.pose.position.z = global_pos_data.pos[i*3+1]
                pose_stemp.pose.orientation.x = global_pos_data.q[i*4]
                pose_stemp.pose.orientation.y = global_pos_data.q[i*4+1]
                pose_stemp.pose.orientation.z = global_pos_data.q[i*4+2]
                pose_stemp.pose.orientation.w = global_pos_data.q[i*4+3]
                pose_stemp.header = global_pos_data.header
                pub.publish(pose_stemp)
            mutexA.release()
        send_rate.sleep()

def  callback(data):
    global global_pos_data,mutexA
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
        global_pos_data = data
        mutexA.release()

def thread_init():
    rospy.Subscriber("demo_udp",pos_data,callback)
    rospy.init_node(NAME,anonymous=True)
    threads = []
    t1 = threading.Thread(target = sending_data)
    threads.append(t1)
    for t in threads:
        t.setDaemon(True)
        t.start()
    rospy.spin()

if __name__ == '__main__':
    try:
        thread_init()
    except rospy.ROSInterruptException:
        pass
    print "exiting"
