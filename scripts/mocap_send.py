#!/usr/bin/env python
# coding=utf-8

NAME = 'mocap_send_py'
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
            pose_stemp = PoseStamped()
            pose_stemp.header = global_pos_data.header
            for i in range(global_pos_data.num):
                pub = rospy.Publisher("/mavros{0}/mocap/pose".format(i+1),PoseStamped,queue_size=10)
                pose_stemp.pose.position.x = global_pos_data.pos[i*3]
                pose_stemp.pose.position.y = global_pos_data.pos[i*3+1]
                pose_stemp.pose.position.z = global_pos_data.pos[i*3+2]
                pose_stemp.pose.orientation.x = global_pos_data.q[i*4]
                pose_stemp.pose.orientation.y = global_pos_data.q[i*4+1]
                pose_stemp.pose.orientation.z = global_pos_data.q[i*4+2]
                pose_stemp.pose.orientation.w = global_pos_data.q[i*4+3]
                pub.publish(pose_stemp)
            mutexA.release()
        send_rate.sleep()

def  callback(data):
    global global_pos_data,mutexA
    header = data.header
    timestamp = header.stamp.to_sec()
    print "数据",header.seq, "：共收到 %d 个刚体的数据 时间 %12f"%(data.num, timestamp)
    print "=========================================================="
    for i in range(data.num):
        print "----------------------------------------------------------"
        print "   ID : %d"%i
        print "   位置 x: %.4f y: %.4f z: %.4f "%(data.pos[i * 3],data.pos[i * 3 + 1],data.pos[i * 3 + 2])
        print "   姿态 x: %.4f y: %.4f z: %.4f w: %.4f "%(data.q[i * 4],data.q[i*4 +1],data.q[i*4 +2],data.q[i*4 +3])
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
    print "开始接收数据包"
    rospy.spin()

if __name__ == '__main__':
    try:
        thread_init()
    except rospy.ROSInterruptException:
        pass
    print "exiting"
