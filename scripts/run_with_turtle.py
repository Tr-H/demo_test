#!/usr/bin/env python
import rospy
import time
from demo_test.srv import *
# import nav_msgs
from geometry_msgs.msg import PoseStamped



def test_server_fun():
    # init service
    global teleop_srv_init2
    rospy.wait_for_service('teleop_ctrl_service3')
    teleop_srv_init2 = rospy.ServiceProxy('teleop_ctrl_service3', teleop_ctrl)

    # takeoff
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
    print resp
    time.sleep(10)
    listener()


def listener():
    rospy.init_node('get_odom', anonymous=True)

    rospy.Subscriber("/slam_out_pose", PoseStamped, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    print "start runing with turtle"
    rospy.spin()


def odom_callback(data):
    # move
    
    global teleop_srv_init2
    print 'move once'

    print data.pose.position.x
    print data.pose.position.y
    resp = teleop_srv_init2(teleop_ctrl_mask=teleop_ctrlRequest.MASK_HOVER_POS,
                            hover_pos_x=data.pose.position.x -1 ,
                            hover_pos_y=- data.pose.position.y + 0.25 ,  # y is reverse
                            hover_pos_z=-1.5,
                            hover_pos_yaw=-1.57)
    print resp

if __name__ == '__main__':
    test_server_fun()
    print 'exit!'
