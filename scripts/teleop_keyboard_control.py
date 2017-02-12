#!/usr/bin/env python
# coding=utf-8
import rospy
import sys,select,termios,tty
import math
from geometry_msgs.msg import PoseStamped
id_uav = 1
msg = """
Reading from keyboard and control the Quad!
------------------------------
Moving around:
  qQ           eE
         W                   J
         w                   j :UP
    A a  *  d D
         s                   k :DOWN
         S                   K
    Vv: arm and takeoff
    Bb: land and disarm
"""
moveBindings = {
    'w':(0.2,0.0,0.0,0.0),
    'W':(0.4,0.0,0.0,0.0),
    'd':(0.0,0.2,0.0,0.0),
    'D':(0.0,0.4,0.0,0.0),
    's':(-0.2,0.0,0.0,0.0),
    'S':(-0.4,0.0,0.0,0.0),
    'a':(0.0,-0.2,0.0,0.0),
    'A':(0.0,-0.4,0.0,0.0),
    'q':(0.0,0.0,0.0,-0.3),
    'Q':(0.0,0.0,0.0,-0.3),
    'e':(0.0,0.0,0.0,0.3),
    'E':(0.0,0.0,0.0,0.3),
    'j':(0.0,0.0,0.15,0.0),
    'J':(0.0,0.0,0.3,0.0),
    'k':(0.0,0.0,-0.15,0.0),
    'K':(0.0,0.0,-0.3,0.0),
}

def rpy_to_q(r,p,yaw):
    cosPhi_2 = math.cos(r / 2.0);
    sinPhi_2 = math.sin(r / 2.0);
    cosTheta_2 = math.cos(p / 2.0);
    sinTheta_2 = math.sin(p / 2.0);
    cosPsi_2 = math.cos(yaw / 2.0);
    sinPsi_2 = math.sin(yaw / 2.0);
    w =cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2
    x =sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2
    y =cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2
    z =cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2
    Q = [w,x,y,z]
    return Q

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin],[],[],0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('test_cmd',PoseStamped,queue_size=10)
    rospy.init_node('teleop_keyboard_control',anonymous=True)
    x = 0.0
    y = 0.0
    z = 0.0
    yaw = -1.57
    temp_pose = PoseStamped()
    k_time = 1.0/100
    send_rate = rospy.Rate(100)
    try:
        print msg
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                x = x + moveBindings[key][0]*math.cos(yaw)*k_time - moveBindings[key][1]*math.sin(yaw)*k_time
                y = y + moveBindings[key][0]*math.sin(yaw)*k_time + moveBindings[key][1]*math.cos(yaw)*k_time
                z = z + moveBindings[key][2]*k_time
                yaw = yaw + moveBindings[key][3]*k_time
                temp_pose.pose.position.x = x
                temp_pose.pose.position.y = y
                temp_pose.pose.position.z = z
                orient_sp = rpy_to_q(0.0,0.0,yaw)
                temp_pose.pose.orientation.w = orient_sp[0]
                temp_pose.pose.orientation.x = orient_sp[1]
                temp_pose.pose.orientation.y = orient_sp[2]
                temp_pose.pose.orientation.z = orient_sp[3]
                pub.publish(temp_pose)
            send_rate.sleep()
    except rospy.ROSInterruptException:
        pass
    print"exit"
