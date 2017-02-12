#!/usr/bin/env python
# coding=utf-8
id_uav = int(1)
import threading
import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

current_state = State()

def state_cb(data):
    global current_state
    current_state = data

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
def task_main():
    rospy.Subscriber("/mavros{0}/state".format(id_uav),State,state_cb)
    local_pos_pub = rospy.Publisher("/mavros{0}/setpoint_position/local".format(id_uav),PoseStamped,queue_size=10)
    arming_client = rospy.ServiceProxy('/mavros{0}/cmd/arming'.format(id_uav),CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros{0}/set_mode'.format(id_uav),SetMode)
    rate = rospy.Rate(20)
    global current_state
    while not rospy.is_shutdown():
        if current_state.connected == True:
            print "已经接受到无人机状态信息准备执行任务"
            break
        else:
            rate.sleep()

    print"阶段一： 进入 offboard >> 解锁 >> 飞至目标点"
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = -1.0
    yaw = -1.57
    orient_sp = rpy_to_q(0.0,0.0,yaw)
    pose.pose.orientation.w = orient_sp[0]
    pose.pose.orientation.x = orient_sp[1]
    pose.pose.orientation.y = orient_sp[2]
    pose.pose.orientation.z = orient_sp[3]

    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    task_start = rospy.Time.now()
    last_request = rospy.Time.now()
    is_arm_offb = False
    while not rospy.is_shutdown():
        if(current_state.mode != "OFFBOARD") and ((rospy.Time.now() - last_request) > rospy.Duration(3.0)):
            if set_mode_client(0,"OFFBOARD").success :
                print "无人机 %d 进入offboard模式"%id_uav
            last_request = rospy.Time.now()
        else:
            if (current_state.mode == "OFFBOARD") and (not current_state.armed) and ((rospy.Time.now() - last_request) > rospy.Duration(3.0)):
                if arming_client(True).success :
                    print "无人机 %d 已经解锁！"%id_uav
                last_request = rospy.Time.now()
                is_arm_offb = True
        local_pos_pub.publish(pose)
        if (rospy.Time.now() - task_start) > rospy.Duration(90.0):
            break
        rate.sleep()

    print"阶段二： 降落"
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    task_start = rospy.Time.now()
    
    while (not rospy.is_shutdown()) and is_arm_offb :
        local_pos_pub.publish(pose)
        if (rospy.Time.now() - task_start) > rospy.Duration(5.0):
            break
        rate.sleep()

    print"阶段三： 锁定无人机"
    task_start = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.armed:
            if arming_client(False).success :
                print "无人机 %d 已经锁定～"%id_uav
                break
        rate.sleep()
    print"任务结束，按ctrl+c结束该节点"

def task_init():
    rospy.init_node('offboard_test',anonymous=True)
    threads = []
    t1 =threading.Thread(target = task_main)
    threads.append(t1)
    for t in threads:
        t.setDaemon(True)
        t.start()
    print "offboard test 开始"
    rospy.spin()

if __name__ == '__main__':
    try:
        task_init()
    except rospy.ROSInterruptException:
        pass
    print "offboard test 结束"
