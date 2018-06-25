#!/usr/bin/env python
import rospy
import time
import sys,select,termios,tty
import threading
from demo_test.srv import *

def getKey():
    global settings
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin],[],[],0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def test_server_fun1():
    # init service
    rospy.wait_for_service('teleop_ctrl_service1')
    teleop_srv_init1 = rospy.ServiceProxy('teleop_ctrl_service1',teleop_ctrl)

    # takeoff
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
    print resp
    while not rospy.is_shutdown():
        key = raw_input("input:")
        if (key == 'y' or key == 'Y'):
            print("uav search target")
            break
        elif(key == 'n' or key == 'N'):
            print("user cancel search")
            # land and disarm
            resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                               base_contrl = teleop_ctrlRequest.LAND_DISARM)
            print resp
            print 'task done!'
            time.sleep(2)

            return resp
        else:
            print("input wrong , try again")
    
    num = 1
    for num in range(1,7):
        print ('stang:',num)
        resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                               hover_pos_x = -1.6,
                               hover_pos_y = 2.0 - float(num - 1) * 0.8,
                               hover_pos_z = -0.9,
                               hover_pos_yaw = -1.57)
        print resp
        time.sleep(8)
        resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                               hover_pos_x = 1.0,
                               hover_pos_y = 2.0 - float(num - 1) * 0.8,
                               hover_pos_z = -0.9,
                               hover_pos_yaw = -1.57)
        print resp
        time.sleep(8)
        num = num + 1

"""
    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = 3.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 2
    print 'stage 2'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 3.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 3
    print 'stage 3'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = 2.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 4
    print 'stage 4'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 2.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 5
    print 'stage 5'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = 2.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 6
    print 'stage 6'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 2.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 7
    print 'stage 7'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = 1.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 8
    print 'stage 8'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 1.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 9
    print 'stage 9'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = 1.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 10
    print 'stage 10'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 1.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 11
    print 'stage 11'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = 0.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 12
    print 'stage 12'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 0.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)

    # fly to point 13
    print 'stage 13'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = 0.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 14
    print 'stage 14'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 0.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 15
    print 'stage 15'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = -0.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 16
    print 'stage 16'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = -0.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 17
    print 'stage 17'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = -1.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 18
    print 'stage 18'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = -1.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 19
    print 'stage 19'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = -1.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 20
    print 'stage 20'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = -1.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 21
    print 'stage 21'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = -2.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 22
    print 'stage 22'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = -2.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 23
    print 'stage 23'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = -2.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 24
    print 'stage 24'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = -2.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 25
    print 'stage 25'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = -3.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 26
    print 'stage 26'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = -3.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 27
    print 'stage 27'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = -3.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 28
    print 'stage 28'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = -3.5,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 29
    print 'stage 29'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.6,
                           hover_pos_y = -4.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # fly to point 30
    print 'stage 30'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = -4.0,
                           hover_pos_z = -0.7,
                           hover_pos_yaw = 0.0)
    print resp
    time.sleep(10)

    # land and disarm
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    print resp

    print 'task done!'
    time.sleep(2)

    return resp
"""

def test_server_fun2():
    # init service
    rospy.wait_for_service('teleop_ctrl_service2')
    teleop_srv_init2 = rospy.ServiceProxy('teleop_ctrl_service2',teleop_ctrl)

    # takeoff
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
    print resp
    time.sleep(5)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.00,
                           hover_pos_y = 1.96,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)
    
    # fly to point 2
    #print 'stage 2'
    #resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = -1.44,
    #                       hover_pos_y = 0.8,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(10)

    # fly to point 3
    #print 'stage 3'
    #resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = -1.9,
    #                       hover_pos_y = -0.18,
    #                       hover_pos_z = -1.3,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # fly to point 4
    #print 'stage 4'
    #resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = -1.44,
    #                       hover_pos_y = -2.8,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # # fly to point 5
    # print 'stage 5'
    # resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = -1.2,
    #                        hover_pos_y = 0.0,
    #                        hover_pos_z = -1.2,
    #                        hover_pos_yaw = 0.0)
    # print resp
    # time.sleep(13)

    # # fly to point 6
    # print 'stage 6'
    # resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = 0.0,
    #                        hover_pos_y = 0.0,
    #                        hover_pos_z = -1.2,
    #                        hover_pos_yaw = -1.57)
    # print resp
    # time.sleep(10)

    # land and disarm
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    print resp

    print 'task done!'
    time.sleep(2)

    return resp

def test_server_fun3():
    # init service
    rospy.wait_for_service('teleop_ctrl_service3')
    teleop_srv_init3 = rospy.ServiceProxy('teleop_ctrl_service3',teleop_ctrl)

    # takeoff
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
    print resp
    time.sleep(5)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 0.20,
                           hover_pos_y = 1.96,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)
    
    # fly to point 2
    #print 'stage 2'
    #resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = 0.64,
    #                       hover_pos_y = 0.8,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(10)

    # fly to point 3
    #print 'stage 3'
    #resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = 1.2,
    #                       hover_pos_y = -0.7,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # fly to point 4
    #print 'stage 4'
    #resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = 0.64,
    #                       hover_pos_y = -2.8,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # # fly to point 5
    # print 'stage 5'
    # resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = 0.0,
    #                        hover_pos_y = 0.0,
    #                        hover_pos_z = -1.3,
    #                        hover_pos_yaw = 0.0)
    # print resp
    # time.sleep(13)

    # # fly to point 6
    # print 'stage 6'
    # resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = 0.0,
    #                        hover_pos_y = 1.2,
    #                        hover_pos_z = -1.3,
    #                        hover_pos_yaw = -1.57)
    # print resp
    # time.sleep(10)

    # land and disarm
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    print resp

    print 'task done!'
    time.sleep(2)

    return resp

def test_server_fun4():
    # init service
    rospy.wait_for_service('teleop_ctrl_service4')
    teleop_srv_init4 = rospy.ServiceProxy('teleop_ctrl_service4',teleop_ctrl)

    # takeoff
    resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
    print resp
    time.sleep(5)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -0.4,
                           hover_pos_y = 3.0,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)
    
    # fly to point 2
    #print 'stage 2'
    #resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = -0.4,
    #                       hover_pos_y = 2.6,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(10)

    # fly to point 3
    #print 'stage 3'
    #resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = 0.6,
    #                       hover_pos_y = 0.34,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # fly to point 4
    #print 'stage 4'
    #resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = -0.4,
    #                       hover_pos_y = -1.0,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # # fly to point 5
    # print 'stage 5'
    # resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = 1.2,
    #                        hover_pos_y = 0.0,
    #                        hover_pos_z = -1.2,
    #                        hover_pos_yaw = 0.0)
    # print resp
    # time.sleep(13)

    # # fly to point 6
    # print 'stage 6'
    # resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = 0.0,
    #                        hover_pos_y = -1.2,
    #                        hover_pos_z = -1.3,
    #                        hover_pos_yaw = -1.57)
    # print resp
    # time.sleep(10)

    # land and disarm
    resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    print resp

    print 'task done!'
    time.sleep(2)

    return resp


def thread_init():
    threads = []
    t1 = threading.Thread(target=test_server_fun1)
   # t2 = threading.Thread(target=test_server_fun2)
   # t3 = threading.Thread(target=test_server_fun3)
   # t4 = threading.Thread(target=test_server_fun4)
    threads.append(t1)
   # threads.append(t2)
   # threads.append(t3)
   # threads.append(t4)
    for t in threads:
        t.setDaemon(True)
        t.start()
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("test_quads4", anonymous = True)
    print "start test teleop control service"
    try:
        thread_init()
    except rospy.ROSInterruptException:
        pass
    print 'exit!'
