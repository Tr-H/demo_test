#!/usr/bin/env python
import rospy
import time
import threading
from demo_test.srv import *

def test_server_fun1():
    # init service
    rospy.wait_for_service('teleop_ctrl_service1')
    teleop_srv_init1 = rospy.ServiceProxy('teleop_ctrl_service1',teleop_ctrl)

#    # takeoff
#    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
#                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
#    print resp
#    time.sleep(5)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 0.0,
                           hover_pos_y = 1.0,
                           hover_pos_z = -1.0,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)
    
    # fly to point 2
    print 'stage 2'
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                          hover_pos_x = 0.0,
                          hover_pos_y = 0.0,
                          hover_pos_z = -1.0,
                          hover_pos_yaw = -1.57)
    print resp
    time.sleep(15)

    # fly to point 3
    #print 'stage 3'
    #resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = 0,
    #                       hover_pos_y = -0.7,
    #                       hover_pos_z = -1.0,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # fly to point 4
    #print 'stage 4'
    #resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = -0.4,
    #                       hover_pos_y = -2.2,
    #                       hover_pos_z = -1.0,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # # fly to point 5
    # print 'stage 5'
    # resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = -1.2,
    #                        hover_pos_y = 0.0,
    #                        hover_pos_z = -1.2,
    #                        hover_pos_yaw = 0.0)
    # print resp
    # time.sleep(13)

    # # fly to point 6
    # print 'stage 6'
    # resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = 0.0,
    #                        hover_pos_y = 0.0,
    #                        hover_pos_z = -1.2,
    #                        hover_pos_yaw = -1.57)
    # print resp
    # time.sleep(10)

    # land and disarm
    resp = teleop_srv_init1(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    print resp

    print 'task done!'
    time.sleep(2)

    return resp


def test_server_fun2():
    # init service
    rospy.wait_for_service('teleop_ctrl_service2')
    teleop_srv_init2 = rospy.ServiceProxy('teleop_ctrl_service2',teleop_ctrl)

#    # takeoff
#    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
#                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
#    print resp
#    time.sleep(5)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.20,
                           hover_pos_y = 1.00,
                           hover_pos_z = -1.0,
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

#    # takeoff
#    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
#                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
#    print resp
#    time.sleep(5)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.20,
                           hover_pos_y = 1.00,
                           hover_pos_z = -1.0,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)
    
    # fly to point 2
    print 'stage 2'
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                          hover_pos_x = 1.20,
                          hover_pos_y = -1.5,
                          hover_pos_z = -1.0,
                          hover_pos_yaw = -1.57)
    print resp
    time.sleep(15)

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

#    # takeoff
#    resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
#                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
#    print resp
#    time.sleep(5)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -0.6,
                           hover_pos_y = -1.0,
                           hover_pos_z = -1.0,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)
    
    # fly to point 2
    print 'stage 2'
    resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                          hover_pos_x = -1.2,
                          hover_pos_y = -1.5,
                          hover_pos_z = -1.0,
                          hover_pos_yaw = -1.57)
    print resp
    time.sleep(15)

    # fly to point 3
    # print 'stage 3'
    # resp = teleop_srv_init4(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = 0.6,
    #                       hover_pos_y = 0.34,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    # print resp
    # time.sleep(20)

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


def test_server_fun5():
    # init service
    rospy.wait_for_service('teleop_ctrl_service5')
    teleop_srv_init5 = rospy.ServiceProxy('teleop_ctrl_service5',teleop_ctrl)

#    # takeoff
#    resp = teleop_srv_init5(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
#                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
#    print resp
#    time.sleep(5)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init5(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 0.6,
                           hover_pos_y = -1.0,
                           hover_pos_z = -1.0,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(10)
    
    # fly to point 2
    print 'stage 2'
    resp = teleop_srv_init5(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                          hover_pos_x = 0.0,
                          hover_pos_y = -1.5,
                          hover_pos_z = -1.0,
                          hover_pos_yaw = -1.57)
    print resp
    time.sleep(15)

    # fly to point 3
    #print 'stage 3'
    #resp = teleop_srv_init5(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = 0.6,
    #                       hover_pos_y = 0.34,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # fly to point 4
    #print 'stage 4'
    #resp = teleop_srv_init5(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                       hover_pos_x = -0.4,
    #                       hover_pos_y = -1.0,
    #                       hover_pos_z = -1.2,
    #                       hover_pos_yaw = -1.57)
    #print resp
    #time.sleep(20)

    # # fly to point 5
    # print 'stage 5'
    # resp = teleop_srv_init5(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = 1.2,
    #                        hover_pos_y = 0.0,
    #                        hover_pos_z = -1.2,
    #                        hover_pos_yaw = 0.0)
    # print resp
    # time.sleep(13)

    # # fly to point 6
    # print 'stage 6'
    # resp = teleop_srv_init5(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
    #                        hover_pos_x = 0.0,
    #                        hover_pos_y = -1.2,
    #                        hover_pos_z = -1.3,
    #                        hover_pos_yaw = -1.57)
    # print resp
    # time.sleep(10)

    # land and disarm
    resp = teleop_srv_init5(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    print resp

    print 'task done!'
    time.sleep(2)

    return resp


def thread_init():
    threads = []
    t1 = threading.Thread(target=test_server_fun1)
    t2 = threading.Thread(target=test_server_fun2)
    t3 = threading.Thread(target=test_server_fun3)
    t4 = threading.Thread(target=test_server_fun4)
    t5 = threading.Thread(target=test_server_fun5)
    threads.append(t1)
    threads.append(t2)
    threads.append(t3)
    threads.append(t4)
    threads.append(t5)
    for t in threads:
        t.setDaemon(True)
        t.start()
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("test_quads5", anonymous = True)
    print "start test teleop control service"
    try:
        thread_init()
    except rospy.ROSInterruptException:
        pass
    print 'exit!'
