#!/usr/bin/env python
import rospy
import time
from demo_test.srv import *

def test_server_fun():
    # init service
    rospy.wait_for_service('teleop_ctrl_service2')
    rospy.wait_for_service('teleop_ctrl_service3')
    teleop_srv_init2 = rospy.ServiceProxy('teleop_ctrl_service2',teleop_ctrl)
    teleop_srv_init3 = rospy.ServiceProxy('teleop_ctrl_service3',teleop_ctrl)

    # takeoff
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
    print resp
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
    print resp
    time.sleep(20)

    # fly to point 1
    print 'stage 1'
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 1.0,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 0.0,
                           hover_pos_y = 0.0,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(12)
    
    # fly to point 2
    print 'stage 2'
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.0,
                           hover_pos_y = -1.0,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(12)

    # fly to point 3
    print 'stage 3'
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 1.0,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.0,
                           hover_pos_y = -1.0,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(12)

    # fly to point 4
    print 'stage 4'
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 1.0,
                           hover_pos_y = 1.0,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = -1.0,
                           hover_pos_y = -1.0,
                           hover_pos_z = -1.2,
                           hover_pos_yaw = -1.57)
    print resp
    time.sleep(12)

    # land and disarm
    resp = teleop_srv_init2(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    print resp
    resp = teleop_srv_init3(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    print resp

    print 'task done!'
    time.sleep(5)

    return resp


if __name__ == "__main__":
    print "start test teleop control service"
    test_server_fun()
    print 'exit!'
