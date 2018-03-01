#!/usr/bin/env python
import rospy
import time
from demo_test.srv import *

def test_server_fun():
    rospy.wait_for_service('teleop_ctrl_service3')
    teleop_srv_init = rospy.ServiceProxy('teleop_ctrl_service3',teleop_ctrl)
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.ARM_TAKEOFF)
    time.sleep(9)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.FORCE_DISARM)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_FLIGHT_MODE,
                           flight_mode = teleop_ctrlRequest.MODE_OFFBOARD)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_FLIGHT_MODE,
                           flight_mode = teleop_ctrlRequest.MODE_TRAJECT)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_FLIGHT_MODE,
                           flight_mode = teleop_ctrlRequest.MODE_WHYCON)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 0.1,
                           hover_pos_y = 0.2,
                           hover_pos_z = 0.3,
                           hover_pos_yaw = -0.3)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_HOVER_POS,
                           hover_pos_x = 0.1,
                           hover_pos_y = -0.2,
                           hover_pos_z = -0.3,
                           hover_pos_yaw = -0.3)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_WASD,
                           teleop_keyboard = teleop_ctrlRequest.TELEOP_W)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_WASD,
                           teleop_keyboard = teleop_ctrlRequest.TELEOP_A)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_WASD,
                           teleop_keyboard = teleop_ctrlRequest.TELEOP_S)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_WASD,
                           teleop_keyboard = teleop_ctrlRequest.TELEOP_D)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_WASD,
                           teleop_keyboard = teleop_ctrlRequest.TELEOP_E)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_WASD,
                           teleop_keyboard = teleop_ctrlRequest.TELEOP_Q)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_WASD,
                           teleop_keyboard = teleop_ctrlRequest.TELEOP_J)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_WASD,
                           teleop_keyboard = teleop_ctrlRequest.TELEOP_K)
    time.sleep(3)
    print resp
    resp = teleop_srv_init(teleop_ctrl_mask = teleop_ctrlRequest.MASK_ARM_DISARM,
                           base_contrl = teleop_ctrlRequest.LAND_DISARM)
    time.sleep(3)
    print resp
    return resp


if __name__ == "__main__":
    print "start test teleop control service"
    test_server_fun()
