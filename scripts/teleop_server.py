#!/usr/bin/env python
import rospy
from test.srv import teleop_ctrl

def teleop_ctrl_srv_fun(req):
    if (req.teleop_ctrl_mask == req.MASK_ARM_DISARM):
        if (req.base_contrl == req.ARM_TAKEOFF):
            print "arm and takeoff"
            return req.success(True)
        elif (req.base_contrl == req.LAND_DISARM):
            print "land and disarm"
            return req.success(True)
        elif (req.base_contrl == req.FORCE_DISARM):
            print "force disarm"
            return req.success(True)
        else:
            print "wrong arg!"
            return req.success(True)   
    elif (req.teleop_ctrl_mask == req.MASK_FLIGHT_MODE):
        if (req.flight_mode == req.MODE_OFFBOARD):
            print "flight mode : OFFBOARD"
            return req.success(True)
        elif (req.flight_mode == req.MODE_TRAJECT):  
            print "flight mode : TRAJECT"
            return req.success(True)
        elif (req.flight_mode == req.MODE_WHYCON):  
            print "flight mode : WHYCON"
            return req.success(True)
        else:
            print "wrong arg!"
            return req.success(True)
    elif (req.teleop_ctrl_mask == req.MASK_HOVER_POS):
        print ("hover at pos [%f %f %f %f]" % (req.hover_pos_x,req.hover_pos_y,req.hover_pos_z,req.hover_pos_yaw))
        return req.success(True)
    elif (req.teleop_ctrl_mask == req.MASK_WASD):
        if (req.teleop_keyboard == req.TELEOP_W ):
            print "W"
            return req.success(True)
        elif (req.teleop_keyboard == req.TELEOP_A ):
            print "A"
            return req.success(True)
        elif (req.teleop_keyboard == req.TELEOP_S ):
            print "S"
            return req.success(True)
        elif (req.teleop_keyboard == req.TELEOP_D ):
            print "D"
            return req.success(True)
        elif (req.teleop_keyboard == req.TELEOP_Q ):
            print "Q"
            return req.success(True)
        elif (req.teleop_keyboard == req.TELEOP_E ):
            print "E"
            return req.success(True)
        elif (req.teleop_keyboard == req.TELEOP_J ):
            print "J"
            return req.success(True)
        elif (req.teleop_keyboard == req.TELEOP_K ):
            print "K"
            return req.success(True)
        else:
            print "wrong arg!"
            return req.success(True)
    else:
        print "wrong arg!"
        return req.success(True)

def teleop_ctrl_srv_init():
    rospy.init_node('test_teleop_control_srv')
    rospy.Service('quad_srv_1', teleop_ctrl, teleop_ctrl_srv_fun)
    print "Ready for receive srv"
    rospy.spin()

if __name__ == "__main__":
    teleop_ctrl_srv_init()
