#!/usr/bin/env python
# coding=utf-8
import rospy
import sys,select,termios,tty
import math
import argparse
import threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from demo_test.srv import * 
id_uav = 1
name_ros_node = 'teleop_keyboard_control{0}'

temp_pose = PoseStamped()
mutexA = threading.Lock()
current_state = State()
is_arm_offb = False

home_x = 0.0
home_y = 0.0
home_z = 0.0
home_yaw = -1.57
home_init_done = False

msg = """
Reading from keyboard and control the Quad!
--------------------------------------------------------------
Moving around:
                             tT: oftraject mode enable
                             oO: ofwhycon mode enable
                             pP: offboard force enable
                             hH: fixed hover enable

          qQ           eE
                 W                             J
                 w                             j :UP
            A a  *  d D
                 s                             k :DOWN
                 S                             K
                       Vv: arm and takeoff
                       Bb: land and disarm
            ctrl+c: exit
--------------------------------------------------------------
Moving around:
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
    'q':(0.0,0.0,0.0,-0.2),
    'Q':(0.0,0.0,0.0,-0.3),
    'e':(0.0,0.0,0.0,0.2),
    'E':(0.0,0.0,0.0,0.3),
    'j':(0.0,0.0,-0.15,0.0),
    'J':(0.0,0.0,-0.3,0.0),
    'k':(0.0,0.0,0.15,0.0),
    'K':(0.0,0.0,0.3,0.0),
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

def limit_fram_xy(fram_x,fram_y):
    length_xy = math.sqrt(math.pow(fram_x,2) + math.pow(fram_y,2))
    if math.fabs(length_xy) > 1.65:
        normal_x = fram_x/length_xy
        normal_y = fram_y/length_xy
        res = [normal_x*1.65,normal_y*1.65]
    else:
        res = [fram_x,fram_y]
    return res

def limit_fram_z(fram_z):
    if math.fabs(fram_z) > 1.5:
        res = math.copysign(1.5,fram_z)
    else:
        res = fram_z
    return res

def limit_yaw(yaw):
    if yaw > math.pi:
        res = yaw - 2.0 * math.pi
    elif yaw < -math.pi:
        res = yaw + 2.0 * math.pi
    else:
        res = yaw
    return res

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin],[],[],0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def command_send():
    global temp_pose,id_uav,mutexA
    pub = rospy.Publisher("/mavros{0}/setpoint_position/local".format(id_uav),PoseStamped,queue_size=10)
    send_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if mutexA.acquire():
            pub.publish(temp_pose)
            mutexA.release()
        send_rate.sleep()
    print "command_send exit"

def arm_and_takeoff():
    global current_state,is_arm_offb,temp_pose,mutexA,id_uav
    if mutexA.acquire():
        temp_pose.pose.position.z = -1.0
        mutexA.release()
    last_request = rospy.Time.now()
    is_arm_offb = False
    arming_client = rospy.ServiceProxy('/mavros{0}/cmd/arming'.format(id_uav),CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros{0}/set_mode'.format(id_uav),SetMode)
    print " ready for takeoffing "
    while not rospy.is_shutdown():
        if(current_state.mode != "OFFBOARD") and ((rospy.Time.now() - last_request) > rospy.Duration(3.0)):
            if set_mode_client(0,"OFFBOARD").success :
                print "无人机 %d 进入offboard模式"%id_uav
            last_request = rospy.Time.now()
        else:
            if (current_state.mode == "OFFBOARD") and (not current_state.armed) and ((rospy.Time.now() - last_request) > rospy.Duration(3.0)):
                if arming_client(True).success :
                    print "无人机 %d 已经解锁！"%id_uav
                    is_arm_offb = True
                    break
            elif current_state.armed:
                print "无人机 %d 已经解锁！"%id_uav
                is_arm_offb = True
                break

def offboard_whycon_enable():
    global current_state,is_arm_offb,temp_pose,mutexA,id_uav
    set_ofwhycon_client = rospy.ServiceProxy('/mavros{0}/set_mode'.format(id_uav),SetMode)
    print" ready for follow target "
    while not rospy.is_shutdown():
        if set_ofwhycon_client(0,"OFWHYCON").success :
            print "无人机 %d 进入OFWHYCON模式"%id_uav
            break

def offboard_traject_enable():
    global current_state,is_arm_offb,temp_pose,mutexA,id_uav
    set_ofwhycon_client = rospy.ServiceProxy('/mavros{0}/set_mode'.format(id_uav),SetMode)
    print" ready for traject mode "
    while not rospy.is_shutdown():
        if set_ofwhycon_client(0,"OFTRAJECT").success :
            print "无人机 %d 进入OFTRAJECT模式"%id_uav
            break

def offboard_force_enable():
    global home_x,home_y,home_z,home_yaw,current_state,is_arm_offb,temp_pose,mutexA,id_uav,home_init_done
    home_init_done = False
    sub_once = rospy.Subscriber("/mavros{0}/mocap/pose".format(id_uav),PoseStamped,home_p_cb)
    rate_wait = rospy.Rate(40)
    while not rospy.is_shutdown():
        if home_init_done:
	    if mutexA.acquire():
		temp_pose.pose.position.x = home_x
		temp_pose.pose.position.y = home_y
                temp_pose.pose.position.z = home_z
                orient_sp = rpy_to_q(0.0,0.0,home_yaw)
                temp_pose.pose.orientation.w = orient_sp[0]
                temp_pose.pose.orientation.x = orient_sp[1]
                temp_pose.pose.orientation.y = orient_sp[2]
                temp_pose.pose.orientation.z = orient_sp[3]
                mutexA.release()
            sub_once.unregister()
            break
        else:
            rate_wait.sleep()
    set_offboard_client = rospy.ServiceProxy('/mavros{0}/set_mode'.format(id_uav),SetMode)
    while not rospy.is_shutdown():
        if set_offboard_client(0,"OFFBOARD").success :
            print "无人机 %d 进入offboard模式"%id_uav
            break

def fixed_hover_enable():
    global temp_pose,id_uav,is_arm_offb,mutexA,yaw
    set_fixhover_client = rospy.ServiceProxy('/mavros{0}/set_mode'.format(id_uav),SetMode)
    while not rospy.is_shutdown():
        if set_fixhover_client(0,"OFFBOARD").success :
            print "无人机 %d 准备进入offboard模式"%id_uav
            x = float(raw_input('target X :'))
            y = float(raw_input('target Y :'))
            z = float(raw_input('target Z :'))
            yaw = float(raw_input('target Yaw :'))
            if is_arm_offb and mutexA.acquire():
                [hover_x,hover_y] = limit_fram_xy(x,y)
                hover_z = limit_fram_z(z)
                yaw = limit_yaw(yaw)
                temp_pose.pose.position.x = hover_x
                temp_pose.pose.position.y = hover_y
                temp_pose.pose.position.z = hover_z
                orient_sp = rpy_to_q(0.0,0.0,yaw)
                temp_pose.pose.orientation.w = orient_sp[0]
                temp_pose.pose.orientation.x = orient_sp[1]
                temp_pose.pose.orientation.y = orient_sp[2]
                temp_pose.pose.orientation.z = orient_sp[3]
                mutexA.release()
                print "无人机 %d Fixed hover has done"%id_uav
                break
            else:
                print "无人机 %d be not ready to hover"%id_uav
                break


def land_and_disarm():
    global current_state,is_arm_offb,temp_pose,mutexA,id_uav
    set_offboard_client = rospy.ServiceProxy('/mavros{0}/set_mode'.format(id_uav),SetMode)
    while not rospy.is_shutdown():
        if set_offboard_client(0,"OFFBOARD").success :
            print "无人机 %d 进入offboard模式"%id_uav
            break
    if mutexA.acquire():
        temp_pose.pose.position.z = 0.05
        mutexA.release()
    last_request = rospy.Time.now()
    arming_client = rospy.ServiceProxy('/mavros{0}/cmd/arming'.format(id_uav),CommandBool)
    print " ready for landing "
    while (not rospy.is_shutdown()) and is_arm_offb :
        if current_state.armed and ((rospy.Time.now() - last_request) > rospy.Duration(3.0)):
            if arming_client(False).success :
                print "无人机 %d 已经锁定～"%id_uav
                break
            elif not current_state.armed:
                print "无人机 %d 已经锁定～"%id_uav
                break
    
def state_cb(data):
    global current_state
    current_state = data

def home_p_cb(data):
    global home_x,home_y,home_z,home_yaw,home_init_done
    home_x = data.pose.position.x
    home_y = data.pose.position.z
    home_z = -data.pose.position.y
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    z = data.pose.orientation.z
    w = data.pose.orientation.w
    home_yaw = -math.atan2(2*(w*y + z*x),(1 -2*(x*x + y*y)))
    home_init_done = True

def init_home_point():
    global home_x,home_y,home_z,home_yaw,home_init_done,id_uav
    sub_once = rospy.Subscriber("/mavros{0}/mocap/pose".format(id_uav),PoseStamped,home_p_cb)
    rate_wait = rospy.Rate(10)
    x = home_x
    y = home_y
    z = home_z
    yaw = home_yaw
    while not rospy.is_shutdown():
        if home_init_done:
            x = home_x
            y = home_y
            z = home_z
            yaw = home_yaw
            sub_once.unregister()
            break
        else:
            rate_wait.sleep()
    home_p = [x,y,z,yaw]
    print "[ home_init ] done position: %.2f %.2f %.2f yaw: %.2f"%(x,y,z,yaw)
    return home_p

def task_main():
    global msg,temp_pose,id_uav,is_arm_offb,yaw 
    rospy.Subscriber("/mavros{0}/state".format(id_uav),State,state_cb)
    home = init_home_point()
    x = home[0]
    y = home[1]
    z = home[2]
    yaw = home[3]
    if mutexA.acquire():
        temp_pose.pose.position.x = x
        temp_pose.pose.position.y = y
        temp_pose.pose.position.z = z
        orient_sp = rpy_to_q(0.0,0.0,yaw)
        temp_pose.pose.orientation.w = orient_sp[0]
        temp_pose.pose.orientation.x = orient_sp[1]
        temp_pose.pose.orientation.y = orient_sp[2]
        temp_pose.pose.orientation.z = orient_sp[3]
        mutexA.release()
    last_time = rospy.Time.now()
    t_step = 0.1
    print msg
    #while(1):
    while not rospy.is_shutdown():
        key = getKey()
        dt = (rospy.Time.now()-last_time).to_sec()
        if dt > 0.05 :
            if key in moveBindings.keys():
                if is_arm_offb and mutexA.acquire():
                    #x = limit_fram_xy(temp_pose.pose.position.x + moveBindings[key][0]*math.cos(yaw)*t_step- moveBindings[key][1]*math.sin(yaw)*t_step)
                    #y = limit_fram_xy(temp_pose.pose.position.y + moveBindings[key][0]*math.sin(yaw)*t_step+ moveBindings[key][1]*math.cos(yaw)*t_step)
                    [x,y] = limit_fram_xy((temp_pose.pose.position.x + moveBindings[key][0]*math.cos(yaw)*t_step- moveBindings[key][1]*math.sin(yaw)*t_step),(temp_pose.pose.position.y + moveBindings[key][0]*math.sin(yaw)*t_step+ moveBindings[key][1]*math.cos(yaw)*t_step))
                    z = limit_fram_z(temp_pose.pose.position.z + moveBindings[key][2]*t_step)
                    #there should increase a function for q_to_rpy 
                    yaw = limit_yaw(yaw + moveBindings[key][3]*t_step)
                    temp_pose.pose.position.x = x
                    temp_pose.pose.position.y = y
                    temp_pose.pose.position.z = z
                    orient_sp = rpy_to_q(0.0,0.0,yaw)
                    temp_pose.pose.orientation.w = orient_sp[0]
                    temp_pose.pose.orientation.x = orient_sp[1]
                    temp_pose.pose.orientation.y = orient_sp[2]
                    temp_pose.pose.orientation.z = orient_sp[3]
                    mutexA.release()
            elif (key == 'v' or key == 'V'):
                arm_and_takeoff()
            elif (key == 'b' or key == 'B'):
                land_and_disarm()
            elif (key == 'o' or key == 'O'):
                offboard_whycon_enable()
            elif (key == 'p' or key == 'P'):
                offboard_force_enable()
            elif (key == 't' or key == 'T'):
                offboard_traject_enable()
            elif (key == 'h' or key == 'H'):
                fixed_hover_enable()
            elif (key == '\x03'):
                break 
            last_time = rospy.Time.now()
    print "take_main exit"

def teleop_ctrl_srv_fun(req):
    global yaw,temp_pose,mutexA
    if (req.teleop_ctrl_mask == req.MASK_ARM_DISARM):
        process_valid = False
        if (req.base_contrl == req.ARM_TAKEOFF):
            print "arm and takeoff"
            arm_and_takeoff()
            process_valid = True 
        elif (req.base_contrl == req.LAND_DISARM):
            print "land and disarm"
            land_and_disarm()
            process_valid = True 
        elif (req.base_contrl == req.FORCE_DISARM):
            print "force disarm"
            #offboard_force_enable()
            process_valid = True   
        else:
            print "wrong arg!"
            process_valid = False 
        return teleop_ctrlResponse(success = process_valid , result = req.MASK_ARM_DISARM) 
    elif (req.teleop_ctrl_mask == req.MASK_FLIGHT_MODE): 
        process_valid = False 
        if (req.flight_mode == req.MODE_OFFBOARD):
            offboard_force_enable()
            process_valid = True
        elif (req.flight_mode == req.MODE_TRAJECT):  
            print "flight mode : TRAJECT"
            offboard_traject_enable()
            process_valid = True 
        elif (req.flight_mode == req.MODE_WHYCON):  
            print "flight mode : WHYCON"
            offboard_whycon_enable()
            process_valid = True 
        else:
            print "wrong arg!"
            process_valid = False
        return teleop_ctrlResponse(success = process_valid, result = req.MASK_FLIGHT_MODE) 
    elif (req.teleop_ctrl_mask == req.MASK_HOVER_POS):
        print ("hover at pos [%f %f %f %f]" % (req.hover_pos_x,req.hover_pos_y,req.hover_pos_z,req.hover_pos_yaw))
        x = req.hover_pos_x
        y = req.hover_pos_y
        z = req.hover_pos_z
        temp_yaw = req.hover_pos_yaw
        if mutexA.acquire():
            [hover_x,hover_y] = limit_fram_xy(x,y)
            hover_z = limit_fram_z(z)
            yaw = limit_yaw(temp_yaw)
            temp_pose.pose.position.x = hover_x
            temp_pose.pose.position.y = hover_y
            temp_pose.pose.position.z = hover_z
            orient_sp = rpy_to_q(0.0,0.0,yaw)
            temp_pose.pose.orientation.w = orient_sp[0]
            temp_pose.pose.orientation.x = orient_sp[1]
            temp_pose.pose.orientation.y = orient_sp[2]
            temp_pose.pose.orientation.z = orient_sp[3]
            mutexA.release()
        return teleop_ctrlResponse(success = True , result = req.MASK_HOVER_POS) 
    elif (req.teleop_ctrl_mask == req.MASK_WASD):
        if (req.teleop_keyboard == req.TELEOP_W ):
            print "W"
            key = "W" 
        elif (req.teleop_keyboard == req.TELEOP_A ):
            print "A"
            key = "A"   
        elif (req.teleop_keyboard == req.TELEOP_S ):
            print "S"
            key = "S"
        elif (req.teleop_keyboard == req.TELEOP_D ):
            print "D"
            key = "D" 
        elif (req.teleop_keyboard == req.TELEOP_Q ):
            print "Q"
            key = "Q" 
        elif (req.teleop_keyboard == req.TELEOP_E ):
            print "E"
            key = "E" 
        elif (req.teleop_keyboard == req.TELEOP_J ):
            print "J"
            key = "J" 
        elif (req.teleop_keyboard == req.TELEOP_K ):
            print "K"
            key = "K"
        else:
            print "wrong arg!"
            return teleop_ctrlResponse(success = False, result = req.MASK_WASD)
        t_step = 1.0

        [x,y] = limit_fram_xy((temp_pose.pose.position.x + moveBindings[key][0]*math.cos(yaw)*t_step- moveBindings[key][1]*math.sin(yaw)*t_step),(temp_pose.pose.position.y + moveBindings[key][0]*math.sin(yaw)*t_step+ moveBindings[key][1]*math.cos(yaw)*t_step))
        z = limit_fram_z(temp_pose.pose.position.z + moveBindings[key][2]*t_step)
        #there should increase a function for q_to_rpy 
        yaw = limit_yaw(yaw + moveBindings[key][3]*t_step)
        temp_pose.pose.position.x = x
        temp_pose.pose.position.y = y
        temp_pose.pose.position.z = z
        orient_sp = rpy_to_q(0.0,0.0,yaw)
        temp_pose.pose.orientation.w = orient_sp[0]
        temp_pose.pose.orientation.x = orient_sp[1]
        temp_pose.pose.orientation.y = orient_sp[2]
        temp_pose.pose.orientation.z = orient_sp[3]
        return teleop_ctrlResponse(success = True , result = req.MASK_WASD)
    else:
        print "wrong arg!"
        return teleop_ctrlResponse(success = False, result = req.MASK_WASD)

def teleop_ctrl_srv_init():
    #rospy.init_node('test_teleop_control_srv',anonymous = True)
    rospy.Service('teleop_ctrl_service{0}'.format(id_uav), teleop_ctrl, teleop_ctrl_srv_fun)
    print "Ready for receive srv"

def thread_init():
    threads = []
    t1 = threading.Thread(target = task_main)
    t2 = threading.Thread(target = command_send)
    t3 = threading.Thread(target = teleop_ctrl_srv_init)
    threads.append(t1)
    threads.append(t2)
    threads.append(t3)
    for t in threads:
        t.setDaemon(True)
        t.start()
    rospy.spin()

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    parser = argparse.ArgumentParser()
    parser.add_argument("idofuav",nargs = '?', help = "输入被控无人机的ID", default = 1,
                        type = int)
    args = parser.parse_args()
    id_uav = args.idofuav

    rospy.init_node(name_ros_node.format(id_uav),anonymous=True)
    try:
        thread_init()
    except rospy.ROSInterruptException:
        pass
    print "exit"
