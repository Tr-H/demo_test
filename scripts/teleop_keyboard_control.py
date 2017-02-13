#!/usr/bin/env python
# coding=utf-8
import rospy
import sys,select,termios,tty
import math
import threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
id_uav = 1
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

def limit_fram(fram_i):
    if math.fabs(fram_i) > 1.5:
        res = math.copysign(1.5,fram_i)
    else:
        res = fram_i
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
    global temp_pose,id_uav
    pub = rospy.Publisher("/mavros{0}/setpoint_position/local".format(id_uav),PoseStamped,queue_size=10)
    send_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if mutexA.acquire():
            pub.publish(temp_pose)
            mutexA.release()
        send_rate.sleep()
    print"command_send exit"

def arm_and_takeoff():
    global current_state,is_arm_offb,temp_pose,mutexA,id_uav
    if mutexA.acquire():
        temp_pose.pose.position.z = -1.0
        mutexA.release()
    last_request = rospy.Time.now()
    is_arm_offb = False
    arming_client = rospy.ServiceProxy('/mavros{0}/cmd/arming'.format(id_uav),CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros{0}/set_mode'.format(id_uav),SetMode)
    print" ready for takeoffing "
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

def land_and_disarm():
    global current_state,is_arm_offb,temp_pose,mutexA,id_uav
    if mutexA.acquire():
        temp_pose.pose.position.z = 0.0
        mutexA.release()
    last_request = rospy.Time.now()
    arming_client = rospy.ServiceProxy('/mavros{0}/cmd/arming'.format(id_uav),CommandBool)
    print" ready for landing "
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
    global msg,temp_pose,id_uav,is_arm_offb 
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
    t_step = 0.08
    print msg
    while(1):
        key = getKey()
        dt = (rospy.Time.now()-last_time).to_sec()
        if dt > 0.05 :
            if key in moveBindings.keys():
                if is_arm_offb and mutexA.acquire():
                    x = limit_fram(temp_pose.pose.position.x + moveBindings[key][0]*math.cos(yaw)*t_step- moveBindings[key][1]*math.sin(yaw)*t_step)
                    y = limit_fram(temp_pose.pose.position.y + moveBindings[key][0]*math.sin(yaw)*t_step+ moveBindings[key][1]*math.cos(yaw)*t_step)
                    z = limit_fram(temp_pose.pose.position.z + moveBindings[key][2]*t_step)
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
            elif (key == '\x03'):
                break
            last_time = rospy.Time.now()
    print"take_main exit"

def thread_init():
    threads = []
    t1 = threading.Thread(target = task_main)
    t2 = threading.Thread(target = command_send)
    threads.append(t1)
    threads.append(t2)
    for t in threads:
        t.setDaemon(True)
        t.start()
    rospy.spin()

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard_control',anonymous=True)
    try:
        thread_init()
    except rospy.ROSInterruptException:
        pass
    print"exit"
