#!/usr/bin/env python
# coding=utf-8
import rospy
import sys,select,termios,tty
import math
import argparse
import threading
import teleop_keyboard_control_2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from demo_test.srv import *

