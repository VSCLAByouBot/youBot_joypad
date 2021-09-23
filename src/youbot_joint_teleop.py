#!/usr/bin/env python
# *****************************************************
#  Keyboard teleop control for KUKA youBot arm
# *****************************************************
#  Reference : 
#   > youbot_keyboard_teleop.py 
# 				from youbot_driver_ros_interface
# *****************************************************

import rospy
from youbot_joypad.youbot_control import youbot_arm_control
import sys, select, termios, tty, signal

_debug = False

msg = """
Reading from the keyboard and Publishing to youBot!
------------------------------------------------------
Arm Control:      

  Joint   1    2    3    4    5
    +     q    w    e    r    t
    -     a    s    d    f    g

b/n : increase/decrease max speeds of the arm
y   : show current joint state
h   : show current end-effector position
anything else : stop

CTRL-C to quit
"""

jointBindings = {
#	J1 ~ J5
	'q': ( 1, 0, 0, 0, 0),
	'a': (-1, 0, 0, 0, 0),
    'w': ( 0, 1, 0, 0, 0),
	's': ( 0,-1, 0, 0, 0),
    'e': ( 0, 0, 1, 0, 0),
	'd': ( 0, 0,-1, 0, 0),
    'r': ( 0, 0, 0, 1, 0),
	'f': ( 0, 0, 0,-1, 0),
    't': ( 0, 0, 0, 0, 1),
	'g': ( 0, 0, 0, 0,-1),
}
ratioBindings = {
	'b' :  0.1,
	'n' : -0.1, 
}

ratio = 0.1

class TimeoutException(Exception): 
    pass 


def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()
    
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1) #this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
       key = sys.stdin.read(1)
    except TimeoutException:
       return "-"
    finally:
       signal.signal(signal.SIGALRM, old_handler)

    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def current_ratio(ratio):
	return "currently:\tratio =  %1f \r" % (ratio)


if __name__ =="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('youbot_joint_teleop')

    arm = youbot_arm_control(debug=_debug)
    print(msg)
    print(current_ratio(ratio))
    print("------------------------------------------")

    while not rospy.is_shutdown():

        key = getKey()

        if key in jointBindings.keys():
            vel_cmd = jointBindings[key]
            #vel_cmd = arm.vel_check_limit(vel_cmd)
        elif key in ratioBindings.keys():
            ratio = ratio + ratioBindings[key]

            if ratio <=0:
                ratio = 0.1
            elif ratio >=0.5:
                ratio = 0.5

            print(current_ratio(ratio))
        elif key == 'y':
            arm.getJointState(show=True)
        elif key == 'h':
            arm.getEefPosition(show=True)
        else:
            vel_cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
            if key == '\x03':
                break

        arm.arm_vel_control(vel_cmd, ratio)
