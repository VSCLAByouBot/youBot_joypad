#!/usr/bin/env python
# *****************************************************
#  Keyboard teleop control for KUKA youBot base
# *****************************************************
#  Reference : 
#   > youbot_keyboard_teleop.py 
# 				from youbot_driver_ros_interface
# *****************************************************

import rospy
import youbot_joypad.keyboard_binding as kb
from youbot_joypad.youbot_control import youbot_base_control
import sys, select, termios, tty, signal

_debug = rospy.get_param("debug", False)

msg = """
Reading from the keyboard and Publishing to youBot!
------------------------------------------------------
Base Moving:

 ↖      ↑      ↗
   u    i    o
 ← j         l →
   m    ,    .
 ↙      ↓      ↘

[ / ] : increase/decrease max speeds of the base
anything else : stop

CTRL-C to quit
"""

baseBindings = kb.baseBindings
ratioBindings = {
	'[' :  0.1,
	']' : -0.1, 
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
	return "currently:\tratio =  %.1f " % (ratio)

if __name__ =="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('youbot_base_teleop')

    # debug module : keyboard_binding.py  
    # rospy.loginfo("keyboard_binding [i] : %d, %d, %d" % (baseBindings['i'][0], baseBindings['i'][1], baseBindings['i'][2]))

    base = youbot_base_control(debug=_debug)
    print(msg)
    print(current_ratio(ratio))
    print("------------------------------------------")

    while not rospy.is_shutdown():

        key = getKey()

        if key in baseBindings.keys():
            vel_cmd = baseBindings[key]
        elif key in ratioBindings.keys():
            ratio = ratio + ratioBindings[key]
            if ratio <= 0.1:
                ratio = 0.1
            print(current_ratio(ratio), '\r')
        else:
            vel_cmd = [0, 0, 0]
            if key == '\x03':
                break

        base.move_base(vel_cmd, ratio)
