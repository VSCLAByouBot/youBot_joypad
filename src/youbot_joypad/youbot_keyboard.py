# ***************************************************
#   Keyboard teleop control for KKA youBot
# ***************************************************

import rospy
import sys, select, termios, tty, signal
from .keyboard_binding import keyboard_msg, baseBindings, ratioBindings, armBindings, stopBindings
from .youbot_control import youbot_base_control, youbot_arm_control
from .youbot_kinematics import youbot_arm_kinematics

class TimeoutException(Exception): 
    pass 

class YouBotKeyboardTeleop:

    msg = keyboard_msg
    arm_ratio = 0.1
    base_ratio = 0.1

    def __init__(self, _debug_arm = False, _debug_base = False):
        
        self.__settings = termios.tcgetattr(sys.stdin)

        self.baseBindings = baseBindings
        self.ratioBindings = ratioBindings
        self.armBindings = armBindings
        self.stopCommand = stopBindings

        self._debug_base = _debug_base
        self._debug_arm = _debug_arm
        self.base = youbot_base_control(debug=self._debug_base)
        self.arm = youbot_arm_control(debug=self._debug_arm)

        self.yk = youbot_arm_kinematics(debug=_debug_arm)

        self.startloop()

    
    def current_ratio(self, arm_ratio, base_ratio):
        return "currently:\tarm ratio =  %.2f\tbase ratio = %.2f \r\n" % (arm_ratio, base_ratio)

    def getKey(self):
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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__settings)
        return key

    def startloop(self):

        print(self.msg)
        print(self.current_ratio(self.arm_ratio, self.base_ratio))
        print("\n")

        while not rospy.is_shutdown():

            key = self.getKey()

            # Base
            if key in self.baseBindings.keys():
                cmd = self.baseBindings[key]

                if self._debug_base:
                    rospy.loginfo("[DEBUG] Send command to base: [ x=%.2f, y=%.2f, th=%.2f ] in ratio = %.2f\n" \
                                     % (cmd[0], cmd[1], cmd[2], self.base_ratio))
                else:
                    self.base.move_base(cmd, self.base_ratio)

            # EEF Position
            elif key in self.armBindings.keys():
                cmd = self.arm_vel_gen(self.armBindings[key])

                if self._debug_base:
                    rospy.loginfo("[DEBUG] Send command to arm: [ J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f ] in ratio = %f\n" \
                                     % (cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], self.arm_ratio))
                else:
                    self.arm.arm_vel_control(cmd, self.arm_ratio)

            # Speed Ratio
            elif key in self.ratioBindings.keys():
                self.base_ratio = self.base_ratio + self.ratioBindings[key][0]
                self.arm_ratio = self.arm_ratio + self.ratioBindings[key][1]
                print(self.current_ratio(self.arm_ratio, self.base_ratio))

            # Show States
            elif key == 'g':    # joint state
                self.arm.getJointState(show=True)

            elif key == 'h':    # EEF position
                self.arm.getEefPosition(show=True)

            # Stop or Quit
            else:
                if self._debug_base or self._debug_arm:
                    #rospy.loginfo("[DEBUG] Stop.")
                    pass
                else:
                    self.base.move_base(self.stopCommand["base"], 0)
                    self.arm.arm_vel_control(self.stopCommand["joint"], 0)

                if key == '\x03':
                    break
    
    def arm_vel_gen(self, delta_xyz):

        old_xyz = self.arm.getEefPosition(show=False)
        new_xyz = []
        for i in range(3):
            new_xyz.append( delta_xyz[i] + old_xyz[i] )

        if self._debug_arm:
            rospy.loginfo("[DEBUG | generate arm velocity command] New target position : [ x = %.2f, y = %.2f, z = %.2f]\n" \
                            % ( new_xyz[0], new_xyz[1], new_xyz[2] ))
        
        new_joint = self.yk(new_xyz)
        old_joint = self.arm.getJointState(show=False)
        delta_joint = []
        for j in range(5):
            delta_joint.append( new_joint[j] - old_joint[j] )
 
        # change in one second, return rps = delta_joint / 1 sec.
        return delta_joint
            
