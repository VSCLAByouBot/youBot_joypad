# *****************************************************
#  Class for basic functions of youbot
# *****************************************************
#  >> current joint value, eef position
#  >> velocity control, position control of arm
#  >> velocity control of base
# *****************************************************


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointVelocities, JointValue, JointPositions
from .youbot_kinematics import youbot_arm_kinematics as YK

class youbot_arm_control:

    def __init__(self, debug=False):

        self.debug = debug

        self.max_limit = [5.8401, 2.61795, -0.01571, 3.4291, 5.64155]
        self.min_limit = [0.010070, 0.010070, -5.0265, 0.022124, 0.11062]

        if self.debug:
            print("It's Debug Mode.")
        
        self.yk = YK()
        self.sub = rospy.Subscriber("/joint_states", JointState, self.__jointCallback)
        self.arm_joint_name = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.current_joint = [ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]
        self.current_pos = [ 0.0 , 0.0 , 0.0 ]

    # ------------------------------------------------------
    # ---------------- Read from Encoder -------------------
    # ------------------------------------------------------

    def __jointCallback(self, msg):
        for i in range(len(msg.position)):
            for j in range(len(self.arm_joint_name)):
                if msg.name[i] == self.arm_joint_name[j]:
                    self.current_joint[j] = msg.position[i]

    def getJointState(self, show = False):
        if show:
            self.__showJointState(self.current_joint)
        return self.current_joint

    def __showJointState(self, joint_set):
        print("===== Current Joint State =====")
        for i in range(len(self.arm_joint_name)):
            print("> %s = %s" % (self.arm_joint_name[i], joint_set[i]))

    def getEefPosition(self, show = False):
        self.current_pos = self.yk.forward_kinematic(self.current_joint)
        if show:
            self.__showEefPosition(self.current_pos)
        return self.current_pos

    def __showEefPosition(self, pos_set):
        print("===== Current EEF Position =====")
        for i in range(len(self.arm_joint_name)):
            print("> %s = %s" % (self.arm_joint_name[i], pos_set[i]))

    def vel_check_limit(self, cmd):
        joint_num = [i for i, e in enumerate(cmd) if e != 0]

        if self.debug:
            print('Rotate joint #', joint_num[0]+1)

        if self.current_joint[joint_num[0]] > self.max_limit[joint_num[0]]:
            print('Arrived the maximum limitation of joint #', joint_num[0]+1, ' : ', self.max_limit[joint_num[0]], ' (rad).')
            return [0.0, 0.0, 0.0, 0.0, 0.0]
        elif self.current_joint[joint_num[0]] > self.min_limit[joint_num[0]]:
            print('Arrived the minimum limitation of joint #', joint_num[0]+1, ' : ', self.min_limit[joint_num[0]], ' (rad).')
            return [0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            return cmd

    def pos_check_limit(self, cmd):

        over_range = []
        down_range = []
        send = True
        for i, c in enumerate(cmd):
            if c > self.max_limit[i]:
                over_range.append(i)
                send = False
            elif c < self.min_limit[i]:
                down_range.append(i)
                send = False   
        for o in over_range:
            print('Arrived the maximum limitation of joint #', o+1, ' : ', self.max_limit[o], ' (rad).')
        for d in down_range:
            print('Arrived the minimum limitation of joint #', d+1, ' : ', self.min_limit[d], ' (rad).')
        return send

    def pos2vel(self, pos_off):
        # generate the vel. cmd. for eef position move control
        old_pos = self.getEefPosition()
        new_pos = old_pos + pos_off
        new_joint = self.yk.inverse_kinematics(new_pos)
        old_joint = self.getJointState()
        joint_off = new_joint - old_joint

        # vel (rad/s) = joint_offset (rad.) / 1 (sec.)
        return joint_off
    
    def pos2pos(self, pos_off):
        # generate the pos. cmd. for eef position move control
        old_pos = self.getEefPosition()
        new_pos = old_pos + pos_off

        return new_pos

    # ------------------------------------------------------
    # --------------- Send Control Command -----------------
    # ------------------------------------------------------

    def arm_vel_control(self, joint_vel, ratio):

        pub = rospy.Publisher('arm_1/arm_controller/velocity_command', JointVelocities, queue_size=1)

        arm_vel = JointVelocities()
        for i in range(len(self.arm_joint_name)):
            vel = JointValue()
            vel.timeStamp = rospy.Time.now()
            vel.joint_uri = self.arm_joint_name[i]
            vel.unit = "s^-1 rad"    # radian_per_second
            vel.value = joint_vel[i]*ratio
            arm_vel.velocities.append(vel)

        if self.debug:
            rospy.loginfo("[DEBUG] Send vel. cmd. to arm : [ %.3f, %.3f, %.3f, %.3f, %.3f ]\r" % \
                                (arm_vel.velocities[0].value, arm_vel.velocities[1].value, arm_vel.velocities[2].value, \
                                    arm_vel.velocities[3].value, arm_vel.velocities[4].value) )
        else:
            pub.publish(arm_vel)


    def arm_pos_control(self, joint_pos):

        pub = rospy.Publisher('arm_1/arm_controller/position_command', JointVelocities, queue_size=1)

        arm_pos = JointPositions()
        for i in range(len(self.arm_joint_name)):
            pos = JointValue()
            pos.joint_uri = self.arm_joint_name[i]
            pos.unit = "rad"
            pos.value = joint_pos[i]
            arm_pos.positions.append(pos)

        if self.debug:
            rospy.loginfo("[DEBUG] Send pos. cmd. to arm : [ %.3f, %.3f, %.3f, %.3f, %.3f ]\r" % \
                                (arm_pos.positions[0].value, arm_pos.positions[1].value, arm_pos.positions[2].value, \
                                    arm_pos.positions[3].value, arm_pos.positions[4].value) )
        else:
            pub.publish(arm_pos)

        


class youbot_base_control:

    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            print("It's Debug Mode.")

    def move_base(self, vel, ratio):

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        twist = Twist()
        twist.linear.x = vel[0] * ratio
        twist.linear.y = vel[1] * ratio
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.x = 0
        twist.angular.z = vel[2] * ratio

        if self.debug:
            print("[DEBUG]Send base velocity command : [ x = %f, y = %f, th = %f ]\r" % (vel[0], vel[1], vel[2]))

        else:
            pub.publish(twist)