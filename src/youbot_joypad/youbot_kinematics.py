# *************************************************
#  kinematics of youBot arm 
#  (without consideration of base)
# *************************************************
#  [unit] position : m
#         joiint   : rad
# *************************************************

from numpy.lib.function_base import append
import rospy
import math
import numpy as np

class youbot_arm_kinematics():

    def __init__(self, debug = False, J5=2.5, has_base = False):
        self.debug = debug
        self.has_base = has_base
        if self.debug:
            print("Successfully call youbot_kinematics.")

        # DH Table (meter)
        self.dh_theta = [ 0 , math.pi/2 , 0 , math.pi/2 , 0 ]
        self.dh_d     = [ 0.115 , 0 , 0 , 0 , 0.171 ]
        self.dh_a     = [ 0.033 , 0.155 , 0.135 , 0 , 0 ]
        self.dh_alpha = [ math.pi/2 , 0 , 0 , math.pi/2, 0 ]
        self.fixed_J5 = J5
        

    def getTMat(self, joints, debug):
        
        T = []
        T_mat = np.identity(4)

        for i in reversed(range(len(self.dh_theta))):
            angle = joints[i] + self.dh_theta[i]
            cos_th = math.cos(angle)
            sin_th = math.sin(angle)
            cos_al = math.cos(self.dh_alpha[i])
            sin_al = math.sin(self.dh_alpha[i])

            _T = np.array( [ [cos_th, -sin_th*cos_al,  sin_th*sin_al, self.dh_a[i]*cos_th], \
                             [sin_th,  cos_th*cos_al, -cos_th*sin_al, self.dh_a[i]*sin_th], \
                             [0     ,  sin_al       ,  cos_al       , self.dh_d[i]], \
                             [0, 0, 0, 1]] )
            
            T_mat = np.dot(_T, T_mat)
        
        return T_mat


    def forward_kinematics(self, joints, debug=False):

        j_trans = []
        j_trans.append(-joints[0] + 169   * math.pi/180)
        j_trans.append(-joints[1] + 65    * math.pi/180)
        j_trans.append(-joints[2] - 146   * math.pi/180)
        j_trans.append(-joints[3] + 102.5 * math.pi/180)
        j_trans.append(-joints[4] + 167.5 * math.pi/180)

        T_mat = self.getTMat(j_trans, debug)

        end_pos = np.dot(T_mat, np.array([[0], [0], [0], [1]]))

        if not self.has_base:
            pos = [0.0, 0.0, 0.0]
            pos[0] = end_pos[0][0]
            pos[1] = end_pos[1][0]
            pos[2] = end_pos[2][0]
        else:
            pass
        
        return pos


    def inverse_kinematics(self, pos, debug = False):
        x = -pos[0]; y = -pos[1]; z = pos[2]

        if debug:
            print('target position = ', pos)

        # J1 ---------------------------
        jj1 = math.atan2(y, x)
        if jj1 <= 0:
            jj1 = -jj1
        else:
            jj1 = 2 * math.pi - jj1

        # J3 ---------------------------
        aa = math.sqrt(x*x + y*y) - self.dh_a[0]     # xp
        bb = z - self.dh_d[0];                       # zp
        # cc = beta
        if bb > 0.145 :
            cc = math.atan2(bb, aa)
        elif bb <= 0.145 and bb > 0:
            if aa <= -0.3:
                cc = -0.35
            else:
                cc = -0.5
        else:
            cc = -0.87

        aa = aa - self.dh_d[4] * math.cos(cc)		# xpp
        bb = bb - self.dh_d[4] * math.sin(cc)		# zpp
        J3_cos = -(bb*bb + aa*aa - self.dh_a[1]*self.dh_a[1] - self.dh_a[2]*self.dh_a[2])/(2 * self.dh_a[1] * self.dh_a[2])
        
        if (1 - J3_cos*J3_cos) <= 0:
            print("J3_sin is not a real number, NAN.")
            return [0, 0, 0, 0, 0], False
        
        jj3 = math.atan2( math.sqrt(1 - J3_cos*J3_cos) , J3_cos)

        # J2 ----------------------------
        dd = self.dh_a[1] - self.dh_a[2] * math.cos( jj3 )	# k1
        ee = self.dh_a[2] * math.sin ( jj3 )				# k2
        jj2 = math.atan2( bb, aa ) + math.atan2( ee, dd )

        # J4 ----------------------------
        jj4 = jj2 + jj3 - cc

        # J5 ----------------------------
        jj5 = self.fixed_J5

        # revise ------------------------
        jj3 = math.pi + jj3
        jj4 = 3/2 * math.pi - jj4
        if jj4 >= 2*math.pi:
            jj4 -= 2*math.pi

        # offset ------------------------
        jj1 -= 0.191888
        jj2 = -jj2 + math.pi/2 + 1.04883
        jj3 = - jj3 -  2.435
        if jj3 <= -2*math.pi:
            jj3 += 2*math.pi
        jj4 = -jj4 + math.pi/2 + 1.73184

        return [jj1, jj2, jj3, jj4, jj5], True