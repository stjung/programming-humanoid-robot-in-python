'''In this exercise you need to implemente inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinemtatics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h25/joints_h25.html
       http://doc.aldebaran.com/2-1/family/nao_h25/links_h25.html
    2. use the results of inverse kinemtatics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinemtatics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from math import atan2, sqrt
from sympy import *
import sys
from numpy import matrix

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        
        target = from_trans(transform)
        
        init_printing()
        symParams = {}
        yawAngles = []
        pitchAngles = []
        rollAngles = []
        for joint in self.chains[effector_name]:
            a = symbols('a_' + joint)
            
            if joint.endswith("Yaw"):
                yawAngles.append(a)
            elif joint.endswith("Pitch"):
                pitchAngles.append(a)
            elif joint.endswith("Roll"):
                rollAngles.append(a)
            
            symParams[joint] = [a]
            symParams[joint] += [symbols('l_' + joint + '_%d' % i) for i in range(len(self.tParam[joint]))]
            
        
        joint_angles = []
        # YOUR CODE HERE
        self.forward_kinematics(self.perception.joint, symbolic=symParams)
        
        T = self.transforms[self.chains[effector_name][-1]]
        print latex(matrix(T))
        #sys.setrecursionlimit(10050)
        #simplify(T)
        eq1 = sum(yawAngles)
        eq2 = sum(pitchAngles)
        eq3 = sum(rollAngles)
        eq = T * transform.inv() - T0
        eq = [eq1-target[3], eq2-target[4], eq3-target[5]]
        
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.inverse_kinematics(effector_name, transform)
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

def from_trans(m):
    # decomposition as in http://nghiaho.com/?page_id=846    
    return [m[0, -1], m[1, -1], m[2, -1], atan2(m[2, 1], m[2, 2]), atan2(-m[2, 0], sqrt(m[2, 1]**2 +  m[2, 2]**2)), atan2(m[1, 0], m[0, 0])]

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
   # T[-1, 1] = 0.05
   # T[-1, 2] = 0.26
    T[1, -1] = 0.05
    T[2, -1] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
