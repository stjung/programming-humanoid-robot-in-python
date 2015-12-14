'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h25/joints_h25.html
       http://doc.aldebaran.com/2-1/family/nao_h25/links_h25.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for differnt joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import identity

from angle_interpolation import AngleInterpolationAgent

from math import sin,cos

from sympy import sin,cos
from numpy import matrix

class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll','LElbowYaw', 'LElbowRoll', 'LWristYaw'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll','RElbowYaw', 'RElbowRoll', 'RWristYaw']
                       }
        
        
        # around z
        self.r_yaw = lambda a, x, y, z: matrix(  [[cos(a), -sin(a), 0, x],
                                                  [sin(a),  cos(a), 0, y],
                                                  [0,       0,      1, z],
                                                  [0,       0,      0, 1]])
        
        #around x
        self.r_roll = lambda a, x, y, z: matrix( [[1, 0,      0,       x],
                                                  [0, cos(a), -sin(a), y],
                                                  [0, sin(a), cos(a),  z],
                                                  [0, 0,      0,       1]])
        
        # around y
        self.r_pitch = lambda a, x, y, z: matrix([[cos(a), 0, -sin(a), x],
                                                  [0,      1, 0,       y],
                                                  [sin(a), 0, cos(a),  z],
                                                  [0,      0, 0,       1]])
        
        
        self.tParam =          {'HeadYaw': [0,0,0], 
                                'HeadPitch': [0,0,0],
                                'LShoulderPitch': [0,0,0], 
                                'LShoulderRoll': [0,0,0],
                                'LElbowYaw': [105,15,0], 
                                'LElbowRoll': [0,0,0], 
                                'LWristYaw': [55,95,0,0],
                                'RShoulderPitch': [0,0,0], 
                                'RShoulderRoll': [0,0,0],
                                'RElbowYaw': [105,-15,0], 
                                'RElbowRoll': [0,0,0], 
                                'RWristYaw': [55,-95,0,0],
                                'LHipYawPitch': [0,0,0], 
                                'LHipRoll': [0,0,0], 
                                'LHipPitch': [0,0,0], 
                                'LKneePitch': [0,0,-100],  
                                'LAnkleRoll': [0,0,0],
                                'LAnklePitch': [0,0,-102.9],
                                'RHipYawPitch': [0,0,0], 
                                'RHipRoll': [0,0,0], 
                                'RHipPitch': [0,0,0], 
                                'RKneePitch': [0,0,-100], 
                                'RAnkleRoll': [0,0,0], 
                                'RAnklePitch': [0,0,-102.9]
                                }
        
    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle, symbolic={}):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        #T = matrix()
        # YOUR CODE HERE
        
        if not joint_name in symbolic.keys():
            params = self.tParam[joint_name]
        else:
            params = self.tParam[joint_name]
            for i in xrange(len(symbolic[joint_name][1:])):
                if params[i] != 0:
                    params[i] = symbolic[joint_name][i]
            joint_angle = symbolic[joint_name][0]
            
        if joint_name.endswith("Yaw"):
            return self.r_yaw(joint_angle, params[0], params[1], params[2])
        elif joint_name.endswith("Pitch"):
            return self.r_pitch(joint_angle, params[0], params[1], params[2])
        elif joint_name.endswith("Roll"):
            return self.r_roll(joint_angle, params[0], params[1], params[2])
        
        return None

    def forward_kinematics(self, joints, symbolic={}):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                if not joint in joints.keys():
                    print joint
                    continue
                
                angle = joints[joint]
                Tl = self.local_trans(joint, angle, symbolic)
                # YOUR CODE HERE
                
                T = T*Tl
                
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
