'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
import numpy as np

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        
        # additions
        self.starttime = 0
        self.splineParams = {}
        self.keyFramePositions = {}
        
    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        # get time since start
        if self.starttime == 0:
            self.starttime = perception.time
        time = perception.time - self.starttime

        # init key frame pos dictionary
        if len(self.keyFramePositions.keys()) == 0:
            for joint in self.joint_names:
                self.keyFramePositions[joint] = -1
        

        # for all joints
        for i in xrange(len(keyframes[0])):
            joint = keyframes[0][i]
            
            if not (joint in self.joint_names):
                continue
            
            # get current keyframe pos
            j = 0
            while j < len(keyframes[1][i])-1 and time > keyframes[1][i][j]:
                j += 1
            
            
            if j > 0:
                timeOffset = keyframes[1][i][j-1]
            else:
                timeOffset = 0.
                
            t = time-timeOffset
            tf = keyframes[1][i][j]-timeOffset
            
            
            # if new keyframe position is reached, compute new parameters
            if j > self.keyFramePositions[joint]:
                self.keyFramePositions[joint] = j
                if j > 0:
                    print joint, keyframes[2][i][j-1][0] - perception.joint[joint], keyframes[2][i][j-1][0], perception.joint[joint]
                
                u0 = perception.joint[joint]
                uf = keyframes[2][i][j][0]
                
                # first keyframe pos
                if j == 0:
                    v0 = 0.
                    
                    if len(keyframes[2][i]) == 1:
                        vf = viaSpeedHeuristic(u0, u0, u0, 1, 1)
                    elif len(keyframes[2][i]) == 2:
                        vf = viaSpeedHeuristic(u0, uf, uf, tf, tf)
                    else:
                        vf = viaSpeedHeuristic(u0, uf, keyframes[2][i][j+1][0], tf, keyframes[1][i][j+1])
                    
                
                # last keyframe pos
                elif j == len(keyframes[1][i])-1:
                    v0 = self.speed[joint]
                    vf = viaSpeedHeuristic(u0, uf, uf, tf, tf)
    
                
                # keyframe positions in between
                else:
                    v0 = self.speed[joint]
                    vf = viaSpeedHeuristic(u0, uf, keyframes[2][i][j+1][0], tf, keyframes[1][i][j+1])
                
                
                a0 = u0
                a1 = v0
                a2 = 3./tf**2 * (uf-u0) - 2./tf*v0 - 1./tf*vf
                a3 = -2./tf**3 * (uf-u0) + 1./tf**2 * (vf+v0)
                
                self.splineParams[joint] = [a0, a1, a2, a3]
                
                
            # compute new target
            if t <= tf:
                target_joints[joint] = self.splineParams[joint][0] + self.splineParams[joint][1]*t + self.splineParams[joint][2]*t**2 + self.splineParams[joint][3]*t**3
            
            #if i == 2:
            #    print j, len(keyframes[2][i]), target_joints[joint], " - ", perception.joint[joint], " - ", keyframes[2][i][j][0], " - ", self.splineParams[joint][0] + self.splineParams[joint][1]*tf + self.splineParams[joint][2]*tf**2 + self.splineParams[joint][3]*tf**3
            
        return target_joints


# get average of current and next average speeds
def viaSpeedHeuristic(lastPos, targetPos, afterPos, targetTime, afterTime):
    if targetPos == afterPos:
        return 0.
    
    return ((targetPos-lastPos)/targetTime + (afterPos-targetPos)/afterTime) / 2
    


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    #agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = leftBackToStand()
    agent.run()
