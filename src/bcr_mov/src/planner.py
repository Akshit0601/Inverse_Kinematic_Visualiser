
import pinocchio
from sys import argv
from os.path import dirname, join, abspath
import numpy as np
from numpy.linalg import solve,norm
from rclpy.node import Node
from sensor_msgs.msg import JointState
import rclpy

from math import sqrt,pow
import argparse

class planner(Node):

    _iter = 0

    def __init__(self):
        super().__init__('joint_state_live')

        self.model = pinocchio.buildModelFromUrdf('urdf/arm.urdf') #inport model
        # self.model = pinocchio.buildModelFromUrdf('urdf/arm.urdf') #use this if above model doesnt works
        
        self.data = self.model.createData()   
        self.home_pose = pinocchio.neutral(self.model)  #neutral position
        
        self.threshold = 1.5               #  Computational paramaters
        
        self.max_iteratons = 5000
        self.DT = 1e-1
        self.damp = 1e-12
        pinocchio.forwardKinematics(self.model,self.data,self.home_pose)
        self.goalReached = False
        self.JointID = 4

        self.max_workspace = sqrt(2)
        self.safe_input = False
        self.joint_pub = self.create_publisher(JointState,"/joint_states",10)
        self.msg = JointState()
    
    def perform_inverse_kinematics(self,translation_input):
        
        dist = sqrt(pow(translation_input[0],2)+pow(translation_input[2],2)+pow(translation_input[1],2))

        if dist > self.max_workspace:
            return Exception("workspace out of bound") #safe condition
        
        else:
            goal_array = np.array(translation_input)
            self.goalPose = pinocchio.SE3(np.eye(3),goal_array)  #set goal pose  
            self.currentPose = self.home_pose
            while(not self.goalReached):
                pinocchio.forwardKinematics(self.model,self.data,self.home_pose)
                iMd = self.data.oMi[self.JointID].actInv(self.goalPose)
                err = pinocchio.log(iMd).vector 
                if norm(err) < self.threshold:
                    self.goalReached = True
                    break
                if self._iter >= self.max_iteratons:
                    self.goalReached = False
                    break
                J = pinocchio.computeJointJacobian(self.model,self.data,self.currentPose,self.JointID)  # in joint frame
                J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
                v = - J.T.dot(solve(J.dot(J.T) + self.damp * np.eye(6), err))
                self.currentPose = pinocchio.integrate(self.model,self.currentPose,v*self.DT)
                self.msg.name = ['joint1','joint','joint2','joint3']
                # self.msg.position = self.currentPose.flatten().tolist()
                self.msg.position = [1.57,1.57,1.57,1.57]
                self.joint_pub.publish(self.msg)

                self._iter +=1

            if self.goalReached:
                print("Goal Reached")
            else:
                print("Goal Pose not reached, consider increasing threshold")

            return self.currentPose.flatten().tolist()
            


def main():
    rclpy.init()
    plan1 = planner()
    print(plan1.perform_inverse_kinematics([0.3,0.0,0.3]))



if __name__ == '__main__':
    main() 




              