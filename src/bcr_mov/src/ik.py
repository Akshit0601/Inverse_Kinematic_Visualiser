from __future__ import print_function
  
import numpy as np
from numpy.linalg import norm, solve
from sensor_msgs.msg import JointState
from rclpy.node import Node
import rclpy
import pinocchio

rclpy.init()


model = pinocchio.buildModelFromUrdf('/home/akshit/bcr_mani/src/bcr_mov/urdf/arm.urdf')
node = rclpy.create_node('pub_joint_live')
pub_joint  = node.create_publisher(JointState,"/joint_states",10)


data  = model.createData()

msg = JointState()



JOINT_ID = 3
oMdes = pinocchio.SE3(np.eye(3), np.array([0.3, 0.3, 0.3]))

q      = pinocchio.neutral(model)
eps    = 1e-4
IT_MAX = 1000
DT     = 1e-1
damp   = 1e-12

i=0
rate = node.create_rate(1.0)
while True:
    pinocchio.forwardKinematics(model,data,q)
    dMi = oMdes.actInv(data.oMi[JOINT_ID])
    err = pinocchio.log(dMi).vector
    if norm(err) < eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    J = pinocchio.computeJointJacobian(model,data,q,JOINT_ID)
    v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    q = pinocchio.integrate(model,q,v*DT)
    msg.name = ["joint1","joint","joint2","joint3"]
    msg.position = [1.57,1.57,1.57,1.57]
    pub_joint.publish(msg)


    if not i % 10:
        print('%d: error = %s' % (i, err.T))
    i += 1
    # rate.sleep()

if success:
    print("Convergence achieved!")
else:
    print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")

print('\nresult: %s' % q.flatten().tolist())
print('\nfinal error: %s' % err.T)
rclpy.spin()
rclpy.shutdown()
