import numpy as np
import math as m
import time
import sys
import sim

l1 = 0.475
l2 = 0.4
offset = 0.1
E = 0.2
dt = 0.01
def fkine(theta_1, theta_2, theta_3, d_4):
  return np.array([[m.cos(theta_3+theta_2+theta_1), m.sin(theta_3+theta_2+theta_1), 0, l1*m.cos(theta_1)+l2*m.cos(theta_2+theta_1)],
                   [m.sin(theta_3+theta_2+theta_1), -m.cos(theta_3+theta_2+theta_1), 0, l1*m.sin(theta_1)+l2*m.sin(theta_2+theta_1)],
                   [0, 0, -1, -d_4],
                   [0, 0, 0, 1]])

def extract(T):
  x = T[0][3]
  y = T[1][3]
  z = T[2][3]
  roll = 0
  pitch = m.pi
  yaw = m.atan2(-T[1][0], -T[0][0])
  return np.array([x, y, z, roll, pitch, yaw])
def jacobian(l1, l2, q1, q2):
  return np.array([[-l2*m.sin(q1+q2)-l1*m.sin(q1), -l2*m.sin(q1+q2), 0, 0],
                  [-l2*m.sin(q1+q2)-l1*m.sin(q1), -l2*m.cos(q1+q2), 0, 0],
                  [0, 0, 0, -1],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0],
                  [1, 1, 1, 0]])
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # enable the synchronous mode on the client:
    sim.simxSynchronous(clientID,True)

    # start the simulation:
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    err_code, axis_1_handle = sim.simxGetObjectHandle(clientID,"axis_1", sim.simx_opmode_blocking)
    err_code, axis_2_handle = sim.simxGetObjectHandle(clientID,"axis_2", sim.simx_opmode_blocking)
    err_code, axis_3_handle = sim.simxGetObjectHandle(clientID,"axis_3", sim.simx_opmode_blocking)
    err_code, axis_4_handle = sim.simxGetObjectHandle(clientID,"axis_4", sim.simx_opmode_blocking)
    joint_pos_1 = (1, 0) 
    joint_pos_2 = (1, 0)
    joint_pos_3 = (1, 0) 
    joint_pos_4 = (1, 0) 
    while joint_pos_1[0]+joint_pos_2[0]+joint_pos_3[0]+joint_pos_4[0] != 0 :
      joint_pos_1 = sim.simxGetJointPosition(clientID, axis_1_handle, sim.simx_opmode_streaming)
      joint_pos_2 = sim.simxGetJointPosition(clientID, axis_2_handle, sim.simx_opmode_streaming)
      joint_pos_3 = sim.simxGetJointPosition(clientID, axis_3_handle, sim.simx_opmode_streaming)
      joint_pos_4 = sim.simxGetJointPosition(clientID, axis_4_handle, sim.simx_opmode_streaming)
    qm = [joint_pos_1[1], joint_pos_2[1], joint_pos_3[1], joint_pos_4[1]]
    T = fkine(joint_pos_1[1], joint_pos_2[1], joint_pos_3[1], joint_pos_4[1])
    Xd = extract(T)
    Xm = np.array([0.3188, 0.425, 0.1+offset, 0, m.pi, m.pi/4])
    while np.linalg.norm(Xd-Xm)>= E:
      J = jacobian(l1, l2, joint_pos_1[1], joint_pos_2[1])
      q_dot = np.matmul(np.linalg.pinv(J), Xd-Xm)
      qm = qm + q_dot*dt
      ret_pos_1 = sim.simxSetJointPosition(clientID, axis_1_handle, qm[0], sim.simx_opmode_oneshot)
      ret_pos_2 = sim.simxSetJointPosition(clientID, axis_2_handle, qm[1], sim.simx_opmode_oneshot)
      ret_pos_3 = sim.simxSetJointPosition(clientID, axis_3_handle, qm[2], sim.simx_opmode_oneshot)
      ret_pos_4 = sim.simxSetJointPosition(clientID, axis_4_handle, qm[3], sim.simx_opmode_oneshot)
      print(ret_pos_1, ret_pos_2, ret_pos_3, ret_pos_4)
      joint_pos_1 = sim.simxGetJointPosition(clientID, axis_1_handle, sim.simx_opmode_buffer)
      joint_pos_2 = sim.simxGetJointPosition(clientID, axis_2_handle, sim.simx_opmode_buffer)
      joint_pos_3 = sim.simxGetJointPosition(clientID, axis_3_handle, sim.simx_opmode_buffer)
      joint_pos_4 = sim.simxGetJointPosition(clientID, axis_4_handle, sim.simx_opmode_buffer)
      qm = [joint_pos_1[1], joint_pos_2[1], joint_pos_3[1], joint_pos_4[1]]
      print(qm)
      T = fkine(qm[0], qm[1], qm[2], qm[3])
      Xd = extract(T)

    # stop the simulation:
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')