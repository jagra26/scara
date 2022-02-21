import numpy as np
import math as m
import time
import sys
import sim
import roboticstoolbox as rtb

# constants

l1 = 0.475
l2 = 0.4
offset = 0.1
E = 0.02
dt = 0.05

#define SCARA

T_0_1 = rtb.robot.DHLink(a = l1, offset=0)
T_1_2 = rtb.robot.DHLink(a = l2, offset=0)
T_2_3 = rtb.robot.DHLink(alpha=m.pi, offset=0)
T_3_4 = rtb.robot.DHLink(sigma=1, qlim=[0, 0.1])
SCARA = rtb.robot.DHRobot([T_0_1, T_1_2, T_2_3, T_3_4], name="SCARA")

print(SCARA)

# extract x,y,z,roll,pitch,yaw from transformatiom matrix

def extract(T):
  x = T[0][3]
  y = T[1][3]
  z = T[2][3]
  roll = 0
  pitch = m.pi
  yaw = m.atan2(T[1][0], T[0][0])
  return np.array([x, y, z, roll, pitch, yaw])


print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # enable the synchronous mode on the client:
    sim.simxSynchronous(clientID,True)

    # start the simulation:
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # get handles

    err_code, axis_1_handle = sim.simxGetObjectHandle(clientID,"axis_1", sim.simx_opmode_blocking)
    print(err_code)
    err_code, axis_2_handle = sim.simxGetObjectHandle(clientID,"axis_2", sim.simx_opmode_blocking)
    print(err_code)
    err_code, axis_3_handle = sim.simxGetObjectHandle(clientID,"axis_3", sim.simx_opmode_blocking)
    print(err_code)
    err_code, axis_4_handle = sim.simxGetObjectHandle(clientID,"axis_4", sim.simx_opmode_blocking)
    print(err_code)
    time.sleep(1)
    # get joint positions

    joint_pos_1 = (1, 0) 
    joint_pos_2 = (1, 0)
    joint_pos_3 = (1, 0) 
    joint_pos_4 = (1, 0) 
    ret_pos_1 = 1
    ret_pos_2 = 1
    ret_pos_3 = 1
    ret_pos_4 = 1
    # avoid errors

    # set initial position

    while ret_pos_1+ret_pos_2+ret_pos_3+ret_pos_4 != 0 :
      ret_pos_1 = sim.simxSetJointPosition(clientID, axis_1_handle, -1, sim.simx_opmode_oneshot)
      ret_pos_2 = sim.simxSetJointPosition(clientID, axis_2_handle, -1, sim.simx_opmode_oneshot)
      ret_pos_3 = sim.simxSetJointPosition(clientID, axis_3_handle, 0, sim.simx_opmode_oneshot)
      ret_pos_4 = sim.simxSetJointPosition(clientID, axis_4_handle, 0, sim.simx_opmode_oneshot)
      time.sleep(dt)
    print(ret_pos_1, ret_pos_2, ret_pos_3, ret_pos_4)
    time.sleep(.5)

    # set initial position

    while joint_pos_1[0]+joint_pos_2[0]+joint_pos_3[0]+joint_pos_4[0] != 0 :
      joint_pos_1 = sim.simxGetJointPosition(clientID, axis_1_handle, sim.simx_opmode_streaming)
      joint_pos_2 = sim.simxGetJointPosition(clientID, axis_2_handle, sim.simx_opmode_streaming)
      joint_pos_3 = sim.simxGetJointPosition(clientID, axis_3_handle, sim.simx_opmode_streaming)
      joint_pos_4 = sim.simxGetJointPosition(clientID, axis_4_handle, sim.simx_opmode_streaming)
      time.sleep(dt)
    print(joint_pos_1, joint_pos_2, joint_pos_3, joint_pos_4)
    time.sleep(.5)    
    
    # resolved rate

    qm = [joint_pos_1[1], joint_pos_2[1], joint_pos_3[1], joint_pos_4[1]]
    T = SCARA.fkine([joint_pos_1[1], joint_pos_2[1], joint_pos_3[1], joint_pos_4[1]])
    Xd = extract(T.data[0])
    a = [0.3188, 0.425, 0.1-offset, 0, m.pi, m.pi/4]
    b = [-0.106, -0.6, 0.05-offset, 0, m.pi, -m.pi/2]
    Xm = np.array(a)
    J = SCARA.jacobe([l1, l2, joint_pos_1[1], joint_pos_2[1]])
    
    #Loop
    
    while np.linalg.norm(Xd-Xm)>= E:
      # print error
      print(np.linalg.norm(Xd-Xm))

      # calculations

      J = SCARA.jacobe([l1, l2, joint_pos_1[1], joint_pos_2[1]])
      q_dot = np.matmul(np.linalg.pinv(J), (Xd-Xm))
      qm = qm + q_dot*dt

      # set positions

      ret_pos_1 = sim.simxSetJointPosition(clientID, axis_1_handle, qm[0], sim.simx_opmode_streaming)
      ret_pos_2 = sim.simxSetJointPosition(clientID, axis_2_handle, qm[1], sim.simx_opmode_streaming)
      ret_pos_3 = sim.simxSetJointPosition(clientID, axis_3_handle, qm[2], sim.simx_opmode_streaming)
      ret_pos_4 = sim.simxSetJointPosition(clientID, axis_4_handle, qm[3], sim.simx_opmode_streaming)
      time.sleep(dt)

      # get positions
      
      joint_pos_1 = sim.simxGetJointPosition(clientID, axis_1_handle, sim.simx_opmode_buffer)
      joint_pos_2 = sim.simxGetJointPosition(clientID, axis_2_handle, sim.simx_opmode_buffer)
      joint_pos_3 = sim.simxGetJointPosition(clientID, axis_3_handle, sim.simx_opmode_buffer)
      joint_pos_4 = sim.simxGetJointPosition(clientID, axis_4_handle, sim.simx_opmode_buffer)
      qm = [joint_pos_1[1], joint_pos_2[1], joint_pos_3[1], joint_pos_4[1]]

      # calculations

      T = SCARA.fkine(qm)
      Xd = extract(T.data[0])
    
    # end result 

    print("Xd: ", Xd)
    print("Xm: ", Xm)
    time.sleep(5)

    # stop the simulation:
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
