import torch
import numpy as np
import swift
import time
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import os
import rospkg
from cycler import cycler

import sys

sys.path.append(os.environ["HOME"]+"/Robotics2-project/devel/lib/python3/dist-packages/ocs2_3r_nom")
sys.path.append(os.environ["HOME"]+"/Robotics2-project/devel/lib/python3/dist-packages/ocs2_3r_up")
sys.path.append(os.environ["HOME"]+"/Robotics2-project/devel/lib/python3/dist-packages/ocs2_3r_below")

from ocs2_3r_nom.BallbotPyBindings import mpc_interface, scalar_array, vector_array, TargetTrajectories
from ocs2_3r_up.BallbotPyBindings_up import mpc_interface_up
from ocs2_3r_below.BallbotPyBindings import mpc_interface_down

from PolicyNet import ExpertMixturePolicy as PolicyNet

packageDir = rospkg.RosPack().get_path('ocs2_3r_nom')
taskFile = os.path.join(packageDir, 'config/mpc/task.info')
libFolder = os.path.join(packageDir, 'auto_generated')

packageDir_up = rospkg.RosPack().get_path('ocs2_3r_up')
taskFile_up = os.path.join(packageDir_up, 'config/mpc/task.info')
libFolder_up = os.path.join(packageDir_up, 'auto_generated')

packageDir_down = rospkg.RosPack().get_path('ocs2_3r_below')
taskFile_down = os.path.join(packageDir_down, 'config/mpc/task.info')
libFolder_down = os.path.join(packageDir_down, 'auto_generated')

print("Instantiating MPC interface")
mpc = mpc_interface(taskFile, libFolder)
mpc_up = mpc_interface_up(taskFile_up, libFolder_up)
mpc_down = mpc_interface_down(taskFile_down, libFolder_down)



## Launch the simulator Swift
env = swift.Swift()
env.launch()
era = rtb.ERobot.URDF("path_to_urdf/planar_3R.urdf")

print(era)
env.add(era)
device = torch.device("cpu")
#device = torch.device("cuda:0") # Uncomment this to run on GPU



def plot(save_path, t_end=10.0):
    policy = torch.load(save_path)

    dt = 1./100.
    tx0 = np.zeros((mpc.getStateDim() + 1, 1))
    u0 = np.zeros((mpc.getInputDim(), 1))

    # Assign the initial configuration
    tx0[1, 0] = -np.pi
    tx0[2, 0] = 0.0
    tx0[3, 0] = -0.

    tx0[4, 0] = 0
    tx0[5, 0] = 0.
    tx0[6, 0] = 0.

    tx_history = np.zeros((int(t_end/dt)+1, mpc.getStateDim() + 1))
    u_history = np.zeros((int(t_end/dt)+1, mpc.getInputDim()))

    tx = tx0
    u_np = u0
    average_constraint_violation = 0
    steps = int(t_end/dt)
    era.q = tx0[1:4]

    for it in range(steps):
        tx_history[it, :] = np.transpose(tx)
        tx_torch = torch.tensor(np.transpose(tx), dtype=torch.float, requires_grad=False, device=device)
        tx_torch[0][0] = 0.0 #optionally run it in MPC style

        p, u_pred = policy(tx_torch)
        if len(p) > 1:
            u = torch.mm(p.t(), u_pred)
        else:
            u = u_pred[0]

        u_np = u.t().cpu().detach().numpy().astype('float64')
        u_history[it, :] = np.transpose(u_np)
        dx = mpc.flowMap(tx[0], tx[1:], u_np)[:6]
        
        np.transpose(tx[1:])[0] += dx*dt
        tx[0] += dt

        era.q = tx[1:4]  
        env.step(dt)
        time.sleep(dt)

    tx_history[it+1,:] = np.transpose(tx)
    u_history[it+1, :] = np.transpose(u_np)


    default_cycler = (cycler(color=['#1f77b4', '#ff7f0e', '#2ca02c']))

    plt.figure(num = 1, figsize=(8, 8), facecolor='w')
    plt.rc('lines', linewidth=2)
    plt.rc('axes', prop_cycle=default_cycler)
    lineObjects = plt.plot(tx_history[:, 0], tx_history[:, 1:4])
    ax = plt.axes()
    ax.set_ylabel('angle position [rad]', fontsize=15)
    ax.set_xlabel('time [s]', fontsize = 15)
    ax.set_facecolor('#FFFFFF')
    plt.legend(iter(lineObjects), ('q1', 'q2', 'q3'), fontsize=15)

    plt.figure(num = 2, figsize=(8, 8))
    plt.rc('lines', linewidth=2)
    plt.rc('axes', prop_cycle=default_cycler)
    lineObjects = plt.step(tx_history[:, 0], u_history[:], lw = 2)
    ax = plt.axes()
    ax.set_xlabel('time [s]', fontsize=15)
    ax.set_ylabel('torque [Nm]', fontsize=15)
    ax.set_facecolor('#FFFFFF')
    plt.legend(iter(lineObjects), ('u1', 'u2', 'u3'), fontsize=15)




plot(save_path="/tmp/mpcPolicy_2022-12-21_030548.pt", t_end=20.0)

plt.show()
