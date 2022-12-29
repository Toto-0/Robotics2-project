import numpy as np
import torch
from tensorboardX import SummaryWriter
import rospkg
import datetime
import time
import pickle
from replay_memory_alternative import ReplayMemory
import os


import sys

sys.path.append(os.environ["HOME"]+"/Robotics2-project/devel/lib/python3/dist-packages/ocs2_3r_nom")
sys.path.append(os.environ["HOME"]+"/Robotics2-project/devel/lib/python3/dist-packages/ocs2_3r_up")
sys.path.append(os.environ["HOME"]+"/Robotics2-project/devel/lib/python3/dist-packages/ocs2_3r_below")

from ocs2_3r_nom.BallbotPyBindings import mpc_interface, scalar_array, vector_array, TargetTrajectories
from ocs2_3r_up.BallbotPyBindings_up import mpc_interface_up
from ocs2_3r_below.BallbotPyBindings_below import mpc_interface_down

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


systemHasConstraints = False
desiredState = np.zeros((mpc.getStateDim(), 1)) # change the desired state here

def getTargetTrajectories(case):
    desiredTimeTraj = scalar_array()
    desiredTimeTraj.resize(1)
    desiredTimeTraj[0] = 5.0

    desiredInputTraj = vector_array()
    desiredInputTraj.resize(1)
    if case == 0:
        desiredInputTraj[0] = np.array([77.25375044, 36.78750044, 11.03625044]).reshape(-1,1)
    elif case == 1:
        desiredInputTraj[0] = np.array([80.9325, 39.24,  12.2625]).reshape(-1,1)
    else:
        desiredInputTraj[0] = np.array([84.61125044, 41.69250044, 13.48875044]).reshape(-1,1)
    #desiredInputTraj[0] = np.zeros((mpc.getInputDim(), 1))

    desiredStateTraj = vector_array()
    desiredStateTraj.resize(1)
    desiredStateTraj[0] = desiredState

    return TargetTrajectories(desiredTimeTraj, desiredStateTraj, desiredInputTraj)


#targetTrajectories = getTargetTrajectories()
targetTrajectories_down = getTargetTrajectories(0)
targetTrajectories = getTargetTrajectories(1)
targetTrajectories_up = getTargetTrajectories(2)

mpc.reset(targetTrajectories)
mpc_up.reset(targetTrajectories_up)
mpc_down.reset(targetTrajectories_down)



dtype = torch.float
#device = torch.device("cpu")
device = torch.device("cuda:0") # Uncomment this to run on GPU



class FlowMap(torch.autograd.Function):
    @staticmethod
    def forward(ctx, t, x, u):
        """
        In the forward pass we receive a Tensor containing the input and return
        a Tensor containing the output. ctx is a context object that can be used
        to stash information for backward computation. You can cache arbitrary
        objects for use in the backward pass using the ctx.save_for_backward method.
        """
        x_cpu = x.cpu()
        u_cpu = u.cpu()
        ctx.save_for_backward(t, x_cpu, u_cpu)
        x_np = x_cpu.t().detach().numpy().astype('float64')
        u_np = u_cpu.t().detach().numpy().astype('float64')
        xDot = torch.tensor(mpc.flowMap(t, x_np, u_np), device=device, dtype=dtype)
        return xDot

    @staticmethod
    def backward(ctx, grad_output):
        """
        In the backward pass we receive a Tensor containing the gradient of the loss
        with respect to the output, and we need to compute the gradient of the loss
        with respect to the input.
        """
        grad_t = grad_x = grad_u = None
        t, x, u = ctx.saved_tensors
        x_np = x.t().detach().numpy().astype('float64')
        u_np = u.t().detach().numpy().astype('float64')

        if ctx.needs_input_grad[0]:
            raise NotImplementedError("Derivative of dynamics w.r.t. time not available")
        if ctx.needs_input_grad[1]:
            dfdx = torch.tensor(mpc.flowMapLinearApproximation(t, x_np, u_np).dfdx, device=device, dtype=dtype)
            grad_x = torch.matmul(grad_output, dfdx).reshape((-1, x_np.size))
        if ctx.needs_input_grad[2]:
            dfdu = torch.tensor(mpc.flowMapLinearApproximation(t, x_np, u_np).dfdu, device=device, dtype=dtype)
            grad_u = torch.matmul(grad_output, dfdu).reshape((-1, u_np.size))
        return grad_t, grad_x, grad_u


class IntermediateCost(torch.autograd.Function):
    @staticmethod
    def forward(ctx, t, x, u):
        """
        In the forward pass we receive a Tensor containing the input and return
        a Tensor containing the output. ctx is a context object that can be used
        to stash information for backward computation. You can cache arbitrary
        objects for use in the backward pass using the ctx.save_for_backward method.
        """
        x_cpu = x.cpu()
        u_cpu = u.cpu()
        ctx.save_for_backward(t, x_cpu, u_cpu)
        x_np = x_cpu.t().detach().numpy().astype('float64')
        u_np = u_cpu.t().detach().numpy().astype('float64')
        L = torch.tensor(mpc.cost(t, x_np, u_np), device=device, dtype=dtype)
        return L

    @staticmethod
    def backward(ctx, grad_output):
        """
        In the backward pass we receive a Tensor containing the gradient of the loss
        with respect to the output, and we need to compute the gradient of the loss
        with respect to the input.
        """
        grad_t = grad_x = grad_u = None
        t, x, u = ctx.saved_tensors
        x_np = x.t().detach().numpy().astype('float64')
        u_np = u.t().detach().numpy().astype('float64')

        if ctx.needs_input_grad[0]:
            raise NotImplementedError("Derivative of RunningCost w.r.t. time not available")
        if ctx.needs_input_grad[1]:
            dLdx = torch.tensor(np.array([[mpc.costQuadraticApproximation(t, x_np, u_np).dfdx]]), device=device, dtype=dtype)
            grad_x = grad_output * dLdx
        if ctx.needs_input_grad[2]:
            dLdu = torch.tensor(np.array([[mpc.costQuadraticApproximation(t, x_np, u_np).dfdu]]), device=device, dtype=dtype)
            grad_u = grad_output * dLdu
        return grad_t, grad_x, grad_u


class StateInputConstraint(torch.autograd.Function):
    @staticmethod
    def forward(ctx, t, x, u):
        """
        In the forward pass we receive a Tensor containing the input and return
        a Tensor containing the output. ctx is a context object that can be used
        to stash information for backward computation. You can cache arbitrary
        objects for use in the backward pass using the ctx.save_for_backward method.
        """
        x_cpu = x.cpu()
        u_cpu = u.cpu()
        ctx.save_for_backward(t, x_cpu, u_cpu)
        x_np = x_cpu.t().detach().numpy().astype('float64')
        u_np = u_cpu.t().detach().numpy().astype('float64')
        g1 = torch.tensor(mpc.stateInputEqualityConstraint(t, x_np, u_np), device=device, dtype=dtype)
        return g1

    @staticmethod
    def backward(ctx, grad_output):
        """
        In the backward pass we receive a Tensor containing the gradient of the loss
        with respect to the output, and we need to compute the gradient of the loss
        with respect to the input.
        """
        grad_t = grad_x = grad_u = None
        t, x, u = ctx.saved_tensors
        x_np = x.t().detach().numpy().astype('float64')
        u_np = u.t().detach().numpy().astype('float64')

        if ctx.needs_input_grad[0]:
            raise NotImplementedError("Derivative of StateInputConstraint w.r.t. time not available")
        if ctx.needs_input_grad[1]:
            raise NotImplementedError("Derivative of StateInputConstraint w.r.t. state not available")
        if ctx.needs_input_grad[2]:
            dg1du = torch.tensor(mpc.stateInputEqualityConstraintLinearApproximation(t, x_np, u_np).dfdu, device=device, dtype=dtype)
            grad_u = torch.matmul(grad_output, dg1du).reshape((-1, u_np.size))
        return grad_t, grad_x, grad_u


def loss_function(tx, u_pred, dVdx, nu):
    f = FlowMap.apply(tx[0], tx[1:], u_pred)
    L = IntermediateCost.apply(tx[0], tx[1:], u_pred)
    loss = L + dVdx.dot(f)
    if systemHasConstraints:
        g1 = StateInputConstraint.apply(tx[0], tx[1:], u_pred)
        loss += g1.dot(nu)
    return loss


def num_samples_per_trajectory_point(t, max_num_points, half_value_decay_t):
    """
    Calculates number of samples drawn for each nominal state point in trajectory
    :param t: Query time along trajectory
    :param max_num_points:
    :param half_value_decay_t: time into trajectory after which number of sampled point is halfed
    :return: Number of samples to be drawn
    """
    return max_num_points * np.exp(-np.log(2) * t / half_value_decay_t)


def trajectoryCost(policy, duration, dt_control):
    cost = 0.0  # running sum
    numStartingPoints = 1
    for _ in range(numStartingPoints):
        startPos = np.zeros([mpc.getStateDim(), 1])
        tx = np.concatenate(([[0.0]], startPos))
        for it in range(int(duration / dt_control)):
            ttx_torch = torch.tensor(np.concatenate((tx[0, 0], tx[1:]), axis=None), dtype=dtype,
                                   device=device, requires_grad=False)
            p, u_pred = policy(ttx_torch)
            if len(p) > 1:
                u = torch.matmul(p, u_pred)
            else:
                u = u_pred[0]

            u_np = u.t().cpu().detach().numpy().astype('float64')
            cost += torch.tensor(mpc.cost(tx[0], tx[1:], u_np), device=device, dtype=dtype)
            if torch.isnan(cost):
                return np.nan, tx[0]
            dx = mpc.flowMap(tx[0], tx[1:], u_np)

            tx[1:] += dx.reshape(mpc.getStateDim(), 1) * dt_control
            tx[0, 0] += dt_control
    return cost, duration


class FlowMap_up(torch.autograd.Function):
    @staticmethod
    def forward(ctx, t, x, u):
        """
        In the forward pass we receive a Tensor containing the input and return
        a Tensor containing the output. ctx is a context object that can be used
        to stash information for backward computation. You can cache arbitrary
        objects for use in the backward pass using the ctx.save_for_backward method.
        """
        x_cpu = x.cpu()
        u_cpu = u.cpu()
        ctx.save_for_backward(t, x_cpu, u_cpu)
        x_np = x_cpu.t().detach().numpy().astype('float64')
        u_np = u_cpu.t().detach().numpy().astype('float64')
        xDot = torch.tensor(mpc_up.flowMap(t, x_np, u_np), device=device, dtype=dtype)
        return xDot

    @staticmethod
    def backward(ctx, grad_output):
        """
        In the backward pass we receive a Tensor containing the gradient of the loss
        with respect to the output, and we need to compute the gradient of the loss
        with respect to the input.
        """
        grad_t = grad_x = grad_u = None
        t, x, u = ctx.saved_tensors
        x_np = x.t().detach().numpy().astype('float64')
        u_np = u.t().detach().numpy().astype('float64')

        if ctx.needs_input_grad[0]:
            raise NotImplementedError("Derivative of dynamics w.r.t. time not available")
        if ctx.needs_input_grad[1]:
            dfdx = torch.tensor(mpc_up.flowMapLinearApproximation(t, x_np, u_np).dfdx, device=device, dtype=dtype)
            grad_x = torch.matmul(grad_output, dfdx).reshape((-1, x_np.size))
        if ctx.needs_input_grad[2]:
            dfdu = torch.tensor(mpc_up.flowMapLinearApproximation(t, x_np, u_np).dfdu, device=device, dtype=dtype)
            grad_u = torch.matmul(grad_output, dfdu).reshape((-1, u_np.size))
        return grad_t, grad_x, grad_u


class IntermediateCost_up(torch.autograd.Function):
    @staticmethod
    def forward(ctx, t, x, u):
        """
        In the forward pass we receive a Tensor containing the input and return
        a Tensor containing the output. ctx is a context object that can be used
        to stash information for backward computation. You can cache arbitrary
        objects for use in the backward pass using the ctx.save_for_backward method.
        """
        x_cpu = x.cpu()
        u_cpu = u.cpu()
        ctx.save_for_backward(t, x_cpu, u_cpu)
        x_np = x_cpu.t().detach().numpy().astype('float64')
        u_np = u_cpu.t().detach().numpy().astype('float64')
        L = torch.tensor(mpc_up.cost(t, x_np, u_np), device=device, dtype=dtype)
        return L

    @staticmethod
    def backward(ctx, grad_output):
        """
        In the backward pass we receive a Tensor containing the gradient of the loss
        with respect to the output, and we need to compute the gradient of the loss
        with respect to the input.
        """
        grad_t = grad_x = grad_u = None
        t, x, u = ctx.saved_tensors
        x_np = x.t().detach().numpy().astype('float64')
        u_np = u.t().detach().numpy().astype('float64')

        if ctx.needs_input_grad[0]:
            raise NotImplementedError("Derivative of RunningCost w.r.t. time not available")
        if ctx.needs_input_grad[1]:
            dLdx = torch.tensor(np.array([[mpc_up.costQuadraticApproximation(t, x_np, u_np).dfdx]]), device=device, dtype=dtype)
            grad_x = grad_output * dLdx
        if ctx.needs_input_grad[2]:
            dLdu = torch.tensor(np.array([[mpc_up.costQuadraticApproximation(t, x_np, u_np).dfdu]]), device=device, dtype=dtype)
            grad_u = grad_output * dLdu
        return grad_t, grad_x, grad_u


def loss_function_up(tx, u_pred, dVdx, nu):
    f = FlowMap_up.apply(tx[0], tx[1:], u_pred)
    L = IntermediateCost_up.apply(tx[0], tx[1:], u_pred)
    loss = L + dVdx.dot(f)
    if systemHasConstraints:
        g1 = StateInputConstraint.apply(tx[0], tx[1:], u_pred)
        loss += g1.dot(nu)
    return loss



class FlowMap_down(torch.autograd.Function):
    @staticmethod
    def forward(ctx, t, x, u):
        """
        In the forward pass we receive a Tensor containing the input and return
        a Tensor containing the output. ctx is a context object that can be used
        to stash information for backward computation. You can cache arbitrary
        objects for use in the backward pass using the ctx.save_for_backward method.
        """
        x_cpu = x.cpu()
        u_cpu = u.cpu()
        ctx.save_for_backward(t, x_cpu, u_cpu)
        x_np = x_cpu.t().detach().numpy().astype('float64')
        u_np = u_cpu.t().detach().numpy().astype('float64')
        xDot = torch.tensor(mpc_down.flowMap(t, x_np, u_np), device=device, dtype=dtype)
        return xDot

    @staticmethod
    def backward(ctx, grad_output):
        """
        In the backward pass we receive a Tensor containing the gradient of the loss
        with respect to the output, and we need to compute the gradient of the loss
        with respect to the input.
        """
        grad_t = grad_x = grad_u = None
        t, x, u = ctx.saved_tensors
        x_np = x.t().detach().numpy().astype('float64')
        u_np = u.t().detach().numpy().astype('float64')

        if ctx.needs_input_grad[0]:
            raise NotImplementedError("Derivative of dynamics w.r.t. time not available")
        if ctx.needs_input_grad[1]:
            dfdx = torch.tensor(mpc_down.flowMapLinearApproximation(t, x_np, u_np).dfdx, device=device, dtype=dtype)
            grad_x = torch.matmul(grad_output, dfdx).reshape((-1, x_np.size))
        if ctx.needs_input_grad[2]:
            dfdu = torch.tensor(mpc_down.flowMapLinearApproximation(t, x_np, u_np).dfdu, device=device, dtype=dtype)
            grad_u = torch.matmul(grad_output, dfdu).reshape((-1, u_np.size))
        return grad_t, grad_x, grad_u


class IntermediateCost_down(torch.autograd.Function):
    @staticmethod
    def forward(ctx, t, x, u):
        """
        In the forward pass we receive a Tensor containing the input and return
        a Tensor containing the output. ctx is a context object that can be used
        to stash information for backward computation. You can cache arbitrary
        objects for use in the backward pass using the ctx.save_for_backward method.
        """
        x_cpu = x.cpu()
        u_cpu = u.cpu()
        ctx.save_for_backward(t, x_cpu, u_cpu)
        x_np = x_cpu.t().detach().numpy().astype('float64')
        u_np = u_cpu.t().detach().numpy().astype('float64')
        L = torch.tensor(mpc_down.cost(t, x_np, u_np), device=device, dtype=dtype)
        return L

    @staticmethod
    def backward(ctx, grad_output):
        """
        In the backward pass we receive a Tensor containing the gradient of the loss
        with respect to the output, and we need to compute the gradient of the loss
        with respect to the input.
        """
        grad_t = grad_x = grad_u = None
        t, x, u = ctx.saved_tensors
        x_np = x.t().detach().numpy().astype('float64')
        u_np = u.t().detach().numpy().astype('float64')

        if ctx.needs_input_grad[0]:
            raise NotImplementedError("Derivative of RunningCost w.r.t. time not available")
        if ctx.needs_input_grad[1]:
            dLdx = torch.tensor(np.array([[mpc_down.costQuadraticApproximation(t, x_np, u_np).dfdx]]), device=device, dtype=dtype)
            grad_x = grad_output * dLdx
        if ctx.needs_input_grad[2]:
            dLdu = torch.tensor(np.array([[mpc_down.costQuadraticApproximation(t, x_np, u_np).dfdu]]), device=device, dtype=dtype)
            grad_u = grad_output * dLdu
        return grad_t, grad_x, grad_u



def loss_function_down(tx, u_pred, dVdx, nu):
    f = FlowMap_down.apply(tx[0], tx[1:], u_pred)
    L = IntermediateCost_down.apply(tx[0], tx[1:], u_pred)
    loss = L + dVdx.dot(f)
    if systemHasConstraints:
        g1 = StateInputConstraint.apply(tx[0], tx[1:], u_pred)
        loss += g1.dot(nu)
    return loss



writer = SummaryWriter()


load_policy = False
if load_policy:
    save_path = "/path/to/saved/policy.pt"
    policy = torch.load(save_path)
    policy.eval()
else:
    print(mpc.getStateDim(), mpc.getInputDim())
    policy = PolicyNet(mpc.getStateDim()+1, mpc.getInputDim())

policy.to(device)

print("Initial policy parameters:")
print(list(policy.named_parameters()))

learning_rate = 1e-2
optimizer = torch.optim.Adam(policy.parameters(), lr=learning_rate)


load_memory = False
if load_memory:
    with open("/path/to/memory.pkl", 'rb') as memFile:
        mem = pickle.load(memFile)
else:
    mem_capacity = 1000000
    mem = ReplayMemory(mem_capacity)

# prepare saving of MPC solution trajectory (always add first point of a slq run)
mpc_traj_len_sec = 5.0 # length of trajectories to generate with MPC
dt_control = 1.0/100. # 100 Hz control frequency
mpc_traj_t = np.linspace(0.0, mpc_traj_len_sec, int(mpc_traj_len_sec/dt_control))
last_policy_save_time = time.time()

learning_iterations = 100000

print("==============\nStarting training\n==============")
try:
    for it in range(learning_iterations):
        alpha_mix = np.clip(1.0 - 1.0 * it / learning_iterations, 0.2, 1.0)

        # run data collection (=MPC) less frequently than the policy updates
        mpc_decimation = 1 if len(mem) < 15000 else 500
        if it % mpc_decimation == 0:

            mpc.reset(targetTrajectories)
            mpc_up.reset(targetTrajectories_up)
            mpc_down.reset(targetTrajectories_down)
            x0 = np.zeros((mpc.getStateDim(), 1))
            x0[0] = np.random.uniform(-0.5, 0.5) # j1
            x0[1] = np.random.uniform(-0.5, 0.5) # j2
            x0[2] = np.random.uniform(-0.5, 0.5) # j3
            x0_up = np.zeros((mpc.getStateDim(), 1))
            x0_up[0] = x0[0]# j1
            x0_up[1] = x0[1]# j2
            x0_up[2] = x0[2]# j3
            x0_down = np.zeros((mpc.getStateDim(), 1))
            x0_down[0] = x0[0]# j1
            x0_down[1] = x0[1]# j2
            x0_down[2] = x0[2]# j3
            u0 = np.zeros(mpc.getInputDim())
            u0_up = np.zeros(mpc.getInputDim())
            u0_down = np.zeros(mpc.getInputDim())

            print("resetting MPC")
            print("proportion of MPC policy is", alpha_mix)
            print("starting from", x0.transpose())

            for mpc_time in mpc_traj_t: # mpc dummy loop
                mpc.setObservation(mpc_time, x0, u0)
                mpc_up.setObservation(mpc_time, x0_up, u0_up)
                mpc_down.setObservation(mpc_time, x0_down, u0_down)
                try:
                    mpc.advanceMpc()
                    mpc_up.advanceMpc()
                    mpc_down.advanceMpc()
                except RuntimeError:
                    print("Caught error in MPC advance!!")
                    break
                t_result = scalar_array()
                x_result = vector_array() #changed
                u_result = vector_array() #changed
                t_result_up = scalar_array()
                x_result_up = vector_array() #changed
                u_result_up = vector_array() #changed
                t_result_down = scalar_array()
                x_result_down = vector_array() #changed
                u_result_down = vector_array() #changed

                mpc.getMpcSolution(t_result, x_result, u_result)
                mpc_up.getMpcSolution(t_result_up, x_result_up, u_result_up)
                mpc_down.getMpcSolution(t_result_down, x_result_down, u_result_down)

                K = mpc.getLinearFeedbackGain(t_result[0])
                K_up = mpc_up.getLinearFeedbackGain(t_result_up[0])
                K_down = mpc_down.getLinearFeedbackGain(t_result_down[0])

                ## push of up and down
                x_up = x_result_up[0]
                x_down = x_result_down[0]

                dVdx_up = mpc_up.valueFunctionStateDerivative(t_result_up[0], x_up)
                dVdx_down = mpc_down.valueFunctionStateDerivative(t_result_down[0], x_down)

                if systemHasConstraints:
                    nu_up = mpc_up.stateInputEqualityConstraintLagrangian(t_result_up[0], x_up)
                    nu_down = mpc_down.stateInputEqualityConstraintLagrangian(t_result_down[0], x_down)
                else:
                    nu_up = None
                    nu_down = None

                mem.push(mpc_time, x_up, dVdx_up, None, nu_up, None, u_result_up[0] + K_up.dot(x_up - x_result_up[0]), 2)
                mem.push(mpc_time, x_down, dVdx_down, None, nu_down, None, u_result_down[0] + K_down.dot(x_down - x_result_down[0]), 0)


                sample_around_factor = 0.01*np.linalg.norm(x_result[0].reshape(mpc.getStateDim(),1)-desiredState)
                if sample_around_factor > 0.1: sample_around_factor = 0.1

                # sample around the initial point and push it to the replay buffer -- push of the rest
                for i in range(int(round(num_samples_per_trajectory_point(t_result[0], max_num_points=4, half_value_decay_t=1e10)))):
                    if i == 0:
                        x = x_result[0] # definitely push back the nominal point
                    else:
                        x = np.random.multivariate_normal(x_result[0], cov=np.diag(sample_around_factor * np.ones(mpc.getStateDim())))

                    dVdx = mpc.valueFunctionStateDerivative(t_result[0], x)

                    if systemHasConstraints:
                        nu = mpc.stateInputEqualityConstraintLagrangian(t_result[0], x)
                    else:
                        nu = None

                    mem.push(mpc_time, x, dVdx, None, nu, None, u_result[0] + K.dot(x - x_result[0]), 1)

                # increment state for next time step
                ttx_torch = torch.tensor(np.concatenate((t_result[0], x_result[0]), axis=None), device=device,
                                         dtype=torch.float, requires_grad=False)
                ttx_torch_up = torch.tensor(np.concatenate((t_result_up[0], x_result_up[0]), axis=None), device=device,
                                         dtype=torch.float, requires_grad=False)
                ttx_torch_down = torch.tensor(np.concatenate((t_result_down[0], x_result_down[0]), axis=None), device=device,
                                         dtype=torch.float, requires_grad=False)

                p, u_net = policy(ttx_torch)
                if len(p) > 1:
                    u_net = torch.matmul(p, u_net)
                else:
                    u_net = u_net[0]

                p_up, u_net_up = policy(ttx_torch_up)
                if len(p_up) > 1:
                    u_net_up = torch.matmul(p_up, u_net_up)
                else:
                    u_net_up = u_net_up[0]

                p_down, u_net_down = policy(ttx_torch_down)
                if len(p_down) > 1:
                    u_net_down = torch.matmul(p_down, u_net_down)
                else:
                    u_net_down = u_net_down[0]

                u_mixed = alpha_mix * u_result[0] + (1.0 - alpha_mix) * u_net.cpu().detach().numpy().astype('float64')
                dx = mpc.flowMap(t_result[0], x_result[0], u_mixed)
                x0 += dt_control * dx.reshape(mpc.getStateDim(),1)
                u0 = u_result[0]

                u_mixed_up = alpha_mix * u_result_up[0] + (1.0 - alpha_mix) * u_net_up.cpu().detach().numpy().astype('float64')
                dx_up = mpc_up.flowMap(t_result_up[0], x_result_up[0], u_mixed_up)
                x0_up += dt_control * dx_up.reshape(mpc_up.getStateDim(),1)
                u0_up = u_result_up[0]

                u_mixed_down = alpha_mix * u_result_down[0] + (1.0 - alpha_mix) * u_net_down.cpu().detach().numpy().astype('float64')
                dx_down = mpc_down.flowMap(t_result_down[0], x_result_down[0], u_mixed_down)
                x0_down += dt_control * dx_down.reshape(mpc_down.getStateDim(),1)
                u0_down = u_result_down[0]

            print("mpc ended up at", x_result[0])
            print("system ended up at", x0)


        # extract batch of samples from replay memory
        batch_size = 2**5
        samples = mem.sample(batch_size)

        writeLogThisIteration = True

        def solver_step_closure():
            loss = torch.zeros([1], dtype=dtype, device=device)  # running sum over samples
            mpc_H = torch.zeros([1], dtype=dtype, device=device)  # running sum over samples
            g1_norm = 0.0  # running sum over samples
            for sample in samples:
                tx = torch.tensor(np.concatenate((sample.t, sample.x), axis=None), dtype=dtype, device=device, requires_grad=False)
                ttx_net = torch.tensor(np.concatenate((sample.t, sample.x), axis=None), dtype=dtype, device=device, requires_grad=False)
                p, u_pred = policy(ttx_net)
                dVdx = torch.tensor(sample.dVdx, dtype=dtype, device=device, requires_grad=False)
                if systemHasConstraints:
                    nu = torch.tensor(sample.nu, dtype=dtype, device=device, requires_grad=False)
                else:
                    nu = None
                if sample.type == 0:
                    for pi, u_pred_i in zip(p, u_pred): # loop through experts
                        loss += pi * loss_function_down(tx, u_pred_i, dVdx, nu)
                    mpc_H += loss_function_down(tx, torch.tensor(sample.u0), dVdx, nu)
                elif sample.type == 1:
                    for pi, u_pred_i in zip(p, u_pred): # loop through experts
                        loss += pi * loss_function(tx, u_pred_i, dVdx, nu)
                    mpc_H += loss_function(tx, torch.tensor(sample.u0), dVdx, nu)
                else:
                    for pi, u_pred_i in zip(p, u_pred): # loop through experts
                        loss += pi * loss_function_up(tx, u_pred_i, dVdx, nu)
                    mpc_H += loss_function_up(tx, torch.tensor(sample.u0), dVdx, nu)

                if len(p) > 1:
                    u_net = torch.matmul(p, u_pred)
                else:
                    u_net = u_pred[0]

                if systemHasConstraints:
                    g1_norm += np.linalg.norm(mpc.stateInputEqualityConstraint(sample.t, sample.x, u_net.cpu().detach().numpy().astype('float64')))

            optimizer.zero_grad()
            loss.backward()

            global writeLogThisIteration
            if writeLogThisIteration:
                writer.add_scalar('loss/perSample', loss.item() / batch_size, it)
                writer.add_scalar('loss/constraintViolation', g1_norm / batch_size, it)
                writeLogThisIteration = False

            return loss


        if it % 200 == 0:
            oc_cost, survival_time = trajectoryCost(policy=policy, duration=mpc_traj_len_sec, dt_control=dt_control)
            writer.add_scalar('metric/oc_cost', oc_cost, it)
            writer.add_scalar('metric/survival_time', survival_time, it)
            print("iteration", it, "oc_cost", oc_cost)

        if time.time() - last_policy_save_time > 5.0 * 60.0:
            last_policy_save_time = time.time()
            now = datetime.datetime.now()
            save_path = "/tmp/mpcPolicy_" + now.strftime("%Y-%m-%d_%H%M%S")
            print("Iteration", it, "saving policy to", save_path + ".pt")
            torch.save(policy, save_path + ".pt")


        optimizer.step(solver_step_closure)
        for param in policy.parameters():
            if(torch.isnan(param).any()):
                print("nan in policy!")


    print("==============\nTraining completed.\n==============")
except KeyboardInterrupt:
    print("==============\nTraining interrupted after iteration", it, ".\n==============")
    pass



print("optimized policy parameters:")
print(list(policy.named_parameters()))

now = datetime.datetime.now()
save_path = "/tmp/mpcPolicy_" + now.strftime("%Y-%m-%d_%H%M%S")
print("saving policy to", save_path + ".pt")
torch.save(policy, save_path + ".pt")

# print("Saving data to", save_path+"_memory.pkl")
# with open(save_path+"_memory.pkl", 'wb') as outputFile:
#     pickle.dump(mem, outputFile)


writer.close()

print("Done. Exiting now.")
