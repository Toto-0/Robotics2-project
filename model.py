import numpy as np
import do_mpc
from math import *


model_type = 'continuous' # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

q1 = model.set_variable(var_type='_x', var_name='q1', shape=(1,1))
q2 = model.set_variable(var_type='_x', var_name='q2', shape=(1,1))
q3 = model.set_variable(var_type='_x', var_name='q3', shape=(1,1))
# Variables can also be vectors:
dq = model.set_variable(var_type='_x', var_name='dq', shape=(3,1))
# Two states for the desired (set) motor position:
tau_1_set = model.set_variable(var_type='_u', var_name='q_m_1_set')
tau_2_set = model.set_variable(var_type='_u', var_name='q_m_2_set')
tau_3_set = model.set_variable(var_type='_u', var_name='q_m_3_set')

# Two additional states for the true motor position:
#q_1_m = model.set_variable(var_type='_x', var_name='q_1_m', shape=(1,1))
#q_2_m = model.set_variable(var_type='_x', var_name='q_2_m', shape=(1,1))

m3 = model.set_variable('parameter', 'm3')
dc3 = model.set_variable('parameter', 'dc3')
I3zz = model.set_variable('parameter', 'I3zz')

# We define the other parameters


M = np.array([[ I1zz + I2zz + I3zz + L1^2*m2 + L1^2*m3 + L2^2*m3 + dc1^2*m1 + dc2^2*m2 + dc3^2*m3 + 2*L1*dc3*m3*cos(q2 + q3) + 2*L1*L2*m3*cos(q2) + 2*L1*dc2*m2*cos(q2) + 2*L2*dc3*m3*cos(q3), m3*L2^2 + 2*m3*cos(q3)*L2*dc3 + L1*m3*cos(q2)*L2 + m2*dc2^2 + L1*m2*cos(q2)*dc2 + m3*dc3^2 + L1*m3*cos(q2 + q3)*dc3 + I2zz + I3zz, I3zz + dc3*m3*(dc3 + L1*cos(q2 + q3) + L2*cos(q3))],
              [                                             m3*L2^2 + 2*m3*cos(q3)*L2*dc3 + L1*m3*cos(q2)*L2 + m2*dc2^2 + L1*m2*cos(q2)*dc2 + m3*dc3^2 + L1*m3*cos(q2 + q3)*dc3 + I2zz + I3zz,                                                                 m3*L2^2 + 2*m3*cos(q3)*L2*dc3 + m2*dc2^2 + m3*dc3^2 + I2zz + I3zz,                   I3zz + dc3*m3*(dc3 + L2*cos(q3))],
              [                                                                                                                            I3zz + dc3*m3*(dc3 + L1*cos(q2 + q3) + L2*cos(q3)),                                                                                                  I3zz + dc3*m3*(dc3 + L2*cos(q3)),                                    m3*dc3^2 + I3zz]])

c = np.array([[- L1*dc3*m3*dq[1]^2*sin(q2 + q3) - L1*dc3*m3*dq[2]^2*sin(q2 + q3) - L1*L2*m3*dq[1]^2*sin(q2) - L1*dc2*m2*dq[1]^2*sin(q2) - L2*dc3*m3*dq[2]^2*sin(q3) - 2*L1*dc3*m3*dq[0]*dq[1]*sin(q2 + q3) - 2*L1*dc3*m3*dq[0]*dq[2]*sin(q2 + q3) - 2*L1*dc3*m3*dq[1]*dq[2]*sin(q2 + q3) - 2*L1*L2*m3*dq[0]*dq[1]*sin(q2) - 2*L1*dc2*m2*dq[0]*dq[1]*sin(q2) - 2*L2*dc3*m3*dq[0]*dq[2]*sin(q3) - 2*L2*dc3*m3*dq[1]*dq[2]*sin(q3)],
              [                                                                                                                                                                                                                                      L1*dc3*m3*dq[0]^2*sin(q2 + q3) + L1*L2*m3*dq[0]^2*sin(q2) + L1*dc2*m2*dq[0]^2*sin(q2) - L2*dc3*m3*dq[2]^2*sin(q3) - 2*L2*dc3*m3*dq[0]*dq[2]*sin(q3) - 2*L2*dc3*m3*dq[1]*dq[2]*sin(q3)],
              [                                                                                                                                                                                                                                                                                                                         dc3*m3*(L1*dq[0]^2*sin(q2 + q3) + L2*dq[0]^2*sin(q3) + L2*dq[1]^2*sin(q3) + 2*L2*dq[0]*dq[1]*sin(q3))]])
 
g = np.array([[g0*m3*(L2*cos(q1 + q2) + L1*cos(q1) + dc3*cos(q1 + q2 + q3)) + g0*m2*(dc2*cos(q1 + q2) + L1*cos(q1)) + dc1*g0*m1*cos(q1)],
              [                                                g0*m3*(L2*cos(q1 + q2) + dc3*cos(q1 + q2 + q3)) + dc2*g0*m2*cos(q1 + q2)],
              [                                                                                             dc3*g0*m3*cos(q1 + q2 + q3)]])


model.set_rhs('phi_1', dq[0])
model.set_rhs('phi_2', dq[1])
model.set_rhs('phi_3', dq[2])