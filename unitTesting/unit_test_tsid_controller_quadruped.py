import numpy as np
np.set_printoptions(precision=3, suppress=True)

import pinocchio as se3

import eigenpy
eigenpy.switchToNumpyArray()

from dynamic_graph import plug
import dynamic_graph.sot.core
from dynamic_graph.sot.core.vector_constant import VectorConstant

from dynamic_graph.sot.torque_control.tsid_controller import TsidController

controller = TsidController('tsid_quadruped')
controller.loadURDF('models/quadruped/urdf/quadruped.urdf')

contact_frames = ['FL_contact', 'FR_contact', 'HL_contact', 'HR_contact']

for cf in contact_frames:
    controller.addContact(cf)
    assert(controller.hasSignal(cf + '__pos'))


urdf = 'models/quadruped/urdf/quadruped.urdf'
robot = se3.RobotWrapper.BuildFromURDF(urdf, ['models'], se3.JointModelFreeFlyer())

tolist = lambda v: np.array(v).reshape(-1).tolist()

q = se3.utils.zero(robot.nq)
dq = se3.utils.zero(robot.nv)
q[6] = 1
for i in range(4):
    q[7 + 2*i] = -0.4
    q[8 + 2*i] = 0.8

q[2] -= robot.framePlacement(q, robot.model.getFrameId('FL_contact'), True).translation[2]

sig_q = VectorConstant('q')
sig_q.sout.value = tolist(q)

sig_dq = VectorConstant('dq')
sig_dq.sout.value = tolist(dq)


# Set the task weights for the controller.
controller.w_com.value = 1.0;
controller.w_feet.value = 1.0;
controller.w_forces.value = 1e-5
# controller.w_posture.value = 1.0; # Enable if desired posture is provided.
controller.weight_contact_forces.value = [1., 1., 1e-3]

# Gains.
def set_gain(name, value, dim):
    controller.__getattr__('kp_' + name).value = dim * [value]
    controller.__getattr__('kd_' + name).value = dim * [2. * np.sqrt(value)]

set_gain('com', 1., 3)
set_gain('feet', 1., 6)
set_gain('constraints', 100., 6)


# Signals for the contacts.
for cf in contact_frames:
    controller.__getattr__(cf + '__f_min').value = 10.
    controller.__getattr__(cf + '__f_max').value = 10.
    controller.__getattr__(cf + '__contact_normal').value = [0., 0., 1]
    controller.__getattr__(cf + '__mu').value = 0.6

# Plug the q and dq to the controller.
plug(sig_q.sout, controller.q)
plug(sig_dq.sout, controller.v)

# Use the current com position as desired position and no motion.
com_initial = robot.com(q)
controller.com_ref_pos.value = tolist(com_initial)
controller.com_ref_vel.value = 3 * [0.]
controller.com_ref_acc.value = 3 * [0.]


# Init the controller
dt = 1e-3
controller.init(dt)


for iter in range(1, 1000):
  # Compute the torque at t=0.
  controller.dv_des.recompute(iter)

  tau = controller.tau_des.value
  ddq = controller.dv_des.value

  dq += dt * np.matrix(ddq).T;
  q = se3.integrate(robot.model, q, dt * dq)

print(iter)
print('  tau=', tau)
print('  ddq=', ddq)

com_final = robot.com(q)

print("--> COM drift: ", com_final - com_initial)

for cf in contact_frames:
    sig = controller.__getattr__(cf + '__des_f')
    sig.recompute(iter)
    print(cf + '__des_f', sig.value)
