#!/usr/bin/env python3

from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbot
import rospkg
from os.path import join
import numpy as np
import rospy


rospy.init_node('reach_to_grasp_node')

cfg = get_xbot_config(prefix='xbotcore/')
robot = xbot.RobotInterface(cfg)
model = xbot.ModelInterface(cfg)
dt = 0.01

model.setJointPosition(robot.eigenToMap(robot.getPositionReference()))
model.update()

rp = rospkg.RosPack()
eurobin_grasp_box = rp.get_path('eurobin_grasp_box')

ikpb = open(join(eurobin_grasp_box, 'config/reach_to_grasp_config.yaml'), 'r').read()

ci = pyci.CartesianInterface.MakeInstance(
    solver='OpenSot',
    problem=ikpb,
    model=model,
    dt=dt
)

lee = ci.getTask('left_ee')
ree = ci.getTask('right_ee')

fest = pyest.ForceEstimation(model, dt)
lft = fest.addLink(lee.getDistalLink(), [0, 1, 2], ['left_arm'])
rft = fest.addLink(ree.getDistalLink(), [0, 1, 2], ['right_arm'])

vgrasp = 0.05
lvref = np.array([-vgrasp, 0, 0, 0, 0, 0])
rvref = np.array([-vgrasp, 0, 0, 0, 0, 0])

rate = rospy.Rate(1./dt)

force_threshold = 10  # 1 kg
force_nsamples = 0
force_nsamples_threshold = 10

while False:

    # force estimation
    robot.sense()
    tau = robot.getJointEffort()
    model.setJointEffort([0, 0, 0, 0, 0, 0] + tau.tolist())
    model.update()
    fest.update()
    
    # send vref to grasp box
    T_lee = model.getPose(lee.getDistalLink(), lee.getBaseLink())
    T_ree = model.getPose(ree.getDistalLink(), ree.getBaseLink())
    
    vref = np.zeros((6))
    vref[:3] = T_lee.linear @ lvref[:3]
    lee.setVelocityReference(vref)

    vref = np.zeros((6))
    vref[:3] = T_ree.linear @ rvref[:3]
    ree.setVelocityReference(vref)
    
    ci.update(0, dt)

    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot*dt
    model.setJointPosition(q)
    model.update()

    robot.setPositionReference(model.eigenToMap(q))
    robot.move()

    # exit on force condition
    lforce = lft.getWrench()[0]
    rforce = rft.getWrench()[0]

    print(f'lforce = {lforce}, rforce = {rforce}, n = {force_nsamples}')

    if abs(lforce) > force_threshold and abs(rforce) > force_threshold:
        force_nsamples += 1
    else:
        force_nsamples = 0

    if force_nsamples == force_nsamples_threshold:
        print('SUCCESS')
        break

    rate.sleep()


# hand the box to the drone
print('hand over')
lee.setBaseLink(ree.getDistalLink())
ree.setBaseLink('world')
ree.setWeight(np.diag([0, 1, 0, 1, 1, 1]))
lee.setLambda(0.1)
ree.setLambda(0.1)

T_ree = model.getPose(ree.getDistalLink(), ree.getBaseLink())

print('before', T_ree)

delta_T = Affine3(rot=[0, -1, 0, 1])
T_ree.linear = delta_T.linear @ T_ree.linear
# T_ree.translation[2] += 0.20

ree.setPoseTarget(T_ree, 5.0)

print('after', T_ree)


time = 0

while True:

    ci.update(time, dt)

    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot*dt
    model.setJointPosition(q)
    model.update()

    print(model.eigenToMap(q))

    robot.setPositionReference(model.eigenToMap(q))
    robot.move()

    time += dt
    rate.sleep()
