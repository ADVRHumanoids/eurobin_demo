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

model.setJointPosition(robot.eigenToMap(robot.getPositionReference()))
model.update()

rp = rospkg.RosPack()
eurobin_grasp_box = rp.get_path('eurobin_grasp_box')

ikpb = open(join(eurobin_grasp_box, 'config/reach_to_grasp_config.yaml'), 'r').read()
dt = 0.01

ci = pyci.CartesianInterface.MakeInstance(
    solver='OpenSot',
    problem=ikpb,
    model=model,
    dt=dt
)

tcp = ci.getTask('ee')

vref_local = np.array([0.0, 0, -0.03, 0, 0, 0])

rate = rospy.Rate(1./dt)

while True:
    
    T = model.getPose(tcp.getDistalLink())
    vref = np.zeros((6))

    vref[:3] = T.linear.T @ vref_local[:3]

    tcp.setVelocityReference(vref)
    
    ci.update(0, dt)
    
    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot*dt
    model.setJointPosition(q)
    model.update()

    robot.setPositionReference(model.eigenToMap(q))
    robot.move()
    rate.sleep()