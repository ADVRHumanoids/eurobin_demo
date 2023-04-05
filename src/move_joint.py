#!/usr/bin/env python3

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as copt
from cartesian_interface.pyci_all import get_xbot_config
import sys 
import rospy 
from std_srvs.srv import SetBool, Trigger

rospy.init_node('move_joint_node')

# get robot ifc
cfg = get_xbot_config(prefix='xbotcore')
robot = xbot.RobotInterface(cfg)

# enable ros ctrl and set filters
ros_control = rospy.ServiceProxy('/xbotcore/ros_control/switch', SetBool)
enable_filter = rospy.ServiceProxy('/xbotcore/enable_joint_filter', SetBool)
set_filter_profile_safe = rospy.ServiceProxy('/xbotcore/set_filter_profile_safe', Trigger)

ros_control.wait_for_service()
enable_filter.wait_for_service()
set_filter_profile_safe.wait_for_service()

enable_filter(True)
set_filter_profile_safe()
ros_control(True)

jname = sys.argv[1]
q_tgt = float(sys.argv[2])
trj_time = float(sys.argv[3])

robot.setControlMode(
    {
        jname: xbot.ControlMode.Position()
    }
)

q_ref = robot.getPositionReference()
q_idx = robot.getDofIndex(jname)
q_0 = q_ref[q_idx]

time = 0
dt = 0.01
rate = rospy.Rate(1./dt)

while time <= trj_time:
    tau = time/trj_time
    alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau
    q_ref[q_idx] = (1-alpha)*q_0 + alpha*q_tgt
    robot.setPositionReference(q_ref)
    robot.move()
    time += dt
    rate.sleep()


