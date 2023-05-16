#!/usr/bin/env python3

from cartesian_interface.pyci_all import *
import cartesian_interface.roscpp_utils as roscpp

from planner import planner
import rospy
import rospkg
import sys
import yaml

# initialize rospy and roscpp
rospy.init_node('planner_node')
roscpp.init('planner_node', [])

# get config object (contains urdf, srdf)
model_cfg = get_xbot_config(prefix='xbotcore/')

# get parameters
planner_cfg_path = '/home/centauro/eurobin_ws/ros_src/eurobin_grasp_box/config/planner_config.yaml'  #sys.argv[1]
planner_cfg = yaml.safe_load(open(planner_cfg_path, 'r'))

# construct planner class
pln = planner.Planner(model_cfg.get_urdf(), model_cfg.get_srdf(), planner_cfg)

# generate goal pose from detected aruco markers
pln.generate_goal_pose()

# plan
trj, error = pln.plan(timeout=10.0)

if trj is None:
    exit(1)

# failure
trj_interp, time_vect = pln.interpolate(trj, 0.01, 1, 10)

# run twice on rviz
pln.play_on_rviz(trj, 2, time_vect[-1])

# send to robot
pln.play_on_robot(trj, 5)

rospy.spin()
