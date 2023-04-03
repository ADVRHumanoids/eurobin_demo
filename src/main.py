from cartesian_interface.pyci_all import Affine3
import cartesian_interface.roscpp_utils as roscpp

import planner
import rospy
import rospkg

rospy.init_node('planner_node')
roscpp.init('planner_node', [])
urdf_path = rospkg.RosPack().get_path('centauro_urdf') + '/urdf/centauro.urdf'
urdf = open(urdf_path, 'r').read()

srdf_path = rospkg.RosPack().get_path('centauro_srdf') + '/srdf/centauro.srdf'
srdf = open(srdf_path, 'r').read()

rospy.set_param('robot_description', urdf)

pln = planner.Planner(urdf, srdf)
pln.generate_start_pose()

# move the end effectors
ee_goal = {
    'arm1_8': Affine3([1., 0.25, -0.01], [0.706825, 0.0005629, 0.707388, 0.0005633]),
    'arm2_8': Affine3([1., -0.25, -0.01], [0.706825, 0.0005629, 0.707388, 0.0005633])
}
pln.generate_goal_pose(list(ee_goal.keys()), list(ee_goal.values()))
trj = pln.plan()
trj_interp, time_vect = pln.interpolate(trj, 0.01, 1, 10)
pln.play_on_rviz(trj, 2, time_vect[-1])
pln.play_on_robot(trj, 5)

rospy.spin()