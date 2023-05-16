import rospy
import tf
from robot_ros_nav.srv import *
from geometry_msgs.msg import *
from cartesian_interface.pyci_all import *

import numpy as np

rospy.init_node('go_to_box')

try:
    rospy.wait_for_service('ask_for_nav_plan')
except:
    raise RuntimeError('ask_for_nav_plan not available!')

listener = tf.TransformListener()
listener.waitForTransform('/world', 'fiducial_0', rospy.Time(), timeout=rospy.Duration(1.))
(trans, rot) = listener.lookupTransform('/world', 'fiducial_0', rospy.Time())

listener.waitForTransform('/world', "/base_link", rospy.Time(), timeout=rospy.Duration(1.))
(trans_p, rot_p) = listener.lookupTransform('/world', 'base_link', rospy.Time())

t_p = Affine3(trans_p, rot_p)
t_f = Affine3(trans, rot)

plan_srv = rospy.ServiceProxy("ask_for_nav_plan", AskNavPlan)

req = AskNavPlanRequest()

req.start.position.x = t_p.translation[0]
req.start.position.y = t_p.translation[1]
req.start.position.z = t_p.translation[2]

req.start.orientation.x = t_p.quaternion[0]
req.start.orientation.y = t_p.quaternion[1]
req.start.orientation.z = t_p.quaternion[2]
req.start.orientation.w = t_p.quaternion[3]

t_goal = t_p.copy()

diff_trans = t_f.translation - t_p.translation

diff_trans_norm = diff_trans / np.linalg.norm(diff_trans)

t_goal.translation += diff_trans - diff_trans_norm*1.0

req.goal.position.x = t_goal.translation[0]
req.goal.position.y = t_goal.translation[1]
req.goal.position.z = t_goal.translation[2]

req.goal.orientation.x = t_goal.quaternion[0]
req.goal.orientation.y = t_goal.quaternion[1]
req.goal.orientation.z = t_goal.quaternion[2]
req.goal.orientation.w = t_goal.quaternion[3]

res = plan_srv(req)

print(res)