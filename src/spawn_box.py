#!/usr/bin/env python3

from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
import rospkg

rospack = rospkg.RosPack()

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
rospy.wait_for_service('/gazebo/spawn_sdf_model')
rospy.sleep(1.0)

initial_pose = Pose()
initial_pose.orientation.w = 1

initial_pose.position.x = 2
initial_pose.position.z = 0.31

res = spawn_model_client(
    model_name='anymal',
    model_xml=open(rospack.get_path('eurobin_grasp_box') + '/models/anymal.sdf', 'r').read(),
    robot_namespace='/',
    initial_pose=initial_pose,
    reference_frame='world'
)

print(res)

initial_pose.position.x = 2
initial_pose.position.z = 1

res = spawn_model_client(
    model_name='aruco_box',
    model_xml=open(rospack.get_path('eurobin_grasp_box') + '/models/aruco_marker/model.sdf', 'r').read(),
    robot_namespace='/',
    initial_pose=initial_pose,
    reference_frame='world'
)

print(res)