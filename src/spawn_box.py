#!/usr/bin/env python3

from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
import rospkg
import subprocess

# wait for gazebo's spawn sdf service
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
rospy.wait_for_service('/gazebo/spawn_sdf_model')
rospy.sleep(1.0)

# rospack util
rospack = rospkg.RosPack()

# create anymal box
sdf_xacro_path = rospack.get_path('eurobin_grasp_box') + '/models/anymal/anymal_box.sdf.xacro'

anymal_aruco_id = 1
anymal_mass = 10
anymal_size = (1.20, 0.72, 0.50)  # max = .70
anymal_aruco_pos = (0.2, 0.0)
anymal_sdf_path = rospack.get_path('eurobin_grasp_box') + '/models/anymal/anymal_box.sdf'
xacro_cmd = f'xacro {sdf_xacro_path} -o {anymal_sdf_path} aruco_id:={anymal_aruco_id} mass:={anymal_mass} l_x:={anymal_size[0]} l_y:={anymal_size[1]}  l_z:={anymal_size[2]} aruco_x:={anymal_aruco_pos[0]}'
subprocess.run(xacro_cmd, shell=True)

# define spawn pose and spawn anymal model
initial_pose = Pose()
initial_pose.orientation.w = 0.707
initial_pose.orientation.z = -0.707

initial_pose.position.x = 2.2
initial_pose.position.z = 0.31

res = spawn_model_client(
    model_name='anymal',
    model_xml=open(anymal_sdf_path, 'r').read(),
    robot_namespace='/',
    initial_pose=initial_pose,
    reference_frame='world'
)

print(res)


# create parcel box
sdf_xacro_path = rospack.get_path('eurobin_grasp_box') + '/models/aruco_box/model.sdf.xacro'

parcel_aruco_id = 0
parcel_mass = 0.5
parcel_size = (0.1, 0.2, 0.24)
parcel_sdf_path = rospack.get_path('eurobin_grasp_box') + '/models/aruco_box/parcel.sdf'
xacro_cmd = f'xacro {sdf_xacro_path} -o {parcel_sdf_path} aruco_id:={parcel_aruco_id} mass:={parcel_mass} l_x:={parcel_size[0]} l_y:={parcel_size[1]}  l_z:={parcel_size[2]}'
subprocess.run(xacro_cmd, shell=True)


# define spawn pose and spawn aruco box
initial_pose.orientation.w = 1
initial_pose.orientation.z = 0
initial_pose.orientation.y = 0
initial_pose.position.x = 2.2
initial_pose.position.z = 0.8

res = spawn_model_client(
    model_name='aruco_box',
    model_xml=open(rospack.get_path('eurobin_grasp_box') + '/models/aruco_box/parcel.sdf', 'r').read(),
    robot_namespace='/',
    initial_pose=initial_pose,
    reference_frame='world'
)

print(res)