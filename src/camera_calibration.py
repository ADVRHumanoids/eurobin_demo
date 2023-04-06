from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from cartesian_interface.pyci_all import *

import rospy, rospkg
import trimesh
import trimesh.registration
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from struct import Struct
import numpy as np

class CameraCalibration:
    def __init__(self, urdf, srdf):
        urdf_folder = rospkg.RosPack().get_path('centauro_urdf')
        self.__mesh = trimesh.load(urdf_folder + '/meshes/BallHand.stl')
        edges = self.__mesh.edges
        scene = trimesh.Scene([edges, self.__mesh])
        scene.show()

        opt = co.ConfigOptions()
        opt.set_urdf(urdf)
        opt.set_srdf(srdf)
        opt.generate_jidmap()
        opt.set_bool_parameter('is_model_floating_base', True)
        opt.set_string_parameter('model_type', 'RBDL')
        opt.set_string_parameter('framework', 'ROS')
        self.__model = xbot.ModelInterface(opt)

        # if robot is available, connect to it
        try:
            self.__robot = xbot.RobotInterface(opt)
        except:
            raise Exception('RobotInterface not created')

        dt = 0.01
        tf = 2.
        time = 0.
        start = self.__robot.getJointPositionMap()
        goal = start.copy()
        goal['j_arm1_1'] = -0.65
        goal['j_arm1_2'] = 0.33
        goal['j_arm1_3'] = -0.04
        goal['j_arm1_4'] = -1.06
        goal['j_arm1_5'] = -0.11
        goal['j_arm1_6'] = -0.34
        goal['d435_head_joint'] = -0.42
        while time < tf:
            self.__init_robot_position(tf=tf, time=time, start=start, goal=goal)
            time += dt
            rospy.sleep(dt)

        self.__xyz = None
        self.__x_filt = None
        self.__y_filt = None
        self.__z_filt = None

        self.__filtered_cloud = []

        rospy.Subscriber('D435_head_camera/depth/color/points', PointCloud2, self.__pc_callback, queue_size=10)

    def setXFilters(self, x_filt):
        if len(x_filt) != 2:
            raise Exception('x_filter size must be equal to 2')

        self.__x_filt = x_filt

    def setYFilters(self, y_filt):
        if len(y_filt) != 2:
            raise Exception('y_filter size must be equal to 2')

        self.__y_filt = y_filt

    def setZFilters(self, z_filt):
        if len(z_filt) != 2:
            raise Exception('z_filter size must be equal to 2')

        self.__z_filt = z_filt

    def setFilters(self, x_filt, y_filt, z_filt):
        if len(x_filt) != 2:
            raise Exception('x_filter size must be equal to 2')
        if len(y_filt) != 2:
            raise Exception('y_filter size must be equal to 2')
        if len(z_filt) != 2:
            raise Exception('z_filter size must be equal to 2')

        self.__x_filt = x_filt
        self.__y_filt = y_filt
        self.__z_filt = z_filt

    def getTransformationMatrix(self, frame_id, topic_name):
        T_init = self.__model.getPose('ball1', frame_id).matrix()
        self.__mesh.apply_transform(T_init)
        T, transformed, cost = trimesh.registration.icp(self.__filtered_cloud, self.__mesh)
        print(T)
        self.__generate_pointcloud(frame_id, transformed, topic_name=topic_name)


    def __init_robot_position(self, start, goal, time, tf):
        q = dict()
        for name, start, goal in zip(start.keys(), start.values(), goal.values()):
            q[name] = start + (goal - start) * (time / tf)
        self.__robot.setPositionReference(q)
        self.__robot.move()

    def __generate_pointcloud(self, frame_id, cloud, topic_name):
        # define the fields for the PointCloud2 msg
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]

        # create Struct to stack binary values in PointCloud fields
        point_struct = Struct("<fff")
        buffer = bytearray(point_struct.size * len(cloud))

        id = 0
        for p in cloud:
            point_struct.pack_into(buffer,
                                   id * point_struct.size,
                                   p[0],
                                   p[1],
                                   p[2])
            id += 1

        # return the PointCloud2 msg using the filled buffer
        pc2_msg = PointCloud2(header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                              height=1,
                              width=len(cloud),
                              is_dense=False,
                              is_bigendian=False,
                              fields=fields,
                              point_step=point_struct.size,
                              row_step=len(buffer),
                              data=buffer)
        rospy.Publisher(topic_name, PointCloud2, queue_size=1).publish(pc2_msg)


    def __pc_callback(self, msg):
        self.__filtered_cloud.clear()
        for p in pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z')):
            if self.__x_filt is not None:
                if p[0] < self.__x_filt[0] or p[0] > self.__x_filt[1]:
                    continue
            if self.__y_filt is not None:
                if p[1] < self.__y_filt[0] or p[1] > self.__y_filt[1]:
                    continue
            if self.__z_filt is not None:
                if p[2] < self.__y_filt[0] or p[2] > self.__z_filt[1]:
                    continue
            self.__filtered_cloud.append(p)

        self.__generate_pointcloud(msg.header.frame_id, self.__filtered_cloud, topic_name='filtered_cloud')
        print('cloud filtered')
        self.getTransformationMatrix(msg.header.frame_id, topic_name='aligned_cloud')
        print('cloud_aligned')

if __name__ == '__main__':
    rospy.init_node('calibration_node')

    urdf = rospy.get_param('xbotcore/robot_description')
    srdf = rospy.get_param('xbotcore/robot_description_semantic')

    camera = CameraCalibration(urdf, srdf)
    camera.setFilters([-0.2, 0.2], [-0.2, 0.2], [0., 1.2])

    rospy.spin()