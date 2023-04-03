from cartesian_interface.pyci_all import *

import rospy
import tf

class ArucoWrapper:
    def __init__(self, planning_scene):
        self.listener = tf.TransformListener()

        self.ps = planning_scene

        self.trans = []
        self.rot = []

        self.__frame_dict = {'/fiducial_0': [0.3, 0.2, 0.1], '/fiducial_1': [0.6, 0.6, 0.4]}
        self.__aruco_poses = dict()

    def listen(self):
        for frame, size in self.__frame_dict.items():
            try:
                # listen tf from aruco to robot's base_link
                self.listener.waitForTransform('/base_link', frame, rospy.Time(0), timeout=rospy.Duration(5.))
                (trans, rot) = self.listener.lookupTransform('/base_link', frame, rospy.Time(0))

                # save the aruco transform matrix in world frame
                T = Affine3(trans, rot)
                self.__aruco_poses[frame] = T

                # center the box in aruco frame
                T_inv = T.inverse()
                T_inv.translation[2] += size[2]/2

                # add to PlanningScene
                self.ps.addBox(frame, size, T_inv.inverse())
                print(f'added box in pos {trans}')
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        self.ps.startGetPlanningSceneServer()

    def getArucoPoses(self):
        return self.__aruco_poses

    def getArucoPose(self, fiducial):
        return self.__aruco_poses[fiducial]
