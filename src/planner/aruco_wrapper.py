from cartesian_interface.pyci_all import *

import rospy
import tf
import numpy as np

class ArucoWrapper:

    def __init__(self, planning_scene, model, config):
        self.listener = tf.TransformListener()

        self.ps = planning_scene
        self.model = model

        self.__frame_dict = config['aruco_wrapper']['boxes']
        
        self.__grasp_dict = config['aruco_wrapper']['grasp']

        self.__box_poses = dict()
        self.__grasp_poses = dict()

    def listen(self):
        
        for frame, prop in self.__frame_dict.items():
            size = prop['size']  # size wrt aruco frame
            center = prop['center']  # position of box center wrt aruco frame
            now = rospy.Time.now()

            while True:
                try:
                    # listen tf from aruco to robot's base_link
                    print(f'waiting for transform ( base_link -> {frame} )')
                    self.listener.waitForTransform('/base_link', frame, rospy.Time(), timeout=rospy.Duration(1.))
                    (trans, rot) = self.listener.lookupTransform('/base_link', frame, rospy.Time())
                    break
                except:
                    continue

            # save the aruco transform matrix in world frame
            w_T_base = self.model.getPose('base_link')
            w_T_aruco = w_T_base*Affine3(trans, rot)

            w_center = w_T_aruco * center
            w_T_box = w_T_aruco.copy()
            w_T_box.translation = w_center
        
            self.__box_poses[frame] = w_T_box

            # add to PlanningScene
            self.ps.addBox(frame, size, w_T_box)
            print(f'added box in pos {w_center}')
            

        for frame, props in self.__grasp_dict.items():
            base_frame = props['base_frame']
            offset = props['offset']
            w_T_box = self.getBoxPose(base_frame)

            w_T_grasp = w_T_box.copy()
            w_T_grasp.translation += w_T_box.linear @ np.array(offset)

            self.__grasp_poses[frame] = w_T_grasp

        self.ps.startGetPlanningSceneServer()

    def getBoxPoses(self):
        return self.__box_poses

    def getBoxPose(self, fiducial):
        return self.__box_poses[fiducial]
    
    def getGraspPose(self, frame):
        return self.__grasp_poses[frame]
