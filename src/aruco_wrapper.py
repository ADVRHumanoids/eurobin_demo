import rospy
import tf
import threading

class ArucoWrapper:
    def __init__(self, planning_scene):
        self.listener = tf.TransformListener()

        self.ps = planning_scene

        self.trans = []
        self.rot = []

        self.__frame_dict = {'/fiducial/0': [0.6, 0.6, 0.4], '/fiducial/1': [0.2, 0.3, 0.1]}

    def listen(self):
        for frame, size in self.__frame_dict.items():
            try:
                (trans, rot) = self.listener.lookupTransform('/fiducial/0', '/base_link', rospy.Time(0))
                trans[2] -= size[2]/2
                self.ps.addBox(frame, trans, rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

