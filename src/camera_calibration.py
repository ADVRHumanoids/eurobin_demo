import rospy, rospkg
import trimesh
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class CameraCalibration:
    def __init__(self):
        urdf_folder = rospkg.RosPack().get_path('centauro_urdf')
        self.__mesh = trimesh.load(urdf_folder + '/meshes/BallHand.stl')

        self.__xyz = None
        rospy.Subscriber('D435i_head_camera/depth/color/points', PointCloud2, self.__pc_callback)

    def __pc_callback(self, msg):
        self.__xyz = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))