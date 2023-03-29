#!/usr/bin/env python3

from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import PoseStamped
import rospy 

rospy.init_node('fiducial_marker_publisher')

pubs = dict()

def on_fid_tf_array_recv(msg: FiducialTransformArray):
    hdr = msg.header
    
    for ft in msg.transforms:
        ft : FiducialTransform = ft
        fid = ft.fiducial_id
        if fid not in pubs.keys():
            pubs[fid] = rospy.Publisher(f'fiducial_transforms/{fid}', PoseStamped, queue_size=1)
        
        psmsg = PoseStamped()
        psmsg.header = hdr
        psmsg.pose.position = ft.transform.translation
        psmsg.pose.orientation = ft.transform.rotation
        pubs[fid].publish(psmsg)


sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, on_fid_tf_array_recv, queue_size=1)

rospy.spin()