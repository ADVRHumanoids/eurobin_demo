#!/usr/bin/env python3

from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import rospy 

rospy.init_node('fiducial_marker_publisher')

pubs = dict()
tb = TransformBroadcaster()

def on_fid_tf_array_recv(msg: FiducialTransformArray):
    hdr = msg.header
    
    # for each detected fiducial...
    for ft in msg.transforms:
        
        ft : FiducialTransform = ft
        fid = ft.fiducial_id
        
        # add geomsg publisher if needed
        if fid not in pubs.keys():
            pubs[fid] = rospy.Publisher(f'fiducial_transforms/{fid}', PoseStamped, queue_size=1)
        
        # publish pose
        psmsg = PoseStamped()
        psmsg.header = hdr
        psmsg.pose.position = ft.transform.translation
        psmsg.pose.orientation = ft.transform.rotation
        pubs[fid].publish(psmsg)

        # publish tf
        tfmsg = TransformStamped()
        tfmsg.header = hdr 
        tfmsg.child_frame_id = f'fiducial/{fid}'
        tfmsg.transform = ft.transform

        tb.sendTransform(tfmsg)

# subscribe to detected fiducials
sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, on_fid_tf_array_recv, queue_size=1)

rospy.spin()