#!/usr/bin/env python3

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as copt
from cartesian_interface.pyci_all import *
import sys 
import rospy, rospkg
from tf import TransformListener
from std_srvs.srv import SetBool, Trigger
from cartesian_interface.srv import SetImpedance, SetImpedanceRequest
from geometry_msgs.msg import Wrench, WrenchStamped

rospy.init_node('grasp_the_box_node')
tl = TransformListener()

# enable ros ctrl and disable filters
ros_control = rospy.ServiceProxy('/xbotcore/ros_control/switch', SetBool)
enable_filter = rospy.ServiceProxy('/xbotcore/enable_joint_filter', SetBool)
set_filter_profile_safe = rospy.ServiceProxy('/xbotcore/set_filter_profile_safe', Trigger)

ros_control.wait_for_service()
enable_filter.wait_for_service()
set_filter_profile_safe.wait_for_service()

enable_filter(True)
set_filter_profile_safe()
ros_control(True)

# 
tasks = ['arm1_8', 'arm2_8']

set_impedance = {
    t: rospy.ServiceProxy(f'/cartesian/{t}/set_impedance', SetImpedance) for t in tasks
}

force_pub = {
    t: rospy.Publisher(f'/cartesian/{t}/force_reference', WrenchStamped, queue_size=1) for t in tasks
}

for k, srv in set_impedance.items():
    srv.wait_for_service()


def send_force(tev):
    
    msg = WrenchStamped()    
    msg.wrench.force.y = -20
    force_pub['arm1_8'].publish(msg)

    msg = WrenchStamped()    
    msg.wrench.force.y = 20
    force_pub['arm2_8'].publish(msg)

force_timer = rospy.Timer(rospy.Duration(0.05), send_force)

imp_req = SetImpedanceRequest() 

imp_req.impedance.linear.stiffness.x = 500
imp_req.impedance.linear.stiffness.y = 40
imp_req.impedance.linear.stiffness.z = 500
imp_req.impedance.linear.damping_ratio.x = .7
imp_req.impedance.linear.damping_ratio.y = .7
imp_req.impedance.linear.damping_ratio.z = .7

imp_req.impedance.angular.stiffness.x = 300
imp_req.impedance.angular.stiffness.y = 40
imp_req.impedance.angular.stiffness.z = 300
imp_req.impedance.angular.damping_ratio.x = .7
imp_req.impedance.angular.damping_ratio.y = .7
imp_req.impedance.angular.damping_ratio.z = .7

for k, srv in set_impedance.items():
    res = srv(imp_req)
    print(res)

rospy.spin()