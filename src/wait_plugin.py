#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool
from xbot_msgs.msg import LifecycleEvent
import sys 

rospy.init_node('run_plugin_node', anonymous=True)

plugin = sys.argv[1]

# subscribe to lifecycle events
def on_le_recv(msg: LifecycleEvent):
    print(msg)
    if msg.plugin_name == plugin and msg.to_state == 'Stopped':
        rospy.loginfo(f'plugin {plugin} stopped: success')
        rospy.signal_shutdown('done')

sub = rospy.Subscriber(name='/xbotcore/lifecycle_events', data_class=LifecycleEvent, callback=on_le_recv, queue_size=100)

srvname = f'/xbotcore/{plugin}/switch'

rospy.loginfo(f'waiting for service {srvname}')
switch = rospy.ServiceProxy(srvname, service_class=SetBool)
rospy.wait_for_service(srvname)

# call service
rospy.loginfo('calling service..')

res = switch(True)
rospy.loginfo(res)
rospy.loginfo('waiting for exit..')

rospy.spin()


