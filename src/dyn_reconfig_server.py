#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from controller_adaptiveclbf.cfg import adaptiveClbfConfig as ConfigType

def reconfigure_cb(config, dummy):
    """Create a callback function for the dynamic reconfigure server."""
    return config

if __name__ == "__main__":
    rospy.init_node('controller_adaptiveclbf_reconfig')

    # Create a dynamic reconfigure server.
    server = DynamicReconfigureServer(ConfigType, reconfigure_cb)
    rospy.spin()
