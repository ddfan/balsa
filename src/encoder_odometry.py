#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import rospy
import roslib
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("encoder_odometry")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',50.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 12.0*40.0/(2.0*pi*0.05)))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.2)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
         
        # internal data
        self.enc_fleft = None        # wheel encoder readings
        self.enc_fright = None
        self.enc_rleft = None
        self.enc_rright = None
        self.fleft = 0               # actual values coming back from robot
        self.fright = 0
        self.rleft = 0
        self.rright = 0
        
        self.flmult = 0
        self.frmult = 0
        self.rlmult = 0
        self.rrmult = 0
        
        self.prev_flencoder = 0
        self.prev_frencoder = 0
        self.prev_rlencoder = 0
        self.prev_rrencoder = 0

        self.d_fleft = 0
        self.d_fright = 0
        self.d_rleft = 0
        self.d_rright = 0

        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.past_time = rospy.Time.now()
        
        # subscriptions
        rospy.Subscriber("/encoder/front_left", Int32, self.flwheelCallback)
        rospy.Subscriber("/encoder/front_right", Int32, self.frwheelCallback)
        rospy.Subscriber("/encoder/rear_left", Int32, self.rlwheelCallback)
        rospy.Subscriber("/encoder/rear_right", Int32, self.rrwheelCallback)
        self.odomPub = rospy.Publisher("encoder/odom", Odometry, queue_size=10)
        # self.odomBroadcaster = TransformBroadcaster()
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            r.sleep()
            self.update()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        self.start_time = rospy.Time.now()
        elapsed = (self.start_time - self.past_time).to_sec() + 1.0e-6
        
        if elapsed < 1.0 / self.rate * 0.8:
            return

        self.past_time = self.start_time

        # calculate odometry
        if self.enc_fleft == None:
            self.d_fleft = 0
            self.d_fright = 0
            self.d_rleft = 0
            self.d_rright = 0
        else:
            self.d_fleft = (self.fleft - self.enc_fleft) / self.ticks_meter
            self.d_fright = (self.fright - self.enc_fright) / self.ticks_meter
            self.d_rleft = (self.rleft - self.enc_rleft) / self.ticks_meter
            self.d_rright = (self.rright - self.enc_rright) / self.ticks_meter
        
        self.enc_fleft = self.fleft
        self.enc_fright = self.fright
        self.enc_rleft = self.rleft
        self.enc_rright = self.rright

        # distance traveled is the average of the 4 wheels 
        d = ( self.d_fleft + self.d_fright + self.d_rleft + self.d_rright) / 4.0
        # this approximation works (in radians) for small angles
        th = ( (self.d_fright + self.d_rright)/2.0  - (self.d_fleft + self.d_rleft)/2.0 ) / self.base_width
        # calculate velocities
        dx_old = self.dx
        self.dx = d / elapsed
        self.dr = th / elapsed
         
        if (d != 0):
            # calculate distance traveled in x and y
            x = cos( th ) * d
            y = -sin( th ) * d
            # calculate the final position of the robot
            self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
            self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
        if( th != 0):
            self.th = self.th + th
            
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin( self.th / 2 )
        quaternion.w = cos( self.th / 2 )
        # self.odomBroadcaster.sendTransform(
        #     (self.x, self.y, 0),
        #     (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        #     rospy.Time.now(),
        #     self.base_frame_id,
        #     self.odom_frame_id
        #     )
        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)
        
        


    #############################################################################
    def flwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_flencoder > self.encoder_high_wrap):
            self.flmult = self.flmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_flencoder < self.encoder_low_wrap):
            self.flmult = self.flmult - 1
            
        self.fleft = 1.0 * (enc + self.flmult * (self.encoder_max - self.encoder_min)) 
        self.prev_flencoder = enc
        
    #############################################################################
    def frwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_frencoder > self.encoder_high_wrap):
            self.frmult = self.frmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_frencoder < self.encoder_low_wrap):
            self.frmult = self.frmult - 1
            
        self.fright = 1.0 * (enc + self.frmult * (self.encoder_max - self.encoder_min))
        self.prev_frencoder = enc

    #############################################################################
    def rlwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rlencoder > self.encoder_high_wrap):
            self.rlmult = self.rlmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rlencoder < self.encoder_low_wrap):
            self.rlmult = self.rlmult - 1
            
        self.rleft = 1.0 * (enc + self.rlmult * (self.encoder_max - self.encoder_min))
        self.prev_rlencoder = enc

        #############################################################################
    def rrwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rrencoder > self.encoder_high_wrap):
            self.rrmult = self.rrmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rrencoder < self.encoder_low_wrap):
            self.rrmult = self.rrmult - 1
            
        self.rright = 1.0 * (enc + self.rrmult * (self.encoder_max - self.encoder_min))
        self.prev_rrencoder = enc

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
