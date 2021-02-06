#!/usr/bin/env python

# File: display_sonar.py 
# Function: Display the five Magni sonar distances. 
# Modified by EdSeymoe17 01/31/2021
# Change: 02/02/2021 02/03/2021 02/04/2021 
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Range   # [sonar]

# Range:
# ------ 
# rosmsg show Range
# [sensor_msgs/Range]:
# uint8 ULTRASOUND=1
# uint8 INFRARED=0
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# uint8 radiation_type
# float32 field_of_view
# float32 min_range
# float32 max_range
# float32 range 
# ----------------------------- 

cnt        = 0
limit      = 20
prior_dist = 0        # Remember prior value.
sd         = [0]*10   # Sonar array [list of distances].

def callback(data):
    global prior_dist, cnt, limit, sd
    # sdt    = data.radiation_type
    rtime  = data.header.stamp   
    frame  = data.header.frame_id
    range  = data.range
    if frame == 'sonar_0': # far right 
      sd[0] =  range
    if frame == 'sonar_1': # left front 
      sd[1] =  range      
    if frame == 'sonar_2': # right front 
      sd[2] =  range
    if frame == 'sonar_3': # front
      sd[3] =  range
    if frame == 'sonar_4': # far left
      sd[4] =  range            

    cnt += 1
    if cnt > limit:
        cnt = 0    
    if prior_dist == 0:
        new_dist = sd[3]
        prior_dist = new_dist
        rospy.loginfo(rospy.get_caller_id() + ' Distance = {:2.2f} meters'.format(new_dist))
    new_dist = sd[3]
    if round(new_dist,2) != round(prior_dist,2):
        if cnt == limit:
            # rospy.loginfo(rospy.get_caller_id() + ' Distance = {:2.2f} meters'.format(new_dist))
            prior_dist = new_dist
            print "Distance L {:5.2f}, LF {:5.2f}, F {:5.2f} ,RF {:5.2f}, R {:5.2f} ".format(sd[4], sd[1], sd[3], sd[2], sd[0])

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #print "Register sonar_listener "         # ets 01/31/2021
    rospy.init_node('sonar_listener', anonymous=True)

    # print "Look for sonar three messages "   # ets 01/31/2021
    # rospy.Subscriber('pi_sonar/sonar_3', String, callback)     # ets 01/31/2021
    rospy.Subscriber('sonars', Range, callback)     # ets 01/31/2021

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print "app display_sonar [version 1] has started"   # ets 02/03/2021
    listener()
    print "\napp display_sonar has ended"               # ets 02/03/2021

