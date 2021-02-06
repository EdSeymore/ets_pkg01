#!/usr/bin/env python

# File: robot_safety.py 
# Function: To notify when battery is low or object is too close to robot. 
# Modified by EdSeymore17 01/31/2021 
# Change: 02/02/2021 02/03/2021 02/04/2021 02/05/2021
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
from sensor_msgs.msg import Range         # [sonar]
from sensor_msgs.msg import BatteryState

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

# BatteryState:
# ------ 
# rosmsg show BatteryState
# [sensor_msgs/BatteryState]:
# uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
# uint8 POWER_SUPPLY_STATUS_CHARGING=1
# uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
# uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
# uint8 POWER_SUPPLY_STATUS_FULL=4
# uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
# uint8 POWER_SUPPLY_HEALTH_GOOD=1
# uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2
# uint8 POWER_SUPPLY_HEALTH_DEAD=3
# uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4
# uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
# uint8 POWER_SUPPLY_HEALTH_COLD=6
# uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
# uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8
# uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
# uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1
# uint8 POWER_SUPPLY_TECHNOLOGY_LION=2
# uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3
# uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4
# uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5
# uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32 voltage
# float32 current
# float32 charge
# float32 capacity
# float32 design_capacity
# float32 percentage
# uint8 power_supply_status
# uint8 power_supply_health
# uint8 power_supply_technology
# bool present
# float32[] cell_voltage
# string location
# string serial_number

# [pr2_msgs/BatteryState]:
# int32 lastTimeBattery
# uint16[48] batReg
# uint16[48] batRegFlag
# int32[48] batRegTime
# --------------- 

class RobotSafety(object): 
  def __init__(self):
    self.sonar            = True
    self.battery          = True
    self.rpt_voltage      = True
    self.rpt_percentage   = False
    self.rpt_distance     = False
    # self.value            = 0 
    self.cnt1             = 0       # Sonar information.
    self.limit1           = 1
    self.cnt2             = 0       # Battery information.
    self.limit2           = 10
    self.new_dist         = 0       # Remember prior value.
    self.prior_dist       = 0       # Remember prior value.
    self.sd               = [0]*10  # Sonar array [list of distances].
    self.new_percentage   = 0       # Remember prior value.
    self.prior_percentage = 0       # Remember prior value.
    self.new_voltage      = 0       # Remember prior value.
    self.prior_voltage    = 0       # Remember prior value.

  def sonar_callback(self,data):
    # print "Sonar callback called"
    # self.sdt    = data.radiation_type
    # rtime  = data.header.stamp   
    frame  = data.header.frame_id
    range  = data.range
    # if range > data.max_range: 
    #   range = data.max_range
    # if range < data.min_range:
    #   range = data.min_range
    if range == 0:
      print "Sonar record excluded [zero value]"   # ets 02/05/2021
      return
    if frame == 'sonar_0': # far right 
      self.sd[0] = range
    if frame == 'sonar_1': # left front 
      self.sd[1] = range      
    if frame == 'sonar_2': # right front 
      self.sd[2] = range
    if frame == 'sonar_3': # front
      self.sd[3] = range
    if frame == 'sonar_4': # far left
      self.sd[4] = range            

    if frame == 'sonar_3':
      if self.prior_dist == 0:
        self.new_dist = self.sd[3]
        self.prior_dist = self.new_dist
        print "Current front distance  = {:5.2f} meters".format(self.new_dist)                      # ets 02/05/2021
        print "Dist min = {:5.2f}    max = {:5.2f} meters".format(data.min_range, data.max_range)  # ets 02/05/2021
        # if data.radiation_type == 1:  # ets 02/05/2021
        #   print "Device type = sonar"
        # if data.radiation_type == 0:  # ets 02/05/2021
        #   print "Device type = laser"
        # rospy.loginfo(rospy.get_caller_id() + ' Front distance = {:2.2f} meters'.format(self.new_dist))
        if self.new_dist < 0.10:
          print "Obstacle detected in front {:5.2f} meters".format(self.sd[3])
      if True:
        if self.cnt1 >= self.limit1:
          self.cnt1 = 0
        self.cnt1 += 1
        self.new_dist = self.sd[3]
        # self.new_dist = (.9*self.prior_dist) + (.1*self.sd[3])  # ets 02/05/2021
        if self.cnt1 == self.limit1:
          if round(self.new_dist,1) != round(self.prior_dist,1):
            # print "new = {:2.4f}  old = {:2.4f} meters".format(round(self.new_dist,1), round(self.prior_dist,1))  # ets 02/05/2021
            self.prior_dist = self.new_dist
            if self.rpt_distance:
              # rospy.loginfo(rospy.get_caller_id() + ' Front distance = {:2.2f} meters'.format(self.new_dist))
              print "Distance L {:5.2f}, LF {:5.2f}, F {:5.2f} ,RF {:5.2f}, R {:5.2f} ".format(self.sd[4], self.sd[1], self.new_dist, self.sd[2], self.sd[0])
            if self.new_dist <= 0.20:
              print "Obstacle detected in front {:5.2f} meters".format(self.new_dist)

  def battery_callback(self,data):
    # print " Battery callback called"
    if data.percentage == 0:
      # print "Battery percentsage record excluded [zero value]"  # ets 02/05/2021
      return
    if data.voltage == 0:
      print "Battery voltage record excluded [zero value]"      # ets 02/05/2021
      return
    if self.cnt2 >= self.limit2:
      self.cnt2 = 0
    self.cnt2 += 1
    if True:
      if self.prior_percentage == 0:
        self.new_percentage = data.percentage
        self.prior_percentage = self.new_percentage
        # rospy.loginfo(rospy.get_caller_id() + ' Battery = {:.0f} percent'.format(self.new_percentage*100))
        print 'Battery = {:.0f} percent'.format(self.new_percentage*100)
        if self.new_percentage < .20:      # ets 02/03/2021
          print 'Low Voltage warning Battery = {:.0f} percent'.format(self.new_percentage*100)
    if data.voltage== 0:
      print "Battery voltage record excluded [zero value]"   # ets 02/05/2021
      pass
    else:
      if self.prior_voltage == 0:
        new_voltage = data.voltage
        self.prior_voltage = new_voltage
        # rospy.loginfo(rospy.get_caller_id() + ' Battery voltage = {:2.0f} '.format(new_voltage))
        print 'Battery voltage = {:2.0f} '.format(new_voltage)
        if new_voltage < 23.0:
          print "Low Voltage warning  Battery voltage = {:2.0f} ".format(new_voltage)
      # new_voltage = data.voltage
      new_voltage = .9*self.prior_voltage + .1*data.voltage
      if round(new_voltage) != round(self.prior_voltage):
        if self.cnt2 == self.limit2:
          if self.rpt_voltage:
            rospy.loginfo(rospy.get_caller_id() + ' Battery voltage = {:2.0f} '.format(new_voltage))
          self.prior_voltage = new_voltage
          if new_voltage < 23.0:
            print "Low Voltage warning  Battery voltage = {:2.0f} ".format(self.new_voltage)

  def listener(self):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    print "Register robot_safety "
    rospy.init_node('robot_safety', anonymous=True)

    if self.sonar:
      print "Look for sonar messages " 
      # rospy.Subscriber('pi_sonar/sonar_3', String, callback)     # ets 01/31/2021
      rospy.Subscriber('sonars', Range, self.sonar_callback)

    if self.battery:
      print "Look for battery_state messages "
      rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
  print "app robot_safety [version 1] has started"
  rs = RobotSafety()
  rs.listener()
  print "\napp robot_safety has ended"

