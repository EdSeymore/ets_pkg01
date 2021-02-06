#!/usr/bin/env python

# File: watch_battery.py 
# Function: Display the battery voltage and percentage used. 
# Modified by EdSeymoe17 02/02/2021
# Change: 02/02/2021 02/03/2021
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
from sensor_msgs.msg import BatteryState

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
cnt                 = 0
limit               = 10
rpt_voltage         = True
rpt_percentage      = False
new_percentage      = 0  # Remember prior value.
prior_percentage    = 0  # Remember prior value.
new_voltage         = 0  # Remember prior value.
prior_voltage       = 0  # Remember prior value.

# wa  = get_value()
# loop: 
#   new = get_value()
#   wa = 9*wa + .1*new 
#   print wa

def callback(data):
    global prior_percentage, prior_voltage, rpt_voltage, rpt_percentage, cnt, limit, new_voltage, new_percentage
    # print "Battery callback called"
    cnt += 1
    if cnt > limit:
        cnt = 0
    if True:
        if prior_percentage == 0:
            new_percentage = data.percentage
            prior_percentage = new_percentage
            rospy.loginfo(rospy.get_caller_id() + ' Battery = {:.0f} percent'.format(new_percentage*100))
        new_percentage = data.percentage
        if round(new_percentage,2) != round(prior_percentage,2):
            if cnt == limit:
                if rpt_percentage:
                    rospy.loginfo(rospy.get_caller_id() + ' Battery = {:.0f} percent'.format(new_percentage*100))
                prior_percentage = new_percentage
            if new_percentage < .20:      # ets 02/03/2021
                print "Low Voltage warning Battery = {:.0f} percent".format(new_percentage*100)

    if True:
        if prior_voltage == 0:
            new_voltage = data.voltage
        if round(new_voltage) != round(prior_voltage):
            if cnt == limit:
                if rpt_voltage:          
                    rospy.loginfo(rospy.get_caller_id() + ' Battery voltage = {:2.0f} '.format(new_voltage))
                prior_voltage = new_voltage
            if new_voltage < 23.0:      # ets 02/03/2021
                print "Low Voltage warning  Battery voltage = {:2.0f} ".format(new_voltage)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print "Register watch_battery "         # ets 01/31/2021
    rospy.init_node('watch_battery', anonymous=True)

    print "Look for battery_state messages "   # ets 01/31/2021
    # rospy.Subscriber('pi_sonar/sonar_3', String, callback)     # ets 01/31/2021
    rospy.Subscriber('/battery_state', BatteryState, callback)   # ets 01/31/2021

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print "app watch_battery [version 1] has started"  # ets 02/03/2021
    listener()
    print "\napp watch_battery has ended"  # ets 02/01/2021

