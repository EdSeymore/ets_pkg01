ets_pkg01 README.md: 
-------------------- 

This is my first ROS package. It was built to examine the ROS messages from the Magni robot. 

Environment: ubuntu 16.04, ROS Kinetic Kame, python 2.7 

The main applications are the following: 

- robot_safety      - To notify when battery is low or object is too close to robot. 
- display_sonar     - Display the five Magni sonar distances. 
- watch_battery     - Display the battery voltage and percentage used. 
- sonar_listener    - Display the front facing robot sonar distance. 

The terminal input to run the python applications are the following: 

rosrun ets_pkg01 robot_safety.py 

rosrun ets_pkg01 display_sonar.py 

rosrun ets_pkg01 watch_battery.py 

rosrun ets_pkg01 sonar_listener.py 

rosrun ets_pkg01 add_two_ints_client.py 1 3 

rosrun ets_pkg01 add_two_ints_server.py 

rosrun ets_pkg01 listener.py 

rosrun ets_pkg01 talker.py 

Also: 

cd /home/edmond/catkin_ws/src/ets_pkg01/scripts/ 

python2 sonar_listener.py 

python2 watch_battery.py 

python2 display_sonar.py 

python2 robot_safety.py 


The code was based on the ROS Tutorial instructions: 

Ref: http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29 

Ref: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 

And other online sampe code: 

Ref: https://www.programcreek.com/python/example/14044/rospy.Subscriber 

Ref: https://www.theconstructsim.com/read-laserscan-data/ 


