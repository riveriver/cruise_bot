rosrun map_server map_saver map:=/projected_map
roslaunch ros_rtsp rtsp_streams.launch
roslaunch cruise_nav nav_chiwan.launch
sudo chmod 666 /dev/ttyACM*
roslaunch cruise_decision decision_chiwan.launch
roslaunch theta_z1 start_theta.launch 
rtopic pub /start_theta std_msgs/Bool "data: TRUE"

