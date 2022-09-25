##sitl_gazebo
gnome-terminal --window -e 'zsh -c "roscore; exec zsh"' \
--tab -e 'zsh -c "sleep 1; roslaunch px4 posix_sitl.launch; exec zsh"' \
--tab -e 'zsh -c "sleep 1; roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"; exec zsh"' \
--tab -e 'zsh -c "sleep 2; rosrun px4_cmd set_command; exec zsh"' \
--tab -e 'zsh -c "sleep 2; rosrun px4_cmd set_mode; exec zsh"' \
--tab -e 'zsh -c "sleep 2; rosrun px4_cmd send_command; exec zsh"' \
