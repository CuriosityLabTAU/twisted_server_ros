#!/bin/sh
#killall rosmaster
#killall roslaunch
#pkill -9 python
#sleep 0.5s
export ROS_MASTER_URI="http://${ROS_IP}:11511"
echo "ROS_IP: $ROS_IP"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"

WID1=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID1
xdotool key ctrl+shift+t
wmctrl -i -a $WID1

sleep 2; xdotool type --delay 1 --clearmodifiers "echo -ne '\033]0;roscore\007'; roscore -p 11511"; xdotool key Return;


WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
wmctrl -i -a $WID

sleep 1; xdotool type --delay 1 --clearmodifiers "export ROS_MASTER_URI=\"http://${ROS_IP}:11511\"; roslaunch rosbridge_server rosbridge_websocket.launch port:=8080"; xdotool key Return;

WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
wmctrl -i -a $WID

sleep 1; xdotool type --delay 1 --clearmodifiers "export ROS_MASTER_URI=\"http://${ROS_IP}:11511\"; echo -ne '\033]0;twisted_server\007'; python src/twisted_server_ros2.py"; xdotool key Return;
