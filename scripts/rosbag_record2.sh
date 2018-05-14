echo $1 $2 $3
rosbag record -o ../rosbag/$1_s$2_$3 -e /jibo /jibo_state /jibo_asr_command /log /to_twisted /from_twisted /usb_cam/image_raw/compressed


