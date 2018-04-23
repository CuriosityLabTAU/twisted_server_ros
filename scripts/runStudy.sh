#!/bin/bash
echo -e "\033[1m\033[42mCTRL-C here to stop all\033[0m"
echo
#echo ">>Quick Troubleshoot<<"
#echo
#echo "to restart speech rec:"
#echo -e "\033[1m\033[36m./restart_speech.sh\033[0m"
#echo
#echo "to restart backchannel:"
#echo -e "\033[1m\033[36m./restart_backchannel.py\033[0m"

xterm -geometry 85x40+300+0 -T "msg_to_game" -e bash -c "python scripts/send_msg_to_game.py $1 $2 $3" &
xterm -geometry 45x20+840+0 -T "Front Camera" -e bash -c "roslaunch cv_camera cv_camera.launch" 


