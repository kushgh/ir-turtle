#!/bin/sh

if [ -e /dev/ttyACM0 ]
then
    bash -c "sudo chmod a+rw /dev/ttyACM0"
    bash -c "source /home/$USER/Documents/ir-turtle/catkin_ws/devel/setup.sh && roslaunch ir_turtle irturtle.launch"
else
    echo "Arduino is not connected. Please connect arduino and try again."
fi
exit 0
