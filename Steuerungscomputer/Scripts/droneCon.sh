#!/bin/bash
sleep 10
nohup sudo /home/pi/RPi/HMD/mavCon.sh &
nohup sudo python /home/pi/RPi/HMD/getMatlabParams.py &
#sudo /home/pi/HMD/getMatlabParamsSer.py &
