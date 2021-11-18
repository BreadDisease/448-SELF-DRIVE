#!/bin/bash
pkill -o chromium
pkill remote
pkill python3

mpirun -np 2 remote &
python3 server.py &
sleep 2
export DISPLAY=:0
/usr/bin/chromium-browser --kiosk /home/pi/remote/web/index.html
