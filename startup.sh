#!/bin/bash

sudo systemctl enable ssh
sudo systemctl start ssh

bluetoothctl << EOF
power on
connect 40:1B:5F:6E:8E:43
exit
EOF

sudo python /home/pi/dualshockSerial.py
