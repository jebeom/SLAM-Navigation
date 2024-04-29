#!/bin/bash
source ~/catkin_ws/devel/setup.bash && roscd robot_bringup
sudo cp controller/50-ds4drv.rules /etc/udev/rules.d/
sudo cp controller/ds4drv.service /etc/systemd/system/
sudo cp controller/ds4drv.conf /etc/

sudo systemctl enable ds4drv.service
sudo systemctl start ds4drv.service

cp controller/robotnik-bt-gamepad-auto-pair*.deb /tmp/
sudo apt install /tmp/robotnik-bt-gamepad-auto-pair*.deb -y