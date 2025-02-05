#!/bin/bash

if [ "$(id -u)" -ne 0 ]; then
        echo 'This script must be run by root' >&2
        exit 1
fi

BASHRC='/home/pi/.bashrc'

function removefrom_bashrc {
    LINE=$1
    grep -v "$LINE" $BASHRC > temp && mv temp $BASHRC
    return
}


# TODO: restore old hw_find.service file
# echo
# echo 'Stopping hw_find service'
# systemctl stop hw_find.service
# systemctl disable hw_find.service
# rm /etc/systemd/system/hw_find.service
# echo 'Linking /etc/systemd/system/hw_find.service --> /home/pi/boot/hw_find.service'
# ln -sf /home/pi/boot/hw_find.service /etc/systemd/system/hw_find.service
# echo 'Restarting hw_find discovery service'
# systemctl enable hw_find.service
# systemctl start hw_find.service
# echo


echo 'Stopping buttonman'
systemctl stop buttonman.service
systemctl disable buttonman.service
echo 'Removing symlink /etc/systemd/system/buttonman.service'
rm /etc/systemd/system/buttonman.service

echo 'Stopping rgbd.service (RGB daemon)'
systemctl stop rgbd.service
systemctl disable rgbd.service
echo 'Removing symlink /etc/systemd/system/rgbd.service'
rm /etc/systemd/system/rgbd.service

echo 'Enabling hw_button_scan.service'
systemctl enable hw_button_scan.service
systemctl start hw_button_scan.service

echo 'Removing aliases from bashrc'
removefrom_bashrc "alias batt='sudo python3 /home/pi/boot/battchk.py'"
echo 'Done'
