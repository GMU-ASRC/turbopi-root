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

echo 'Removing symlink /etc/systemd/system/buttonman.service'
systemctl stop buttonman.service
systemctl disable buttonman.service
rm /etc/systemd/system/buttonman.service
echo 'Removing symlink /etc/systemd/system/batterywatcher.service'
systemctl stop batterywatcher.service
systemctl disable batterywatcher.service
rm /etc/systemd/system/batterywatcher.service
echo 'Enabling hw_button_scan.service'
systemctl enable hw_button_scan.service
systemctl start hw_button_scan.service
echo 'Removing aliases from bashrc'
removefrom_bashrc "alias batt='sudo python3 /home/pi/boot/battchk.py'"
echo 'Done'
