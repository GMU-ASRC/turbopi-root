#!/bin/bash

if [ "$(id -u)" -ne 0 ]; then
        echo 'This script must be run by root' >&2
        exit 1
fi

BASHRC='/home/pi/.bashrc'

function update_bashrc {
    LINE=$1
    eval $LINE
    grep -qF -- "$LINE" "$BASHRC"
    RES=$?
    if [ $RES -eq 1 ]; then
        echo "$LINE" >> "$BASHRC"
        return 0
    else
        return 1
    fi
}

echo 'Updating pip'

python -m pip install --upgrade pip
echo
echo 'Installing Dependencies'
echo
pip install psutil python-statemachine

ln -sf /home/pi/boot/buttonman.service /etc/systemd/system/buttonman.service
echo
echo 'buttonman service was linked to /etc/systemd/system/buttonman.service'
echo
echo 'Replacing hw_button_scan.service with buttonman.service'
echo
systemctl disable hw_button_scan.service
systemctl stop hw_button_scan.service
systemctl enable buttonman.service
systemctl start buttonman.service

echo
echo 'Removing old hw_find service'
systemctl stop hw_find.service
systemctl disable hw_find.service
rm /etc/systemd/system/hw_find.service
echo 'Linking /etc/systemd/system/hw_find.service --> /home/pi/boot/hw_find.service'
ln -sf /home/pi/boot/hw_find.service /etc/systemd/system/hw_find.service
echo 'Restarting hw_find discovery service'
systemctl enable hw_find.service
systemctl start hw_find.service
echo
echo 'Checking if aliases already installed...'
update_bashrc "alias batt='sudo python3 /home/pi/boot/battchk.py'"
if [ $? -eq 0 ]; then
    echo 'Added alias for battery check: batt'
else
    echo 'Alias already exists for battery check: batt'
fi
echo
echo Done!
