#!/usr/bin/env bash

# update code before rebooting

# Check if Wi-Fi is connected
if iwconfig 2>&1 | grep -q "ESSID:\""; then
    cd /home/chandelier2/Desktop/chandelier

    # update the code to latest
    git restore .
    git pull

    sudo reboot
    
else
    echo "Wi-Fi not connected. Skipping repo update."
fi