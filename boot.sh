#!/usr/bin/env bash

# Check if Wi-Fi is connected
if iwconfig 2>&1 | grep -q "ESSID:\""; then
    cd /home/chandelier2/Desktop/Chandelier

    # update the code to latest
    git restore .
    git pull
    source ./env/bin/activate
    python ./main.py -m=normal
else
    echo "Wi-Fi not connected. Skipping repo update."
fi