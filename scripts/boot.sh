#!/usr/bin/env bash

cd $(dirname $0)
cd ..

# attempt to update the repo
isOnline=$(ping -q -c1 google.com &>/dev/null && echo online || echo offline)
if [ "$isOnline" = "online" ]; then
    git restore .
    git pull
else
    echo "Wi-Fi not connected. Skipping repo update."
fi


# start the process
# source env/bin/activate
# python main.py  