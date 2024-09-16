#!/usr/bin/env bash

sudo ps -ax | pgrep -f 'python ./main.py' | xargs sudo kill
cd /home/chandelier2/Desktop/Chandelier
source ./env/bin/activate
python ./helper-scripts/stop.py
