#!/usr/bin/env bash

sudo ps -ax | pgrep -f 'python ./main.py' | xargs sudo kill
cd /home/kevin/Desktop/Chandelier
source ./env/bin/activate
python ./helper-scripts/stop.py
