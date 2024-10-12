#!/usr/bin/env bash

sudo ps -ax | pgrep -f 'python ./main.py' | xargs sudo kill
cd /home/chandelier2/Desktop/chandelier
source ./env/bin/activate
python ./helpers/stop.py
