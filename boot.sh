#!/usr/bin/env bash

cd /home/kevin/Desktop/Chandelier

# update the code to latest
git restore .
git pull
source ./env/bin/activate
python ./main.py -m=auto
# python ./blank-test.py