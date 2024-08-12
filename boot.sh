#!/usr/bin/env bash

cd /home/$1/Desktop/Chandelier

# update the code to latest
git restore .
git pull
source ./env/bin/activate
python ./main.py