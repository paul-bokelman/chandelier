#!/bin/bash

cd $(dirname $0)
cd ..

sleep 20 # wait for processes to be active

# start the process
source env/bin/activate
python main.py -e=production -m=normal -c=default