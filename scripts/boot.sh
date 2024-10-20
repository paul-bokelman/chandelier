#!/bin/bash

cd $(dirname $0)
cd ..

# start the process
source env/bin/activate
python main.py  