#!/usr/bin/env bash

cd $(dirname $0)
cd ..
sudo ps -ax | pgrep -f 'python main.py' | xargs sudo kill
source env/bin/activate
python helpers/stop.py
