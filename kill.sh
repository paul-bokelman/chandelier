#!/usr/bin/env bash

sudo ps -ax | pgrep -f 'python ./main.py' | xargs sudo kill
