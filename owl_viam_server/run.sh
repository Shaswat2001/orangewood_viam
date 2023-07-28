#!/bin/sh
cd `dirname $0`

pip3 install -r requirements.txt

# Be sure to use `exec` so that termination signals reach the python process,
# or handle forwarding termination signals manually
exec /home/shaswatgarg/.venvs/viam_venv/bin/python3 -m src.owl_robot_server $@