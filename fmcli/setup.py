#!/bin/sh
sudo apt-get install -y tmux python-virtualenv
virtualenv py
source py/bin/activate
pip install tmuxp urwid

