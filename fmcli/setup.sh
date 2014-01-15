#!/bin/sh
sudo apt-get install -y python-virtualenv
sudo dpkg -i tmux_1.8-4_amd64.deb
virtualenv py
. py/bin/activate
pip install tmuxp urwid

