#!/bin/bash

sudo apt remove python-pip

wget https://bootstrap.pypa.io/get-pip.py

sudo python get-pip.py --prefix=/usr/local

rm get-pip.py

sudo pip install pexpect
sudo pip install pytest
