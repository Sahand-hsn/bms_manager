#!/bin/bash

sudo ifconfig can0 txqueuelen 65536
sudo ip link set can0 up type can bitrate 250000
