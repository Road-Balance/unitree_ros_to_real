#!/bin/bash

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo ifconfig enx00e04c6808ac down
sudo ifconfig enx00e04c6808ac up 192.168.123.162 netmask 255.255.255.0

