#!/bin/bash
roscore &
rosrun rosserial_python serial_node.py tcp &
