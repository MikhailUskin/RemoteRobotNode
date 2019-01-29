#!/bin/bash
rostopic pub /car/actuator_data geometry_msgs/Twist --once -- "[$1.0,0.0,0.0]" "[0.0, 0.0, $2.0]"
