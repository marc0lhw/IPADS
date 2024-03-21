#!/bin/bash

# Loop to run 'ros2 topic pub' 1000 times
for i in {1..1000}
do
  ros2 topic pub -n WmMotionControllerNode --rate 100 /can/control_hardware can_msgs/ControlHardware "{horn: false, head_light: false, left_light: false, right_light: true}" &

done

