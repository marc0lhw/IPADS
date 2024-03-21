#!/bin/bash

# different node name -> attack

ros2 topic pub --rate 10 /can/control_hardware can_msgs/ControlHardware "{horn: false, head_light: false, left_light: false, right_light: true}"