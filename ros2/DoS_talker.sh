#!/bin/bash

# Loop to run 'ros2 topic pub' 100 times
for i in {1..100}
do
  ros2 topic pub -n talker /chatter std_msgs/msg/String &
done

