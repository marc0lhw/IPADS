#!/bin/bash

# Loop to run 'ros2 topic pub' 100 times in the background
for i in {1..100}
do
  ros2 topic echo /chatter &
done
