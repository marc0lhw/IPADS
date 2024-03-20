#!/bin/bash

ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello, world!'"

