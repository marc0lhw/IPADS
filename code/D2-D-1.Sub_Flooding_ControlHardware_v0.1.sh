#!/bin/bash

# Loop to run Subscribers 100 times with different names.
for i in {1..100}
do
  python3 D2-D-1.ControlHardware_v0.1.py $i &
done

