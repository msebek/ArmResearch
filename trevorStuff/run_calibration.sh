#!/bin/bash
~/opencv-2.4.9/build/bin/cpp-example-calibration calibrate calibration_xml.xml -h 8 -w 6 -s 0.1524 -o output.yaml -oe -op
