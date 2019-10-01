#!/bin/bash
cd $(rospack find rosweld_tools)
export ROS_NAMESPACE=/rosweld
python -m src.app