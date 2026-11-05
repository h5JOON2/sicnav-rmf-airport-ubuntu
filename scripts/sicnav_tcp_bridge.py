#!/usr/bin/env python3
"""
SICNav TCP Bridge Node (Client connecting to macOS SICNav Server).
Wrapper pointing to SicnavGazeboClient.
"""
import os
import sys

# Ensure current script directory is in sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from sicnav_gazebo_client import main

if __name__ == '__main__':
    main()
