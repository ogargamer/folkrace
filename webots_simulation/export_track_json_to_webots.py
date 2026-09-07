#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - WEBOTS SIMULATION WORLD GENERATOR FROM MODULAR TRACK JSON
================================================================================
Converts any track layout JSON (from Track Studio / maps folder) to a Webots
.wbt world file with ENU Standard coordinate system (Z-Up).
================================================================================
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, "track_builder"))

from export_webots import convert_json_to_webots_wbt, main

if __name__ == "__main__":
    main()
