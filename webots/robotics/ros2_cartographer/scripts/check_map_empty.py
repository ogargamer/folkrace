#!/usr/bin/env python3
"""
================================================================================
  MAP CONTENT VALIDATOR & NON-EMPTY CHECKER
  Verifies whether an exported PNG map contains rendered track data (non-empty).
================================================================================
"""

import sys
import os
import numpy as np
from PIL import Image

def check_map_file(png_path):
    if not os.path.exists(png_path):
        return False, "File does not exist"
    
    if os.path.getsize(png_path) < 1000:
        return False, "File size too small (<1KB)"

    try:
        im = Image.open(png_path).convert('RGB')
        arr = np.array(im)
        # Check standard deviation across channels/pixels to detect blank/solid frames
        std_dev = np.std(arr)
        unique_colors = len(np.unique(arr.reshape(-1, arr.shape[2]), axis=0))
        
        if std_dev < 1.0 or unique_colors < 10:
            return False, f"Blank image (std_dev={std_dev:.2f}, unique_colors={unique_colors})"
        
        return True, f"Valid map image (unique_colors={unique_colors}, std_dev={std_dev:.2f})"
    except Exception as e:
        return False, f"Image read error: {e}"

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python check_map_empty.py <path_to_png>")
        sys.exit(1)
    
    is_valid, msg = check_map_file(sys.argv[1])
    print(f"[{'VALID' if is_valid else 'EMPTY'}] {sys.argv[1]}: {msg}")
    sys.exit(0 if is_valid else 1)
