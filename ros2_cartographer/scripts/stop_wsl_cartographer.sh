#!/usr/bin/env bash
pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
pkill -9 -f webots_cartographer_bridge.py 2>/dev/null || true
