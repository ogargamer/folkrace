#!/usr/bin/env python3
import sys
import os
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import numpy as np

class MapSaverPNG(Node):
    def __init__(self, out_path):
        super().__init__('map_saver_png')
        self.out_path = out_path
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.done = False

    def map_callback(self, msg):
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Reject uninitialized or singular 0x0 grids from initial ROS2 topic creation
        if w <= 0 or h <= 0 or abs(origin_x) > 1e10 or abs(origin_y) > 1e10 or len(msg.data) == 0:
            self.get_logger().warn(f"Received uninitialized /map ({w}x{h}, origin=[{origin_x:.2f}, {origin_y:.2f}]), waiting for valid submaps...")
            return

        self.get_logger().info(f"Received valid /map from Cartographer: {w}x{h} cells (res={res:.3f}m, origin=[{origin_x:.2f}, {origin_y:.2f}])")
        
        # Grid data: -1=Unknown, 0=Free, 100=Occupied
        raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        
        # Color mapping: Occupied -> Yellow/White, Free -> Dark, Unknown -> Mid-gray
        # Matplotlib visualization
        try:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(10, 8), dpi=200, facecolor='#0d1117')
            ax.set_facecolor('#0d1117')
            
            extent = [origin_x, origin_x + w * res, origin_y, origin_y + h * res]
            display_grid = np.zeros((h, w), dtype=np.float32)
            display_grid[raw_data == -1] = 0.5   # Unknown
            display_grid[raw_data == 0] = 0.0    # Free
            display_grid[raw_data > 50] = 1.0    # Occupied
            
            im = ax.imshow(display_grid, origin='lower', extent=extent, cmap='viridis', vmin=0.0, vmax=1.0)
            cbar = plt.colorbar(im, ax=ax, fraction=0.035, pad=0.04)
            cbar.set_label('Live Google Cartographer SLAM Occupancy', color='#c9d1d9', fontsize=9)
            cbar.ax.yaxis.set_tick_params(color='#8b949e')
            plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='#8b949e', fontsize=8)
            
            ax.set_title("Google Cartographer 2D SLAM - Live Webots Sensor Feed", color='white', fontsize=12, pad=12, fontweight='bold')
            ax.set_xlabel("X Position (meters)", color='#c9d1d9', fontsize=10)
            ax.set_ylabel("Y Position (meters)", color='#c9d1d9', fontsize=10)
            ax.tick_params(colors='#8b949e', labelsize=8)
            
            for spine in ax.spines.values():
                spine.set_color('#30363d')
                
            plt.tight_layout()
            plt.savefig(self.out_path, dpi=200)
            plt.close()
            self.get_logger().info(f"Saved pure Cartographer live map image to {self.out_path}")

            # Export standard ROS2 Cartographer PGM and YAML map files
            base_dir = os.path.dirname(self.out_path)
            # PGM encoding: 0 (black)=Occupied Wall, 254 (white)=Free, 205 (gray)=Unknown
            pgm_grid = np.full((h, w), 205, dtype=np.uint8)
            pgm_grid[raw_data == 0] = 254
            pgm_grid[raw_data > 50] = 0
            pgm_grid = np.flipud(pgm_grid)

            # Save in sweep results directory
            pgm_path = os.path.join(base_dir, "cartographer_2d_slam_static_map.pgm")
            with open(pgm_path, "wb") as f_pgm:
                f_pgm.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
                f_pgm.write(pgm_grid.tobytes())

            yaml_path = os.path.join(base_dir, "cartographer_2d_slam_static_map.yaml")
            yaml_content = f"""image: cartographer_2d_slam_static_map.pgm
mode: trinary
resolution: {res}
origin: [{origin_x:.4f}, {origin_y:.4f}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
            with open(yaml_path, "w", encoding="utf-8") as f_yaml:
                f_yaml.write(yaml_content)

            # Also update canonical map in maps/cyan_champion_clean_static/
            clean_dir = "/mnt/c/Users/nurmi/Documents/Da Winna/maps/cyan_champion_clean_static"
            try:
                os.makedirs(clean_dir, exist_ok=True)
                with open(os.path.join(clean_dir, "cartographer_2d_slam_static_map.pgm"), "wb") as f_clean:
                    f_clean.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
                    f_clean.write(pgm_grid.tobytes())
                with open(os.path.join(clean_dir, "cartographer_2d_slam_static_map.yaml"), "w", encoding="utf-8") as f_clean_yaml:
                    f_clean_yaml.write(yaml_content)
                self.get_logger().info(f"Updated canonical Cartographer SLAM map in {clean_dir}")
            except Exception as ex:
                self.get_logger().warn(f"Could not copy to canonical dir: {ex}")

            self.done = True
        except Exception as e:
            self.get_logger().error(f"Error rendering map PNG: {e}")

def main():
    rclpy.init()
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/mnt/c/Users/nurmi/Documents/Da Winna/maps/cyan_champion_clean_static/cartographer_2d_slam_static_result.png"
    node = MapSaverPNG(out_path)
    
    print("[MAP RENDERER] Waiting for live /map from Cartographer (up to 25s)...")
    for _ in range(250):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.done:
            break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
