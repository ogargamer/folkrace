"""
Raw Boundary Track Generator with Smart Obstacle Verification
Tracks are defined purely by outer/inner barrier walls and verified inside-corridor obstacles.
"""

import numpy as np

TILE_SIZE = 1.2    # 1.2m
TRACK_WIDTH = 1.0  # 1.0m

DIR_VECTORS = [
    np.array([1.0, 0.0]),   # 0: East (+X)
    np.array([0.0, 1.0]),   # 1: North (+Y)
    np.array([-1.0, 0.0]),  # 2: West (-X)
    np.array([0.0, -1.0])   # 3: South (-Y)
]

LEFT_VECTORS = [
    np.array([0.0, 1.0]),   # 0: North
    np.array([-1.0, 0.0]),  # 1: West
    np.array([0.0, -1.0]),  # 2: South
    np.array([1.0, 0.0])    # 3: East
]

def build_raw_track_corridor(blocks_sequence, tile_size=TILE_SIZE, track_width=TRACK_WIDTH):
    """
    Builds the left and right barrier wall point chains for modular blocks.
    No centerline is included — centerline is calculated dynamically via Voronoi.
    """
    cur_x = 0.0
    cur_y = 0.0
    cur_h = 0

    left_wall = []
    right_wall = []
    
    half_width = track_width / 2.0
    steps_straight = 30
    steps_curve = 30

    for cmd in blocks_sequence:
        fwd = DIR_VECTORS[cur_h]
        left = LEFT_VECTORS[cur_h]

        if cmd == 'S':
            for i in range(steps_straight):
                t = (i + 1) / steps_straight
                px = cur_x + fwd[0] * tile_size * t
                py = cur_y + fwd[1] * tile_size * t
                left_wall.append([px + left[0] * half_width, py + left[1] * half_width])
                right_wall.append([px - left[0] * half_width, py - left[1] * half_width])

            cur_x += fwd[0] * tile_size
            cur_y += fwd[1] * tile_size

        elif cmd == 'L':
            r_center = tile_size / 2.0
            center = np.array([cur_x + left[0] * r_center, cur_y + left[1] * r_center])
            start_angle = np.arctan2(cur_y - center[1], cur_x - center[0])
            
            for i in range(steps_curve):
                angle = start_angle + (np.pi / 2.0) * ((i + 1) / steps_curve)
                lx = center[0] + (r_center - half_width) * np.cos(angle)
                ly = center[1] + (r_center - half_width) * np.sin(angle)
                rx = center[0] + (r_center + half_width) * np.cos(angle)
                ry = center[1] + (r_center + half_width) * np.sin(angle)
                left_wall.append([lx, ly])
                right_wall.append([rx, ry])

            cur_x = center[0] + r_center * np.cos(start_angle + np.pi / 2.0)
            cur_y = center[1] + r_center * np.sin(start_angle + np.pi / 2.0)
            cur_h = (cur_h + 1) % 4

        elif cmd == 'R':
            r_center = tile_size / 2.0
            right = -left
            center = np.array([cur_x + right[0] * r_center, cur_y + right[1] * r_center])
            start_angle = np.arctan2(cur_y - center[1], cur_x - center[0])
            
            for i in range(steps_curve):
                angle = start_angle - (np.pi / 2.0) * ((i + 1) / steps_curve)
                lx = center[0] + (r_center + half_width) * np.cos(angle)
                ly = center[1] + (r_center + half_width) * np.sin(angle)
                rx = center[0] + (r_center - half_width) * np.cos(angle)
                ry = center[1] + (r_center - half_width) * np.sin(angle)
                left_wall.append([lx, ly])
                right_wall.append([rx, ry])

            cur_x = center[0] + r_center * np.cos(start_angle - np.pi / 2.0)
            cur_y = center[1] + r_center * np.sin(start_angle - np.pi / 2.0)
            cur_h = (cur_h + 3) % 4

    return {
        "left_wall": np.array(left_wall),
        "right_wall": np.array(right_wall),
        "start_pos": np.array([0.0, 0.0]),
        "track_width": track_width,
        "tile_size": tile_size,
        "blocks": blocks_sequence
    }

def validate_track_obstacles(track_data):
    """
    Strict Geometric Validator: Verifies that every single obstacle (including rectangle corners
    and circular perimeter points) lies 100% within the physical track width corridor.
    """
    l_wall = track_data['left_wall']
    r_wall = track_data['right_wall']
    mid = (l_wall + r_wall) / 2.0
    half_w = track_data.get('track_width', 1.0) / 2.0
    
    obstacles = track_data.get('obstacles', [])
    valid_obs = []
    
    for obs in obstacles:
        is_valid = True
        test_points = []
        
        if obs['type'] == 'circle':
            cx, cy, r = obs['cx'], obs['cy'], obs['radius']
            test_points.append([cx, cy])
            for a in np.linspace(0, 2*np.pi, 12, endpoint=False):
                test_points.append([cx + r*np.cos(a), cy + r*np.sin(a)])
        elif obs['type'] == 'wall_rectangle':
            cx, cy = obs['center_x'], obs['center_y']
            w, h = obs['width'], obs['extrusion']
            test_points.append([cx - w/2, cy - h/2])
            test_points.append([cx + w/2, cy - h/2])
            test_points.append([cx + w/2, cy + h/2])
            test_points.append([cx - w/2, cy + h/2])
            
        test_points = np.array(test_points)
        for pt in test_points:
            dists = np.hypot(mid[:, 0] - pt[0], mid[:, 1] - pt[1])
            closest_idx = np.argmin(dists)
            d_to_center = dists[closest_idx]
            if d_to_center > (half_w + 0.03):
                is_valid = False
                break
                
        if is_valid:
            valid_obs.append(obs)
            
    track_data['obstacles'] = valid_obs
    return track_data

# ==============================================================================
#  TRACK 1: NEW High-Speed Chicane Arena (16 Pieces, 6 Obstacles)
# ==============================================================================
def get_track_1_chicane_with_pillar():
    seq = ['S', 'S', 'S', 'S', 'S', 'L', 'S', 'L', 'S', 'L', 'R', 'S', 'S', 'S', 'L', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 1: High-Speed Chicane Arena (16 Tiles, 6 Obstacles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Start Straight Right Choke (250mm)", "side": "right", "center_x": 1.40, "center_y": -0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Straight Slalom Pillar 1 (Ø220mm)", "cx": 2.80, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Straight Slalom Pillar 2 (Ø220mm)", "cx": 4.20, "cy": -0.16, "radius": 0.11},
        {"type": "circle", "name": "Vertical Turn Pillar (Ø220mm)", "cx": 5.40, "cy": 1.80, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Backstretch Left Choke (220mm)", "side": "left", "center_x": 3.20, "center_y": 3.22, "width": 0.35, "extrusion": 0.22},
        {"type": "circle", "name": "Chicane Exit Apex Pillar (Ø220mm)", "cx": 1.40, "cy": 1.80, "radius": 0.11}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 2: NEW Double Hairpin & Slalom (16 Pieces, 6 Obstacles)
# ==============================================================================
def get_track_2_bottleneck_circuit():
    seq = ['S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 2: Double Hairpin & Slalom (16 Tiles, 6 Obstacles)"
    track["obstacles"] = [
        {"type": "circle", "name": "Pillar 1 North Offset (Ø220mm)", "cx": 1.40, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Pillar 2 South Offset (Ø220mm)", "cx": 2.80, "cy": -0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Main Straight Choke (240mm)", "side": "left", "center_x": 3.80, "center_y": 0.37, "width": 0.35, "extrusion": 0.24},
        {"type": "wall_rectangle", "name": "Hairpin Entry Left Choke (240mm)", "side": "left", "center_x": 4.20, "center_y": 2.00, "width": 0.35, "extrusion": 0.24},
        {"type": "circle", "name": "Return Corridor Pillar 1 (Ø220mm)", "cx": 3.20, "cy": 3.60, "radius": 0.11},
        {"type": "circle", "name": "Return Corridor Pillar 2 (Ø220mm)", "cx": 1.60, "cy": 3.60, "radius": 0.11}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 3: NEW Grand Prix Arena (18 Pieces, 7 Obstacles)
# ==============================================================================
def get_track_3_slalom_pillars():
    seq = ['S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'L', 'S', 'L', 'R', 'S', 'S', 'S', 'S', 'L', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 3: Grand Prix Arena (18 Tiles, 7 Obstacles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Main Straight Right Choke (250mm)", "side": "right", "center_x": 1.20, "center_y": -0.37, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Speedway Offset Pillar 1 (Ø220mm)", "cx": 2.60, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Speedway Offset Pillar 2 (Ø220mm)", "cx": 4.20, "cy": -0.16, "radius": 0.11},
        {"type": "circle", "name": "Speedway Offset Pillar 3 (Ø220mm)", "cx": 5.60, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Vertical Turn Apex Pillar (Ø220mm)", "cx": 6.60, "cy": 1.80, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Exit Straight Left Choke (240mm)", "side": "left", "center_x": 4.20, "center_y": 3.22, "width": 0.35, "extrusion": 0.24},
        {"type": "circle", "name": "Chicane Mid Pillar (Ø220mm)", "cx": 1.80, "cy": 1.80, "radius": 0.11}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 4: KEPT EXACTLY AS USER REQUESTED (Technical Switchback Arena, 14 Pieces)
# ==============================================================================
def get_track_4_switchback_arena():
    seq = ['S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'R', 'L', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 4: Technical Switchback Arena (14 Pieces)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Corner Entry Choke (250mm)", "side": "left", "center_x": 1.80, "center_y": 0.375, "width": 0.40, "extrusion": 0.25},
        {"type": "circle", "name": "Mid-Corridor Pillar (Ø250mm)", "cx": 4.20, "cy": 1.80, "radius": 0.125}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 5: NEW Bottleneck Ring (16 Pieces, 5 Obstacles)
# ==============================================================================
def get_track_5_dual_choke_chicane_ring():
    seq = ['S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 5: Quad-Choke Bottleneck Ring (16 Tiles, 5 Obstacles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Left Wall Choke 1 (250mm)", "side": "left", "center_x": 1.20, "center_y": 0.37, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "North Slalom Pillar (Ø220mm)", "cx": 2.40, "cy": 0.18, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Right Wall Choke 2 (250mm)", "side": "right", "center_x": 3.60, "center_y": -0.37, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Hairpin Center Pillar (Ø220mm)", "cx": 4.20, "cy": 2.00, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Return Straight Left Choke (240mm)", "side": "left", "center_x": 2.80, "center_y": 3.60 - 0.37, "width": 0.35, "extrusion": 0.24}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 6: NEW Multi-Apex Circuit (18 Pieces, 6 Obstacles)
# ==============================================================================
def get_track_6_quad_pillar_loop():
    seq = ['S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 6: Multi-Apex High-Speed Circuit (18 Tiles, 6 Obstacles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Straight 1 Right Choke (250mm)", "side": "right", "center_x": 1.20, "center_y": -0.37, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Pillar 1 North (Ø220mm)", "cx": 2.60, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Pillar 2 South (Ø220mm)", "cx": 4.20, "cy": -0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Turn 2 Entry Left Choke (240mm)", "side": "left", "center_x": 5.40, "center_y": 2.00, "width": 0.35, "extrusion": 0.24},
        {"type": "circle", "name": "Return Stretch Pillar 1 (Ø220mm)", "cx": 4.20, "cy": 3.60, "radius": 0.11},
        {"type": "circle", "name": "Return Stretch Pillar 2 (Ø220mm)", "cx": 2.20, "cy": 3.60, "radius": 0.11}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 7: NEW Slalom Speedway (18 Pieces, 5 Obstacles)
# ==============================================================================
def get_track_7_multi_bottleneck_slalom():
    seq = ['S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'L', 'S', 'L', 'R', 'S', 'S', 'S', 'S', 'L', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 7: Slalom Speedway (18 Tiles, 5 Obstacles)"
    track["obstacles"] = [
        {"type": "circle", "name": "Slalom Pillar 1 (Ø220mm)", "cx": 1.20, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Slalom Pillar 2 (Ø220mm)", "cx": 2.80, "cy": -0.16, "radius": 0.11},
        {"type": "circle", "name": "Slalom Pillar 3 (Ø220mm)", "cx": 4.40, "cy": 0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Mid-Straight Left Choke (240mm)", "side": "left", "center_x": 5.60, "center_y": 0.37, "width": 0.35, "extrusion": 0.24},
        {"type": "wall_rectangle", "name": "Exit Choke (240mm)", "side": "left", "center_x": 3.60, "center_y": 3.22, "width": 0.35, "extrusion": 0.24}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 8: NEW Dual Chicane Speedway (20 Pieces, 6 Obstacles)
# ==============================================================================
def get_track_8_dual_chicane_arena():
    seq = ['S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 8: Dual Chicane Speedway (20 Tiles, 6 Obstacles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Straight 1 Right Choke (250mm)", "side": "right", "center_x": 1.20, "center_y": -0.37, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Pillar 1 North (Ø220mm)", "cx": 2.80, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Pillar 2 South (Ø220mm)", "cx": 4.60, "cy": -0.16, "radius": 0.11},
        {"type": "circle", "name": "Pillar 3 North (Ø220mm)", "cx": 6.20, "cy": 0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Turn 2 Left Choke (240mm)", "side": "left", "center_x": 6.60, "center_y": 2.20, "width": 0.35, "extrusion": 0.24},
        {"type": "circle", "name": "Return Stretch Pillar (Ø220mm)", "cx": 4.20, "cy": 3.60, "radius": 0.11}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 9: NEW Double Switchback Arena (18 Pieces, 5 Obstacles)
# ==============================================================================
def get_track_9_super_speedway():
    seq = ['S', 'S', 'S', 'S', 'L', 'S', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 9: Double Switchback Arena (18 Tiles, 5 Obstacles)"
    track["obstacles"] = [
        {"type": "circle", "name": "Slalom Pillar 1 (Ø220mm)", "cx": 1.40, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Slalom Pillar 2 (Ø220mm)", "cx": 3.20, "cy": -0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Backstretch Choke (250mm)", "side": "left", "center_x": 3.60, "center_y": 4.80 - 0.37, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Return Pillar (Ø220mm)", "cx": 1.80, "cy": 4.80, "radius": 0.11},
        {"type": "wall_rectangle", "name": "South Choke (240mm)", "side": "right", "center_x": -0.37, "center_y": 2.40, "width": 0.24, "extrusion": 0.35}
    ]
    return validate_track_obstacles(track)

# ==============================================================================
#  TRACK 10: NEW Obstacle Maze (20 Pieces, 6 Obstacles)
# ==============================================================================
def get_track_10_switchback_obstacle_maze():
    seq = ['S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 10: Obstacle Maze (20 Tiles, 6 Obstacles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Start Straight Right Choke (250mm)", "side": "right", "center_x": 1.40, "center_y": -0.37, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Corridor Pillar 1 (Ø220mm)", "cx": 3.00, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Corridor Pillar 2 (Ø220mm)", "cx": 4.80, "cy": -0.16, "radius": 0.11},
        {"type": "circle", "name": "Corridor Pillar 3 (Ø220mm)", "cx": 6.40, "cy": 0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Backstretch Choke (240mm)", "side": "left", "center_x": 4.80, "center_y": 3.60 - 0.37, "width": 0.35, "extrusion": 0.24},
        {"type": "circle", "name": "Return Stretch Pillar (Ø220mm)", "cx": 2.60, "cy": 3.60, "radius": 0.11}
    ]
    return validate_track_obstacles(track)

