#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 10 CAD TRACK MODELS WITH PROTRUDING WALL BOXES & CENTERLINES
================================================================================
Generates pure vector CAD SVGs and high-resolution PNG models for all 10
competition tracks equipped with:
  1. Outer & Inner CAD Ground Truth Wall Boundaries
  2. Ground Truth CAD Centerline (Perpendicular Medial Axis)
  3. Wall-Protruding Rectangular Choke Boxes (with exact millimeter dimensions)
  4. Freestanding Obstacle Pillars
================================================================================
"""

import os
import sys
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import build_raw_track_corridor

OUT_CAD_DIR = os.path.join(BASE_DIR, "racing_results", "cad_models")
os.makedirs(OUT_CAD_DIR, exist_ok=True)
ART_DIR = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"

# ==============================================================================
# DEFINITIONS OF ALL 10 TRACKS WITH WALL-PROTRUDING BOXES & PILLARS
# ==============================================================================

def get_cad_track_1():
    seq = ['S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 1: Alternating Wall Choke Speedway (16 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Main Straight Left Box (350x250mm)", "side": "left", "center_x": 1.40, "center_y": 0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "Main Straight Right Box (350x250mm)", "side": "right", "center_x": 2.80, "center_y": -0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Turn 2 Apex Pillar (Ø220mm)", "cx": 4.00, "cy": 1.00, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Backstretch Left Box (350x250mm)", "side": "left", "center_x": 2.80, "center_y": 1.625, "width": 0.35, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "Backstretch Right Box (350x250mm)", "side": "right", "center_x": 1.40, "center_y": 2.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Return Stretch Pillar (Ø220mm)", "cx": 0.00, "cy": 1.00, "radius": 0.11}
    ]
    return track

def get_cad_track_2():
    seq = ['S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 2: Corner Entry Bottleneck Arena (16 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Turn 1 Entry Left Box (400x250mm)", "side": "left", "center_x": 3.60, "center_y": 0.375, "width": 0.40, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "Turn 2 Entry Right Box (250x350mm)", "side": "right", "center_x": 4.375, "center_y": 1.50, "width": 0.25, "extrusion": 0.35},
        {"type": "circle", "name": "Hairpin Pillar 1 (Ø220mm)", "cx": 3.00, "cy": 2.00, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Turn 3 Entry Left Box (400x250mm)", "side": "left", "center_x": 0.60, "center_y": 1.625, "width": 0.40, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "Turn 4 Exit Right Box (250x350mm)", "side": "right", "center_x": -0.375, "center_y": 0.60, "width": 0.25, "extrusion": 0.35},
        {"type": "circle", "name": "Main Straight Pillar (Ø220mm)", "cx": 1.60, "cy": 0.00, "radius": 0.11}
    ]
    return track

def get_cad_track_3():
    seq = ['S', 'S', 'S', 'S', 'S', 'L', 'S', 'L', 'S', 'L', 'R', 'S', 'S', 'S', 'S', 'L', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 3: Dual-Chicane Squeeze Arena (18 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Start Straight Right Box (400x250mm)", "side": "right", "center_x": 1.60, "center_y": -0.375, "width": 0.40, "extrusion": 0.25},
        {"type": "circle", "name": "Speedway Pillar 1 (Ø220mm)", "cx": 3.00, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Speedway Pillar 2 (Ø220mm)", "cx": 4.40, "cy": -0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Mid-Chicane Left Box (350x260mm)", "side": "left", "center_x": 4.625, "center_y": 1.80, "width": 0.25, "extrusion": 0.35},
        {"type": "wall_rectangle", "name": "Chicane Exit Right Box (350x250mm)", "side": "right", "center_x": 3.20, "center_y": 3.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Return Stretch Pillar (Ø220mm)", "cx": 1.60, "cy": 1.80, "radius": 0.11}
    ]
    return track

def get_cad_track_4():
    seq = ['S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'R', 'L', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 4: Technical Switchback Choke Arena (14 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Turn 1 Entry Left Box (400x250mm)", "side": "left", "center_x": 1.80, "center_y": 0.375, "width": 0.40, "extrusion": 0.25},
        {"type": "circle", "name": "Mid-Corridor Pillar (Ø250mm)", "cx": 4.20, "cy": 1.80, "radius": 0.125},
        {"type": "wall_rectangle", "name": "Acute Switchback Right Box (350x250mm)", "side": "right", "center_x": 2.80, "center_y": 3.375, "width": 0.35, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "Chicane Transition Left Box (350x240mm)", "side": "left", "center_x": 1.375, "center_y": 2.00, "width": 0.25, "extrusion": 0.35}
    ]
    return track

def get_cad_track_5():
    seq = ['S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 5: Quad-Choke Narrowing Ring (16 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "South Straight Left Box (350x250mm)", "side": "left", "center_x": 1.40, "center_y": 0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "East Straight Right Box (250x350mm)", "side": "right", "center_x": 4.375, "center_y": 1.00, "width": 0.25, "extrusion": 0.35},
        {"type": "circle", "name": "Central Hairpin Pillar (Ø220mm)", "cx": 2.80, "cy": 2.00, "radius": 0.11},
        {"type": "wall_rectangle", "name": "North Straight Left Box (350x250mm)", "side": "left", "center_x": 1.60, "center_y": 1.625, "width": 0.35, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "West Straight Right Box (250x350mm)", "side": "right", "center_x": -0.375, "center_y": 1.00, "width": 0.25, "extrusion": 0.35}
    ]
    return track

def get_cad_track_6():
    seq = ['S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 6: High-Speed Slalom with Staggered Boxes (18 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Staggered Left Box A (350x250mm)", "side": "left", "center_x": 1.40, "center_y": 0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Slalom Pillar 1 (Ø220mm)", "cx": 2.80, "cy": -0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Staggered Right Box B (350x250mm)", "side": "right", "center_x": 4.20, "center_y": -0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "Turn 2 Entry Left Box (250x350mm)", "side": "left", "center_x": 5.375, "center_y": 1.40, "width": 0.25, "extrusion": 0.35},
        {"type": "circle", "name": "Return Stretch Pillar 1 (Ø220mm)", "cx": 3.80, "cy": 2.00, "radius": 0.11},
        {"type": "circle", "name": "Return Stretch Pillar 2 (Ø220mm)", "cx": 1.80, "cy": 2.00, "radius": 0.11}
    ]
    return track

def get_cad_track_7():
    seq = ['S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'L', 'S', 'L', 'R', 'S', 'S', 'S', 'S', 'L', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 7: Slalom Speedway with Triple Wall Chokes (18 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Slalom Entry Left Box (350x250mm)", "side": "left", "center_x": 1.20, "center_y": 0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Slalom Pillar 1 (Ø220mm)", "cx": 2.60, "cy": -0.16, "radius": 0.11},
        {"type": "circle", "name": "Slalom Pillar 2 (Ø220mm)", "cx": 4.20, "cy": 0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Mid-Straight Left Box (350x250mm)", "side": "left", "center_x": 5.40, "center_y": 0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Slalom Pillar 3 (Ø220mm)", "cx": 6.00, "cy": 1.60, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Exit Straight Right Box (350x250mm)", "side": "right", "center_x": 3.60, "center_y": 3.375, "width": 0.35, "extrusion": 0.25}
    ]
    return track

def get_cad_track_8():
    seq = ['S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 8: Dual-Chicane Mega Arena with Opposing Pinch Gates (20 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Pinch Left Gate Box (300x220mm)", "side": "left", "center_x": 1.40, "center_y": 0.39, "width": 0.30, "extrusion": 0.22},
        {"type": "wall_rectangle", "name": "Pinch Right Gate Box (300x220mm)", "side": "right", "center_x": 1.40, "center_y": -0.39, "width": 0.30, "extrusion": 0.22},
        {"type": "circle", "name": "Pillar 1 North (Ø220mm)", "cx": 3.20, "cy": 0.16, "radius": 0.11},
        {"type": "circle", "name": "Pillar 2 South (Ø220mm)", "cx": 5.00, "cy": -0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Turn 2 Entry Left Box (250x350mm)", "side": "left", "center_x": 6.375, "center_y": 1.40, "width": 0.25, "extrusion": 0.35},
        {"type": "circle", "name": "Return Stretch Pillar (Ø220mm)", "cx": 3.80, "cy": 2.00, "radius": 0.11}
    ]
    return track

def get_cad_track_9():
    seq = ['S', 'S', 'S', 'S', 'L', 'S', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 9: Double Switchback Pinch Speedway (18 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "South Straight Left Box (350x250mm)", "side": "left", "center_x": 1.60, "center_y": 0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Pillar 1 (Ø220mm)", "cx": 3.40, "cy": -0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "East Turn 1 Right Box (250x350mm)", "side": "right", "center_x": 4.375, "center_y": 1.80, "width": 0.25, "extrusion": 0.35},
        {"type": "wall_rectangle", "name": "Backstretch Left Box (400x250mm)", "side": "left", "center_x": 2.60, "center_y": 2.625, "width": 0.40, "extrusion": 0.25},
        {"type": "circle", "name": "Return Pillar (Ø220mm)", "cx": 1.40, "cy": 3.00, "radius": 0.11},
        {"type": "wall_rectangle", "name": "West Turn 2 Right Box (250x350mm)", "side": "right", "center_x": -0.375, "center_y": 1.80, "width": 0.25, "extrusion": 0.35}
    ]
    return track

def get_cad_track_10():
    seq = ['S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
    track = build_raw_track_corridor(seq)
    track["name"] = "Track 10: Multi-Choke Obstacle Maze (20 Tiles)"
    track["obstacles"] = [
        {"type": "wall_rectangle", "name": "Start Straight Right Box (350x250mm)", "side": "right", "center_x": 1.40, "center_y": -0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Corridor Pillar 1 (Ø220mm)", "cx": 3.00, "cy": 0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Mid-Straight Left Box (350x250mm)", "side": "left", "center_x": 4.60, "center_y": 0.375, "width": 0.35, "extrusion": 0.25},
        {"type": "circle", "name": "Corridor Pillar 2 (Ø220mm)", "cx": 5.80, "cy": -0.16, "radius": 0.11},
        {"type": "wall_rectangle", "name": "Turn 2 Entry Left Box (250x350mm)", "side": "left", "center_x": 6.375, "center_y": 1.40, "width": 0.25, "extrusion": 0.35},
        {"type": "wall_rectangle", "name": "Backstretch Right Box (400x250mm)", "side": "right", "center_x": 4.40, "center_y": 2.375, "width": 0.40, "extrusion": 0.25},
        {"type": "wall_rectangle", "name": "Return Stretch Left Box (350x250mm)", "side": "left", "center_x": 2.00, "center_y": 1.625, "width": 0.35, "extrusion": 0.25}
    ]
    return track

# ==============================================================================
# PURE CAD VECTOR SVG EXPORTER
# ==============================================================================
def export_cad_svg_model(filename, track_data, track_idx):
    l_wall = np.array(track_data["left_wall"])
    r_wall = np.array(track_data["right_wall"])
    cl = np.array(track_data["centerline"])
    obstacles = track_data.get("obstacles", [])
    name = track_data["name"]
    
    all_pts = np.vstack([l_wall, r_wall, cl])
    min_x, min_y = np.min(all_pts, axis=0) - 0.8
    max_x, max_y = np.max(all_pts, axis=0) + 0.8
    scale = 120.0  # 120 pixels per meter (high CAD fidelity)
    
    svg_w = int((max_x - min_x) * scale)
    svg_h = int((max_y - min_y) * scale)
    
    def tx(x): return (x - min_x) * scale
    def ty(y): return svg_h - (y - min_y) * scale
    
    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" width="{svg_w}" height="{svg_h}" style="background:#0a0f1d; font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif;">')
    
    # 1. Background Technical Grid (1.0m major, 0.2m minor)
    svg.append('<defs>')
    svg.append('  <pattern id="grid_minor" width="24" height="24" patternUnits="userSpaceOnUse">')
    svg.append('    <path d="M 24 0 L 0 0 0 24" fill="none" stroke="#1e293b" stroke-width="0.75" stroke-opacity="0.4"/>')
    svg.append('  </pattern>')
    svg.append('  <pattern id="grid_major" width="120" height="120" patternUnits="userSpaceOnUse">')
    svg.append('    <rect width="120" height="120" fill="url(#grid_minor)"/>')
    svg.append('    <path d="M 120 0 L 0 0 0 120" fill="none" stroke="#334155" stroke-width="1.2" stroke-opacity="0.7"/>')
    svg.append('  </pattern>')
    svg.append('</defs>')
    svg.append(f'<rect width="{svg_w}" height="{svg_h}" fill="url(#grid_major)" />')
    
    # Title Header
    svg.append(f'<text x="32" y="44" fill="#f8fafc" font-size="22" font-weight="700" letter-spacing="0.5">{name}</text>')
    svg.append(f'<text x="32" y="70" fill="#94a3b8" font-size="13">CAD Vector Model: Outer/Inner Walls, Mathematical Centerline &amp; Obstacles</text>')
    
    # 2. Drivable Corridor Fill
    poly_l = " ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in l_wall])
    poly_r = " ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in r_wall[::-1]])
    svg.append(f'<polygon points="{poly_l} {poly_r}" fill="#0f172a" opacity="0.9"/>')
    
    # 3. Ground Truth CAD Centerline (Golden)
    cl_path = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cl]) + " Z"
    svg.append(f'<path d="{cl_path}" fill="none" stroke="#ffd700" stroke-width="3.5" stroke-dasharray="8,6" opacity="0.95"/>')
    
    # 4. Outer Wall Boundary (Cyan) & Inner Wall Boundary (Rose)
    l_path = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in l_wall]) + " Z"
    r_path = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in r_wall]) + " Z"
    svg.append(f'<path d="{l_path}" fill="none" stroke="#38bdf8" stroke-width="3.0" opacity="0.95"/>')
    svg.append(f'<path d="{r_path}" fill="none" stroke="#f43f5e" stroke-width="3.0" opacity="0.95"/>')
    
    # 5. Wall-Protruding Rectangular Boxes & Freestanding Pillars (Amber)
    svg.append('<g id="obstacles">')
    for obs in obstacles:
        if obs["type"] == "circle":
            cx, cy, r = tx(obs["cx"]), ty(obs["cy"]), obs["radius"] * scale
            svg.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="#fbbf24" stroke="#ffffff" stroke-width="2.5" opacity="0.95"/>')
            # Label
            svg.append(f'<text x="{cx+r+6:.1f}" y="{cy+4:.1f}" fill="#fbbf24" font-size="11" font-weight="600">{obs["name"]}</text>')
        elif obs["type"] == "wall_rectangle":
            cx, cy = tx(obs["center_x"]), ty(obs["center_y"])
            w_px = obs["width"] * scale
            h_px = obs["extrusion"] * scale
            x0 = cx - w_px / 2.0
            y0 = cy - h_px / 2.0
            svg.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{w_px:.1f}" height="{h_px:.1f}" fill="#fbbf24" stroke="#ffffff" stroke-width="2.5" opacity="0.95" rx="3"/>')
            # Label
            svg.append(f'<text x="{x0:.1f}" y="{y0-6:.1f}" fill="#fbbf24" font-size="11" font-weight="600">{obs["name"]}</text>')
    svg.append('</g>')
    
    # 6. Technical Legend
    leg_w, leg_h = 460, 140
    leg_x, leg_y = svg_w - leg_w - 32, svg_h - leg_h - 32
    svg.append(f'<rect x="{leg_x}" y="{leg_y}" width="{leg_w}" height="{leg_h}" fill="#111827" rx="10" stroke="#374151" stroke-width="1.5" opacity="0.95"/>')
    
    # Legend Items
    svg.append(f'<line x1="{leg_x+24}" y1="{leg_y+30}" x2="{leg_x+64}" y2="{leg_y+30}" stroke="#ffd700" stroke-width="3.5" stroke-dasharray="8,6"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+35}" fill="#ffd700" font-size="13" font-weight="600">CAD Ground Truth Centerline</text>')
    
    svg.append(f'<line x1="{leg_x+24}" y1="{leg_y+60}" x2="{leg_x+64}" y2="{leg_y+60}" stroke="#38bdf8" stroke-width="3"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+65}" fill="#38bdf8" font-size="13" font-weight="600">CAD Left Wall Boundary (Outer/Inner)</text>')
    
    svg.append(f'<line x1="{leg_x+24}" y1="{leg_y+90}" x2="{leg_x+64}" y2="{leg_y+90}" stroke="#f43f5e" stroke-width="3"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+95}" fill="#f43f5e" font-size="13" font-weight="600">CAD Right Wall Boundary (Outer/Inner)</text>')
    
    svg.append(f'<rect x="{leg_x+24}" y="{leg_y+110}" width="40" height="15" fill="#fbbf24" stroke="#ffffff" stroke-width="1.5" rx="2"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+123}" fill="#fbbf24" font-size="13" font-weight="600">Wall-Protruding Boxes &amp; Obstacles (Amber)</text>')
    
    svg.append('</svg>')
    
    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))

# ==============================================================================
# HIGH-RESOLUTION PNG RENDERER
# ==============================================================================
def export_cad_png_render(filename, track_data, track_idx):
    l_wall = np.array(track_data["left_wall"])
    r_wall = np.array(track_data["right_wall"])
    cl = np.array(track_data["centerline"])
    obstacles = track_data.get("obstacles", [])
    name = track_data["name"]
    
    fig, ax = plt.subplots(figsize=(16, 12), facecolor="#0a0f1d")
    ax.set_facecolor("#0a0f1d")
    
    # Close loops for plotting
    lw = np.vstack([l_wall, l_wall[:1]])
    rw = np.vstack([r_wall, r_wall[:1]])
    cw = np.vstack([cl, cl[:1]])
    
    # Corridor polygon
    poly_pts = np.vstack([l_wall, r_wall[::-1]])
    patch = plt.Polygon(poly_pts, facecolor="#111827", edgecolor="none", alpha=0.9, zorder=1)
    ax.add_patch(patch)
    
    # Walls
    ax.plot(lw[:, 0], lw[:, 1], color="#38bdf8", linewidth=2.8, label="CAD Left Wall Boundary", zorder=3)
    ax.plot(rw[:, 0], rw[:, 1], color="#f43f5e", linewidth=2.8, label="CAD Right Wall Boundary", zorder=3)
    
    # Centerline
    ax.plot(cw[:, 0], cw[:, 1], color="#ffd700", linewidth=3.2, linestyle="--", label="CAD Ground Truth Centerline", zorder=4)
    
    # Obstacles
    wall_boxes = [o for o in obstacles if o["type"] == "wall_rectangle"]
    pillars = [o for o in obstacles if o["type"] == "circle"]
    
    for obs in obstacles:
        if obs["type"] == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], facecolor="#fbbf24", edgecolor="#ffffff", linewidth=1.5, zorder=6)
            ax.add_patch(circle)
            ax.text(obs["cx"] + obs["radius"] + 0.08, obs["cy"], obs["name"], color="#fbbf24", fontsize=8.5, fontweight='bold', va='center', zorder=7)
        elif obs["type"] == "wall_rectangle":
            cx, cy = obs["center_x"], obs["center_y"]
            w, h = obs["width"], obs["extrusion"]
            rect = plt.Rectangle((cx - w/2.0, cy - h/2.0), w, h, facecolor="#fbbf24", edgecolor="#ffffff", linewidth=1.5, zorder=6)
            ax.add_patch(rect)
            ax.text(cx, cy + h/2.0 + 0.08, obs["name"], color="#fbbf24", fontsize=8.5, fontweight='bold', ha='center', zorder=7)
            
    ax.plot([], [], color="#fbbf24", linewidth=5, label=f"Obstacles ({len(wall_boxes)} Wall Boxes + {len(pillars)} Pillars)")
    
    ax.set_title(f"{name}\nExact Ground Truth CAD Vector Model with Centerline",
                 color="#f8fafc", fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.25, color="#475569")
    ax.tick_params(colors="#94a3b8", labelsize=10)
    ax.set_xlabel("X Position (meters)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.set_ylabel("Y Position (meters)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.legend(loc="upper left", facecolor="#111827", edgecolor="#374151", labelcolor="#f8fafc", fontsize=10, borderpad=1.0, labelspacing=0.8)
    
    plt.savefig(filename, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

# ==============================================================================
# MAIN EXPORT RUNNER
# ==============================================================================
def main():
    print("=" * 90)
    print("   EXPORTING 10 CAD TRACK SVG MODELS & GROUND TRUTH CENTERLINES")
    print("=" * 90)
    
    tracks = [
        (get_cad_track_1, 1),
        (get_cad_track_2, 2),
        (get_cad_track_3, 3),
        (get_cad_track_4, 4),
        (get_cad_track_5, 5),
        (get_cad_track_6, 6),
        (get_cad_track_7, 7),
        (get_cad_track_8, 8),
        (get_cad_track_9, 9),
        (get_cad_track_10, 10),
    ]
    
    summary = []
    for getter, idx in tracks:
        track = getter()
        name = track["name"]
        
        # 1. Export SVG
        svg_name = f"track{idx:02d}_cad_with_centerline.svg"
        svg_path = os.path.join(OUT_CAD_DIR, svg_name)
        export_cad_svg_model(svg_path, track, idx)
        shutil.copy2(svg_path, os.path.join(ART_DIR, svg_name))
        
        # 2. Export PNG
        png_name = f"track{idx:02d}_cad_with_centerline.png"
        png_path = os.path.join(OUT_CAD_DIR, png_name)
        export_cad_png_render(png_path, track, idx)
        shutil.copy2(png_path, os.path.join(ART_DIR, png_name))
        
        w_boxes = len([o for o in track.get("obstacles", []) if o["type"] == "wall_rectangle"])
        pillars = len([o for o in track.get("obstacles", []) if o["type"] == "circle"])
        
        print(f"  [OK] Track {idx:02d}: {name:<48} | {w_boxes} Wall Boxes | {pillars} Pillars -> {svg_name}")
        summary.append((idx, name, w_boxes, pillars, svg_name, png_name))
        
    print("\n" + "=" * 90)
    print("   ALL 10 CAD TRACK MODELS GENERATED & EXPORTED SUCCESSFULLY")
    print("=" * 90)

if __name__ == "__main__":
    main()
