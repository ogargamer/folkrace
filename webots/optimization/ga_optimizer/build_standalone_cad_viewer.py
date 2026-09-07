#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - STANDALONE ZERO-CORS CAD VIEWER GENERATOR
================================================================================
Embeds all SVG vector data directly into cad_viewer.html so it renders instantly
when opened via file:/// without being blocked by browser CORS security policies.
================================================================================
"""

import os
import sys
import json
import base64
import shutil

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SNAP_DIR = os.path.join(BASE_DIR, "racing_results", "snapshots")
BACKUP_DIR = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup")
CAD_HTML = os.path.join(BASE_DIR, "racing_results", "cad_viewer.html")
BACKUP_HTML = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "index.html")
ART_DIR = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"

views_meta = [
    ("track03_45deg_yellow_racing_line_alpha_hull.svg", "🟡 Track 3: Final Yellow Racing Line (Alpha-Shape Walls + Voronoi Seed)"),
    ("track04_45deg_yellow_racing_line_alpha_hull.svg", "🟡 Track 4: Final Yellow Racing Line (Alpha-Shape Walls + Voronoi Seed)"),
    ("track01_cad_with_centerline.svg", "🔥 Track 1: High-Speed Chicane (CAD Raycast CL)"),
    ("track02_cad_with_centerline.svg", "🔥 Track 2: Double Hairpin & Slalom (CAD Raycast CL)"),
    ("track03_cad_with_centerline.svg", "🔥 Track 3: Grand Prix Arena (CAD Raycast CL)"),
    ("track04_cad_with_centerline.svg", "🔥 Track 4: Technical Switchback (CAD Raycast CL)"),
    ("track05_cad_with_centerline.svg", "🔥 Track 5: Quad-Choke Ring (CAD Raycast CL)"),
    ("track06_cad_with_centerline.svg", "🔥 Track 6: Multi-Apex Circuit (CAD Raycast CL)"),
    ("track07_cad_with_centerline.svg", "🔥 Track 7: Slalom Speedway (CAD Raycast CL)"),
    ("track08_cad_with_centerline.svg", "🔥 Track 8: Dual-Chicane Mega Arena (CAD Raycast CL)"),
    ("track09_cad_with_centerline.svg", "🔥 Track 9: Double Switchback Arena (CAD Raycast CL)"),
    ("track10_cad_with_centerline.svg", "🔥 Track 10: Obstacle Maze (CAD Raycast CL)"),
    ("track7_physics_noisy_points.svg", "★ Track 7: Physics Noisy (Points ONLY)"),
    ("track4_physics_noisy_points.svg", "★ Track 4: Physics Noisy (Points ONLY)"),
    ("07_track_7_webots_deskewed_lidar_map.svg", "Track 7: Physics LiDAR + Overlay"),
    ("webots_noisy_lidar_map_and_yellow_line.svg", "Track 4: Physics LiDAR + Overlay")
]

CAD_MODELS_DIR = os.path.join(BASE_DIR, "racing_results", "cad_models")
SLAM_DIR = os.path.join(BASE_DIR, "racing_results", "cartographer_slam_2cm")

def main():
    print("Building standalone zero-CORS CAD Viewer...")
    
    svg_database = {}
    for filename, label in views_meta:
        fpath = os.path.join(SLAM_DIR, filename)
        if not os.path.exists(fpath):
            fpath = os.path.join(CAD_MODELS_DIR, filename)
        if not os.path.exists(fpath):
            fpath = os.path.join(SNAP_DIR, filename)
        if not os.path.exists(fpath):
            fpath = os.path.join(BACKUP_DIR, filename)
            
        if os.path.exists(fpath):
            # Also copy to snapshots for consistency
            snap_dest = os.path.join(SNAP_DIR, filename)
            if fpath != snap_dest and not os.path.exists(snap_dest):
                shutil.copy2(fpath, snap_dest)

            with open(fpath, "r", encoding="utf-8") as f:
                content = f.read()
                b64 = base64.b64encode(content.encode("utf-8")).decode("ascii")
                svg_database[filename] = {
                    "name": label,
                    "data": b64
                }
                print(f"  [OK] Inlined: {filename} ({len(content)/1024:.1f} KB)")
        else:
            print(f"  [WARN] File not found: {fpath}")

    db_json = json.dumps(svg_database)

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Da Winna - Autonomous Racing Trajectory & CAD Studio</title>
    <style>
        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            user-select: none;
        }}
        body {{
            background-color: #070a13;
            color: #e2e8f0;
            height: 100vh;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }}
        header {{
            background: #0d121f;
            border-bottom: 1px solid #1e293b;
            padding: 8px 16px;
            display: flex;
            align-items: center;
            justify-content: space-between;
            z-index: 10;
            gap: 12px;
        }}
        .title-group {{
            display: flex;
            align-items: center;
            gap: 10px;
        }}
        .logo-badge {{
            background: linear-gradient(135deg, #ffd700, #ff9900);
            color: #070a13;
            font-weight: 900;
            padding: 3px 8px;
            border-radius: 5px;
            font-size: 11px;
            letter-spacing: 1px;
        }}
        h1 {{
            font-size: 15px;
            font-weight: 700;
            color: #f8fafc;
            white-space: nowrap;
        }}
        .track-nav {{
            display: flex;
            gap: 6px;
            overflow-x: auto;
            max-width: 60%;
            padding: 2px 4px;
        }}
        .track-btn {{
            background: #1e293b;
            color: #94a3b8;
            border: 1px solid #334155;
            padding: 6px 12px;
            border-radius: 5px;
            font-size: 12px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.15s ease;
            white-space: nowrap;
        }}
        .track-btn:hover {{
            background: #334155;
            color: #f8fafc;
        }}
        .track-btn.active {{
            background: #ffd700;
            color: #070a13;
            border-color: #ffd700;
            font-weight: 700;
            box-shadow: 0 0 10px rgba(255, 215, 0, 0.4);
        }}
        .controls {{
            display: flex;
            align-items: center;
            gap: 6px;
        }}
        .ctrl-btn {{
            background: #1e293b;
            color: #f8fafc;
            border: 1px solid #334155;
            padding: 5px 10px;
            border-radius: 5px;
            font-size: 12px;
            font-weight: 600;
            cursor: pointer;
            transition: background 0.15s;
        }}
        .ctrl-btn:hover {{
            background: #334155;
        }}
        #viewport {{
            flex: 1;
            position: relative;
            background: #0b0f19;
            overflow: hidden;
            cursor: grab;
        }}
        #viewport:active {{
            cursor: grabbing;
        }}
        #svg-container {{
            width: 100%;
            height: 100%;
            display: flex;
            align-items: center;
            justify-content: center;
        }}
        #svg-canvas {{
            width: 100%;
            height: 100%;
            display: block;
        }}
        #svg-canvas * {{
            vector-effect: non-scaling-stroke !important;
        }}
        .hud-overlay {{
            position: absolute;
            bottom: 14px;
            left: 14px;
            background: rgba(13, 18, 31, 0.94);
            border: 1px solid #1e293b;
            padding: 10px 18px;
            border-radius: 8px;
            font-size: 12px;
            color: #94a3b8;
            backdrop-filter: blur(8px);
            display: flex;
            gap: 20px;
            pointer-events: none;
        }}
        .hud-item span {{
            color: #ffd700;
            font-weight: 700;
        }}
        .legend-bar {{
            position: absolute;
            top: 14px;
            right: 14px;
            background: rgba(13, 18, 31, 0.94);
            border: 1px solid #1e293b;
            padding: 10px 16px;
            border-radius: 8px;
            font-size: 11px;
            backdrop-filter: blur(8px);
            display: flex;
            flex-direction: column;
            gap: 6px;
            pointer-events: none;
        }}
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 8px;
            color: #cbd5e1;
        }}
        .legend-dot {{
            width: 12px;
            height: 4px;
            border-radius: 2px;
        }}
    </style>
</head>
<body>
    <header>
        <div class="title-group">
            <div class="logo-badge">CAD STUDIO</div>
            <h1>Autonomous Racing Trajectory Viewer</h1>
        </div>

        <div class="track-nav" id="trackNav"></div>

        <div class="controls">
            <button class="ctrl-btn" onclick="zoomByFactor(0.75)" title="Zoom In">🔍 +</button>
            <button class="ctrl-btn" onclick="zoomByFactor(1.33)" title="Zoom Out">🔍 -</button>
            <button class="ctrl-btn" onclick="resetView()" title="Fit to Screen">⟲ Reset</button>
        </div>
    </header>

    <div id="viewport">
        <div id="svg-container">
            <svg id="svg-canvas"></svg>
        </div>

        <div class="legend-bar">
            <div class="legend-item"><div class="legend-dot" style="background:#ffd700; height:6px;"></div> Yellow Line (Corner-Phase Sigmoid Late-Apex)</div>
            <div class="legend-item"><div class="legend-dot" style="background:#334155;"></div> Medial Centerline (Voronoi)</div>
            <div class="legend-item"><div class="legend-dot" style="background:#00f0ff; width:6px; height:6px; border-radius:50%;"></div> Webots MS200k LiDAR Raw Scan Points</div>
            <div class="legend-item"><div class="legend-dot" style="background:#ff3366; width:6px; height:6px; border-radius:50%;"></div> Voxel-Filtered Obstacle Boundary</div>
            <div class="legend-item"><div class="legend-dot" style="background:#ff0055; width:8px; height:8px; border-radius:50%;"></div> 135° Blind Apex ToF Sightline</div>
        </div>

        <div class="hud-overlay">
            <div class="hud-item">Track: <span id="hudTrack">Track 1</span></div>
            <div class="hud-item">Rendering: <span style="color:#00ff88;">Zero-CORS Direct Inlined SVG</span></div>
            <div class="hud-item">Zoom: <span style="color:#ffd700;" id="hudZoom">100%</span></div>
        </div>
    </div>

    <script>
        const SVG_DB = {db_json};
        const viewKeys = Object.keys(SVG_DB);

        let currentKey = viewKeys[0];
        let initialViewBox = {{ x: 0, y: 0, width: 1000, height: 800 }};
        let currentViewBox = {{ x: 0, y: 0, width: 1000, height: 800 }};
        let isPanning = false;
        let panStart = {{ x: 0, y: 0 }};

        const nav = document.getElementById("trackNav");
        const svgCanvas = document.getElementById("svg-canvas");
        const hudTrack = document.getElementById("hudTrack");
        const hudZoom = document.getElementById("hudZoom");
        const viewport = document.getElementById("viewport");

        // Build Nav Buttons
        viewKeys.forEach((key, idx) => {{
            const btn = document.createElement("button");
            btn.className = `track-btn ${{idx === 0 ? 'active' : ''}}`;
            btn.textContent = SVG_DB[key].name;
            btn.onclick = () => loadTrack(key);
            nav.appendChild(btn);
        }});

        function decodeBase64Utf8(b64) {{
            const binStr = atob(b64);
            const bytes = new Uint8Array(binStr.length);
            for (let i = 0; i < binStr.length; i++) {{
                bytes[i] = binStr.charCodeAt(i);
            }}
            return new TextDecoder('utf-8').decode(bytes);
        }}

        function loadTrack(key) {{
            currentKey = key;
            const item = SVG_DB[key];
            if (!item) return;

            document.querySelectorAll('.track-btn').forEach((b, i) => {{
                b.className = `track-btn ${{viewKeys[i] === key ? 'active' : ''}}`;
            }});

            hudTrack.textContent = item.name;

            const svgText = decodeBase64Utf8(item.data);
            const parser = new DOMParser();
            const doc = parser.parseFromString(svgText, "image/svg+xml");
            const loadedSvg = doc.querySelector("svg");

            if (loadedSvg) {{
                const vbAttr = loadedSvg.getAttribute("viewBox");
                if (vbAttr) {{
                    const parts = vbAttr.split(/[\s,]+/).map(Number);
                    initialViewBox = {{ x: parts[0], y: parts[1], width: parts[2], height: parts[3] }};
                }} else {{
                    const w = parseFloat(loadedSvg.getAttribute("width")) || 1200;
                    const h = parseFloat(loadedSvg.getAttribute("height")) || 800;
                    initialViewBox = {{ x: 0, y: 0, width: w, height: h }};
                }}
                currentViewBox = {{ ...initialViewBox }};

                svgCanvas.innerHTML = loadedSvg.innerHTML;
                updateViewBox();
            }}
        }}

        function updateViewBox() {{
            svgCanvas.setAttribute("viewBox", `${{currentViewBox.x}} ${{currentViewBox.y}} ${{currentViewBox.width}} ${{currentViewBox.height}}`);
            const zoomPct = Math.round((initialViewBox.width / currentViewBox.width) * 100);
            hudZoom.textContent = `${{zoomPct}}%`;
        }}

        function zoomAtPoint(clientX, clientY, factor) {{
            const rect = viewport.getBoundingClientRect();
            const mouseX = clientX - rect.left;
            const mouseY = clientY - rect.top;

            const svgX = currentViewBox.x + (mouseX / rect.width) * currentViewBox.width;
            const svgY = currentViewBox.y + (mouseY / rect.height) * currentViewBox.height;

            const newWidth = currentViewBox.width * factor;
            const newHeight = currentViewBox.height * factor;

            currentViewBox.x = svgX - (mouseX / rect.width) * newWidth;
            currentViewBox.y = svgY - (mouseY / rect.height) * newHeight;
            currentViewBox.width = newWidth;
            currentViewBox.height = newHeight;

            updateViewBox();
        }}

        function zoomByFactor(factor) {{
            const rect = viewport.getBoundingClientRect();
            zoomAtPoint(rect.left + rect.width / 2, rect.top + rect.height / 2, factor);
        }}

        function resetView() {{
            currentViewBox = {{ ...initialViewBox }};
            updateViewBox();
        }}

        viewport.addEventListener('wheel', (e) => {{
            e.preventDefault();
            e.stopPropagation();
            const factor = e.deltaY < 0 ? 0.85 : 1.18;
            zoomAtPoint(e.clientX, e.clientY, factor);
        }}, {{ passive: false }});

        viewport.addEventListener('mousedown', (e) => {{
            if (e.button === 0) {{
                isPanning = true;
                panStart = {{ x: e.clientX, y: e.clientY }};
            }}
        }});

        window.addEventListener('mousemove', (e) => {{
            if (!isPanning) return;
            const rect = viewport.getBoundingClientRect();
            const dx = (e.clientX - panStart.x) * (currentViewBox.width / rect.width);
            const dy = (e.clientY - panStart.y) * (currentViewBox.height / rect.height);

            currentViewBox.x -= dx;
            currentViewBox.y -= dy;
            panStart = {{ x: e.clientX, y: e.clientY }};
            updateViewBox();
        }});

        window.addEventListener('mouseup', () => {{
            isPanning = false;
        }});

        // Initialize with first track (Webots Noisy Map)
        loadTrack(viewKeys[0]);
    </script>
</body>
</html>
"""

    with open(CAD_HTML, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"[OK] Saved Standalone Zero-CORS CAD Viewer: {CAD_HTML}")

    with open(BACKUP_HTML, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"[OK] Saved Standalone Backup Viewer: {BACKUP_HTML}")

    shutil.copy2(CAD_HTML, os.path.join(ART_DIR, "cad_viewer.html"))
    print("[OK] Synchronized to artifacts directory!")

if __name__ == "__main__":
    main()
