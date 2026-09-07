"""
Visual CAD Vector Snapshot Renderer with Interactive Pan-Zoom & Non-Scaling Stroke
Features:
1. Pure Scalable Vector Graphics (.svg) - No PNG
2. Real-time viewBox Pan & Zoom embedded directly in the SVG (Mouse Wheel Zoom & Click-Drag Pan)
3. vector-effect="non-scaling-stroke" keeps lines strictly 1.5px screen width at 10x, 100x, 1000x zoom!
4. Legend panel positioned completely OUTSIDE the track area at 100% solid opacity
"""

import os
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "racing_results", "snapshots")
os.makedirs(OUTPUT_DIR, exist_ok=True)

BG_COLOR = "#0b0f19"
WALL_COLOR = "#ff0055"
TEXT_COLOR = "#e2e8f0"

PAN_ZOOM_SCRIPT = """
<script type="text/javascript"><![CDATA[
document.addEventListener('DOMContentLoaded', function() {
    var svg = document.documentElement;
    var isPanning = false;
    var startPoint = {x: 0, y: 0};
    
    // Smooth Mouse-Wheel CAD Zoom at cursor position
    window.addEventListener('wheel', function(e) {
        e.preventDefault();
        var vb = svg.viewBox.baseVal;
        var scale = e.deltaY < 0 ? 0.85 : 1.18;
        
        // Get cursor position in SVG coordinates
        var rect = svg.getBoundingClientRect();
        var clientX = e.clientX - rect.left;
        var clientY = e.clientY - rect.top;
        
        var svgX = vb.x + (clientX / rect.width) * vb.width;
        var svgY = vb.y + (clientY / rect.height) * vb.height;
        
        var newWidth = vb.width * scale;
        var newHeight = vb.height * scale;
        
        vb.x = svgX - (clientX / rect.width) * newWidth;
        vb.y = svgY - (clientY / rect.height) * newHeight;
        vb.width = newWidth;
        vb.height = newHeight;
    }, {passive: false});

    // Click and Drag Pan
    window.addEventListener('mousedown', function(e) {
        if (e.button === 0) { // Left click
            isPanning = true;
            startPoint = {x: e.clientX, y: e.clientY};
            svg.style.cursor = 'grabbing';
        }
    });
    
    window.addEventListener('mousemove', function(e) {
        if (!isPanning) return;
        var vb = svg.viewBox.baseVal;
        var rect = svg.getBoundingClientRect();
        var dx = (e.clientX - startPoint.x) * (vb.width / rect.width);
        var dy = (e.clientY - startPoint.y) * (vb.height / rect.height);
        
        vb.x -= dx;
        vb.y -= dy;
        startPoint = {x: e.clientX, y: e.clientY};
    });
    
    window.addEventListener('mouseup', function() {
        isPanning = false;
        svg.style.cursor = 'crosshair';
    });
    
    svg.style.cursor = 'crosshair';
});
]]></script>
"""

def draw_track_walls_and_obstacles(ax, track_data):
    """Draws left and right barrier walls and obstacles"""
    ax.plot(track_data["left_wall"][:, 0], track_data["left_wall"][:, 1], color=WALL_COLOR, linewidth=2.5, label="Left / Outer Barrier Wall")
    ax.plot(track_data["right_wall"][:, 0], track_data["right_wall"][:, 1], color=WALL_COLOR, linewidth=2.5, label="Right / Inner Barrier Wall")
    
    for obs in track_data.get("obstacles", []):
        if obs["type"] == "wall_rectangle":
            rx, ry = obs["center_x"], obs["center_y"]
            w, ext = obs["width"], obs["extrusion"]
            rect = patches.Rectangle((rx - w/2.0, ry - ext/2.0), w, ext, color="#ffb800", alpha=0.95, zorder=6, label=obs.get("name", "Wall Extrusion"))
            ax.add_patch(rect)
        elif obs["type"] == "circle":
            circle = patches.Circle((obs["cx"], obs["cy"]), obs["radius"], color="#ffb800", alpha=0.95, zorder=6, label=obs.get("name", "Center Pillar"))
            ax.add_patch(circle)

def plot_racing_line_with_iterations(track_data, opt_result, save_path):
    """
    Renders iteration evolution lines to pure CAD-like Vector SVG (.svg)
    No PNG is generated.
    """
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)

    draw_track_walls_and_obstacles(ax, track_data)

    # 11 distinct milestone colors
    colors = [
        "#4a5568", # Iter 0 (Medial Axis)
        "#3a86ff", # Iter 30
        "#00b4d8", # Iter 60
        "#06d6a0", # Iter 90
        "#7209b7", # Iter 120
        "#b5179e", # Iter 150
        "#f72585", # Iter 180
        "#ff0054", # Iter 210
        "#ff5400", # Iter 240
        "#ffb800", # Iter 270
        "#00ff88"  # Final Optimal Line
    ]
    history = opt_result["iteration_history"]

    for idx, item in enumerate(history):
        c = colors[idx % len(colors)]
        pts = item["points"]
        pts_c = np.vstack([pts, pts[:1]])
        is_final = (idx == len(history) - 1)
        is_initial = (idx == 0)
        
        if is_initial:
            lw = 1.8
            ls = ":"
            lbl = item["description"]
        elif is_final:
            lw = 2.8
            ls = "-"
            lbl = f"Final Optimal Line: {item['description']}"
        else:
            lw = 1.2
            ls = "--"
            lbl = item["description"]
            
        ax.plot(pts_c[:, 0], pts_c[:, 1], color=c, linewidth=lw, linestyle=ls, label=lbl)

    # Start / Finish line marker
    start_pt = history[-1]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish Line")

    ax.set_title(f"TUM Minimum Curvature Optimization: {track_data.get('name', 'Track')} [CAD Pan & Wheel Zoom Active]", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)

    # Place Legend COMPLETELY OUTSIDE the track axes at 100% solid opacity (framealpha=1.0)
    ax.legend(
        bbox_to_anchor=(1.02, 1.0),
        loc="upper left",
        facecolor="#121826",
        edgecolor="#2d3748",
        labelcolor=TEXT_COLOR,
        framealpha=1.0,
        fontsize=9,
        borderpad=1.0,
        labelspacing=0.8
    )

    # Save SVG
    svg_path = os.path.splitext(save_path)[0] + ".svg"
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    # Post-process SVG:
    # 1. Enforce CAD non-scaling stroke on every element
    # 2. Embed interactive Pan-Zoom script directly in the SVG
    try:
        ET.register_namespace('', 'http://www.w3.org/2000/svg')
        tree = ET.parse(svg_path)
        root = tree.getroot()

        for elem in root.iter():
            tag = elem.tag.split('}')[-1]
            if tag in ['path', 'line', 'polyline', 'polygon', 'circle', 'rect', 'ellipse']:
                elem.set('vector-effect', 'non-scaling-stroke')
                style = elem.get('style')
                if style:
                    parts = [p for p in style.split(';') if not p.strip().startswith('stroke-width')]
                    parts.append('stroke-width: 1.5px')
                    elem.set('style', ';'.join(parts))
                else:
                    elem.set('stroke-width', '1.5px')

        tree.write(svg_path, xml_declaration=True, encoding='utf-8')

        # Inject Pan-Zoom script and global non-scaling CSS
        with open(svg_path, 'r', encoding='utf-8') as f:
            svg_content = f.read()
            
        css_rule = "<style>*{vector-effect: non-scaling-stroke !important; } path, line, polyline { stroke-width: 1.5px !important; }</style>"
        if "</svg>" in svg_content:
            svg_content = svg_content.replace("</svg>", f"{css_rule}\n{PAN_ZOOM_SCRIPT}\n</svg>")
            
        with open(svg_path, 'w', encoding='utf-8') as f:
            f.write(svg_content)

    except Exception as e:
        print(f"Warning: SVG XML post-processing failed: {e}")

    return {
        "svg": svg_path
    }
