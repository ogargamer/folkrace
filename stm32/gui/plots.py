"""
plots.py
========
High-DPI Canvas 14-Bit Angle Compass and Commutation Visualizer Dial for STM32 Dual FOC Bench.
Renders mechanical angle, electrical commutation vector, and raw 14-bit count with dark aesthetic.
"""

import math
import tkinter as tk

class MotorAngleDial(tk.Canvas):
    def __init__(self, parent, size=240, title="MOTOR", accent_color="#00E5FF", **kwargs):
        super().__init__(
            parent,
            width=size,
            height=size,
            bg="#131822",
            highlightthickness=0,
            **kwargs
        )
        self.size = size
        self.title = title
        self.accent_color = accent_color
        self.angle_deg = 0.0
        self.elec_angle_deg = 0.0
        self.raw_14bit = 0
        self.pole_pairs = 11

        self.bind("<Configure>", self._on_resize)
        self.redraw()

    def _on_resize(self, event):
        self.size = min(event.width, event.height)
        self.redraw()

    def update_angle(self, raw_14bit):
        self.raw_14bit = raw_14bit
        self.angle_deg = (raw_14bit / 16384.0) * 360.0
        self.elec_angle_deg = (self.angle_deg * self.pole_pairs) % 360.0
        self.redraw()

    def redraw(self):
        self.delete("all")
        cx = self.winfo_width() / 2 if self.winfo_width() > 1 else self.size / 2
        cy = self.winfo_height() / 2 if self.winfo_height() > 1 else self.size / 2
        r = min(cx, cy) - 10
        if r < 30:
            return

        # 1. Outer Bezel & Subtle Glow Ring
        self.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#1E2738", width=3, fill="#0F131C")
        self.create_oval(cx - r + 6, cy - r + 6, cx + r - 6, cy + r - 6, outline="#2A3447", width=1.5)

        # 2. Tick Marks & Compass Cardinal Points
        for deg in range(0, 360, 30):
            rad = math.radians(deg - 90)
            is_cardinal = (deg % 90 == 0)
            tick_len = 10 if is_cardinal else 5
            x1 = cx + (r - 8) * math.cos(rad)
            y1 = cy + (r - 8) * math.sin(rad)
            x2 = cx + (r - 8 - tick_len) * math.cos(rad)
            y2 = cy + (r - 8 - tick_len) * math.sin(rad)

            color = self.accent_color if is_cardinal else "#4A5A74"
            self.create_line(x1, y1, x2, y2, fill=color, width=2 if is_cardinal else 1)

            if is_cardinal and r > 70:
                tx = cx + (r - 28) * math.cos(rad)
                ty = cy + (r - 28) * math.sin(rad)
                self.create_text(tx, ty, text=f"{deg}°", fill="#8EA1C0", font=("Consolas", 8, "bold"))

        # 3. Electrical Angle Orbit Marker (Amber #FFB300)
        elec_rad = math.radians(self.elec_angle_deg - 90)
        ex = cx + (r - 12) * math.cos(elec_rad)
        ey = cy + (r - 12) * math.sin(elec_rad)
        self.create_oval(ex - 4, ey - 4, ex + 4, ey + 4, fill="#FFB300", outline="#FFE082", width=1.5)

        # 4. Primary Mechanical Angle Needle
        mech_rad = math.radians(self.angle_deg - 90)
        nx = cx + (r - 22) * math.cos(mech_rad)
        ny = cy + (r - 22) * math.sin(mech_rad)
        # Tail
        tx = cx - 14 * math.cos(mech_rad)
        ty = cy - 14 * math.sin(mech_rad)
        self.create_line(tx, ty, nx, ny, fill=self.accent_color, width=3, capstyle=tk.ROUND)

        # Center Pivot Cap
        self.create_oval(cx - 7, cy - 7, cx + 7, cy + 7, fill=self.accent_color, outline="#FFFFFF", width=1.5)
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#0F131C")

        # 5. Digital Readout Overlay (Center-Lower)
        if r > 60:
            deg_str = f"{self.angle_deg:05.1f}°"
            elec_str = f"E: {self.elec_angle_deg:05.1f}°"
            raw_str = f"{self.raw_14bit} / 16383"

            self.create_text(cx, cy + (r * 0.42), text=deg_str, fill="#FFFFFF", font=("Consolas", 12, "bold"))
            self.create_text(cx, cy + (r * 0.62), text=elec_str, fill="#FFB300", font=("Consolas", 9, "bold"))
            self.create_text(cx, cy + (r * 0.78), text=raw_str, fill="#8EA1C0", font=("Consolas", 8))
