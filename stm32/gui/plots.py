"""
plots.py
========
High-DPI Canvas 14-Bit Angle Compass Dial and Real-Time Velocity Tracking Oscilloscope
for STM32G474 20 kHz Dual FOC Bench.
"""

import math
import tkinter as tk


class MotorAngleDial(tk.Canvas):
    def __init__(self, parent, size=200, title="MOTOR", accent_color="#00E5FF", **kwargs):
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

            if is_cardinal and r > 65:
                tx = cx + (r - 26) * math.cos(rad)
                ty = cy + (r - 26) * math.sin(rad)
                self.create_text(tx, ty, text=f"{deg}°", fill="#8EA1C0", font=("Consolas", 8, "bold"))

        # 3. Electrical Angle Orbit Marker (Amber #FFB300)
        elec_rad = math.radians(self.elec_angle_deg - 90)
        ex = cx + (r - 12) * math.cos(elec_rad)
        ey = cy + (r - 12) * math.sin(elec_rad)
        self.create_oval(ex - 4, ey - 4, ex + 4, ey + 4, fill="#FFB300", outline="#FFE082", width=1.5)

        # 4. Primary Mechanical Angle Needle
        mech_rad = math.radians(self.angle_deg - 90)
        nx = cx + (r - 20) * math.cos(mech_rad)
        ny = cy + (r - 20) * math.sin(mech_rad)
        tx = cx - 12 * math.cos(mech_rad)
        ty = cy - 12 * math.sin(mech_rad)
        self.create_line(tx, ty, nx, ny, fill=self.accent_color, width=3, capstyle=tk.ROUND)

        # Center Pivot Cap
        self.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill=self.accent_color, outline="#FFFFFF", width=1.5)
        self.create_oval(cx - 2.5, cy - 2.5, cx + 2.5, cy + 2.5, fill="#0F131C")

        # 5. Digital Readout Overlay (Center-Lower)
        if r > 55:
            deg_str = f"{self.angle_deg:05.1f}°"
            elec_str = f"E: {self.elec_angle_deg:05.1f}°"
            raw_str = f"{self.raw_14bit} / 16383"

            self.create_text(cx, cy + (r * 0.42), text=deg_str, fill="#FFFFFF", font=("Consolas", 11, "bold"))
            self.create_text(cx, cy + (r * 0.62), text=elec_str, fill="#FFB300", font=("Consolas", 8, "bold"))
            self.create_text(cx, cy + (r * 0.78), text=raw_str, fill="#8EA1C0", font=("Consolas", 8))


class VelocityOscilloscopeChart(tk.Canvas):
    """Real-time velocity tracking oscilloscope inspired by the ESP32 suite."""
    def __init__(self, parent, height=130, **kwargs):
        super().__init__(
            parent,
            height=height,
            bg="#0A0E17",
            highlightthickness=1,
            highlightbackground="#1E293B",
            **kwargs
        )
        self.height = height
        self.bind("<Configure>", lambda e: self.redraw([], [], []))

    def redraw(self, target_hist, actual_hist, uq_hist=None):
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 40 or h < 30:
            return

        mid_y = h / 2.0
        self.create_line(0, mid_y, w, mid_y, fill="#1E293B", dash=(4, 4))
        self.create_line(0, h * 0.25, w, h * 0.25, fill="#141C2B", dash=(2, 4))
        self.create_line(0, h * 0.75, w, h * 0.75, fill="#141C2B", dash=(2, 4))

        n = len(actual_hist)
        if n < 2:
            self.create_text(12, 12, text="Target RPM (Yellow)  |  Actual RPM (Green)  |  Waiting for telemetry...", anchor="nw", fill="#64748B", font=("Consolas", 8))
            return

        max_val = max(
            max((abs(x) for x in target_hist), default=100.0),
            max((abs(x) for x in actual_hist), default=100.0),
            100.0
        ) * 1.15

        def to_xy(idx, val):
            x = (idx / max(n - 1, 1)) * (w - 10) + 5
            y = mid_y - (val / max_val) * (mid_y - 12)
            return x, y

        # Target RPM polyline (Amber/Yellow)
        pts_t = []
        for i, v in enumerate(target_hist):
            pts_t.extend(to_xy(i, v))
        if len(pts_t) >= 4:
            self.create_line(*pts_t, fill="#F59E0B", width=1.5)

        # Actual RPM polyline (Neon Green)
        pts_a = []
        for i, v in enumerate(actual_hist):
            pts_a.extend(to_xy(i, v))
        if len(pts_a) >= 4:
            self.create_line(*pts_a, fill="#10B981", width=2.0)

        # Legend & Scale Overlay
        last_t = target_hist[-1] if target_hist else 0.0
        last_a = actual_hist[-1] if actual_hist else 0.0
        err = last_t - last_a
        self.create_text(
            8, 6,
            text=f"Scale: ±{max_val:.0f} RPM   |   Target: {last_t:+.1f} RPM (Yellow)   |   Actual: {last_a:+.1f} RPM (Green)   |   Err: {err:+.1f} RPM",
            anchor="nw",
            fill="#94A3B8",
            font=("Consolas", 8, "bold")
        )
