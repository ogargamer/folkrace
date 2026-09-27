"""
Canvas drawing utilities:
14-Bit Absolute Angle Compass Dial, Real-Time Oscilloscope,
KV Scatter & Linear Regression Plot, and Rotational Symmetry Needle Meter.
"""

import math


def draw_gauge(canvas, deg):
    try:
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 10 or h < 10:
            return

        cx, cy = w / 2, h / 2
        r = min(cx, cy) - 18

        canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#2A374A", width=3, fill="#121824")
        canvas.create_oval(cx - (r - 10), cy - (r - 10), cx + (r - 10), cy + (r - 10), outline="#1F2A38", width=1)

        for a in range(0, 360, 30):
            rad_a = math.radians(a - 90)
            inner_r = r - 14 if a % 90 == 0 else r - 7
            x1 = cx + (r - 2) * math.cos(rad_a)
            y1 = cy + (r - 2) * math.sin(rad_a)
            x2 = cx + inner_r * math.cos(rad_a)
            y2 = cy + inner_r * math.sin(rad_a)
            color = "#00E5FF" if a % 90 == 0 else "#4A5D78"
            canvas.create_line(x1, y1, x2, y2, fill=color, width=2 if a % 90 == 0 else 1)

            if a % 90 == 0:
                lx = cx + (r - 24) * math.cos(rad_a)
                ly = cy + (r - 24) * math.sin(rad_a)
                canvas.create_text(lx, ly, text=f"{a}°", fill="#8A99AD", font=("Segoe UI", 8, "bold"))

        canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#00E5FF", outline="#FFFFFF", width=2)

        needle_rad = math.radians(deg - 90)
        nx = cx + (r - 15) * math.cos(needle_rad)
        ny = cy + (r - 15) * math.sin(needle_rad)

        perp_rad = needle_rad + math.pi / 2
        bx1 = cx + 6 * math.cos(perp_rad)
        by1 = cy + 6 * math.sin(perp_rad)
        bx2 = cx - 6 * math.cos(perp_rad)
        by2 = cy - 6 * math.sin(perp_rad)

        canvas.create_polygon(bx1, by1, nx, ny, bx2, by2, fill="#FF5252", outline="#FF1744", width=1)
        canvas.create_line(cx, cy, nx, ny, fill="#FFFFFF", width=2)
        canvas.create_text(cx, cy + (r / 2) + 6, text=f"{deg:.1f}°", fill="#00E5FF", font=("Segoe UI", 14, "bold"))
    except Exception:
        pass


def draw_chart(canvas, reader):
    try:
        with reader.lock:
            hist_act = list(reader.history_actual_rpm)
            hist_tgt = list(reader.history_target_rpm)

        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 20 or h < 20 or len(hist_act) < 2:
            return

        cy = h / 2
        canvas.create_line(0, cy, w, cy, fill="#2A374A", width=1, dash=(4, 4))
        canvas.create_text(30, cy - 8, text="0 RPM", fill="#546E7A", font=("Segoe UI", 8))

        max_val = max(100.0, max([abs(x) for x in hist_act + hist_tgt] + [100.0]))
        scale_y = (h / 2 - 10) / max_val

        n = len(hist_act)
        dx = w / max(n - 1, 1)

        pts_tgt = []
        for i, val in enumerate(hist_tgt):
            x = i * dx
            y = cy - (val * scale_y)
            pts_tgt.append((x, y))
        for i in range(len(pts_tgt) - 1):
            canvas.create_line(pts_tgt[i][0], pts_tgt[i][1], pts_tgt[i+1][0], pts_tgt[i+1][1], fill="#FFD600", width=2)

        pts_act = []
        for i, val in enumerate(hist_act):
            x = i * dx
            y = cy - (val * scale_y)
            pts_act.append((x, y))
        for i in range(len(pts_act) - 1):
            canvas.create_line(pts_act[i][0], pts_act[i][1], pts_act[i+1][0], pts_act[i+1][1], fill="#69F0AE", width=2)
    except Exception:
        pass


def draw_kv_scatter_plot(canvas, kv_pts, kv_res):
    try:
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 40 or h < 40:
            return

        pad_left = 80
        pad_right = 30
        pad_top = 30
        pad_bottom = 40

        pw = w - pad_left - pad_right
        ph = h - pad_top - pad_bottom

        cx = pad_left + pw / 2
        cy = pad_top + ph / 2

        canvas.create_rectangle(pad_left, pad_top, pad_left + pw, pad_top + ph, fill="#121722", outline="#222D3E", width=1)

        v_max = 12.0
        rpm_max = 1600.0

        for rpm_val in [-1500, -1000, -500, 0, 500, 1000, 1500]:
            y = cy - (rpm_val / rpm_max) * (ph / 2)
            color = "#37474F" if rpm_val == 0 else "#1F2837"
            width = 2 if rpm_val == 0 else 1
            dash = () if rpm_val == 0 else (2, 4)
            canvas.create_line(pad_left, y, pad_left + pw, y, fill=color, width=width, dash=dash)
            canvas.create_text(pad_left - 8, y, text=f"{rpm_val:+} RPM", fill="#8A99AD", font=("Segoe UI", 8), anchor="e")

        for v_val in [-12, -8, -4, 0, 4, 8, 12]:
            x = cx + (v_val / v_max) * (pw / 2)
            color = "#37474F" if v_val == 0 else "#1F2837"
            width = 2 if v_val == 0 else 1
            dash = () if v_val == 0 else (2, 4)
            canvas.create_line(x, pad_top, x, pad_top + ph, fill=color, width=width, dash=dash)
            canvas.create_text(x, pad_top + ph + 14, text=f"{v_val:+} V", fill="#8A99AD", font=("Segoe UI", 8), anchor="center")

        canvas.create_text(pad_left + pw / 2, pad_top + ph + 30, text="Commanded Q-Axis Voltage Uq (V)", fill="#00E5FF", font=("Segoe UI", 9, "bold"))
        canvas.create_text(18, pad_top + ph / 2, text="Actual Speed (RPM)", fill="#69F0AE", font=("Segoe UI", 9, "bold"), angle=90)

        if kv_res and kv_res.get('KV', 0) > 1.0:
            kv = kv_res['KV']
            v0 = kv_res.get('V0', 0.0)

            # Positive voltage line
            x_start_pos = cx + (v0 / v_max) * (pw / 2)
            y_start_pos = cy
            calc_pos_rpm = (12.0 - v0) * kv
            if calc_pos_rpm > rpm_max:
                v_pos_end = v0 + (rpm_max / kv)
                x_end_pos = cx + (v_pos_end / v_max) * (pw / 2)
                y_end_pos = pad_top
            else:
                x_end_pos = cx + (12.0 / v_max) * (pw / 2)
                y_end_pos = cy - (calc_pos_rpm / rpm_max) * (ph / 2)
            canvas.create_line(x_start_pos, y_start_pos, x_end_pos, y_end_pos, fill="#69F0AE", width=2)

            # Negative voltage line
            x_start_neg = cx - (v0 / v_max) * (pw / 2)
            y_start_neg = cy
            if calc_pos_rpm > rpm_max:
                v_neg_end = v0 + (rpm_max / kv)
                x_end_neg = cx - (v_neg_end / v_max) * (pw / 2)
                y_end_neg = pad_top + ph
            else:
                x_end_neg = cx - (12.0 / v_max) * (pw / 2)
                y_end_neg = cy + (calc_pos_rpm / rpm_max) * (ph / 2)
            canvas.create_line(x_start_neg, y_start_neg, x_end_neg, y_end_neg, fill="#69F0AE", width=2)

            eq_text = f"y = {kv:.1f} · (Uq - {v0:.2f}V)  [R² = {kv_res.get('R2', 0):.5f}]"
            canvas.create_text(pad_left + 15, pad_top + 15, text=eq_text, fill="#69F0AE", font=("Segoe UI", 9, "bold"), anchor="w")

        for pt in kv_pts:
            v = pt['voltage']
            rpm = pt['rpm']
            ripple = pt.get('ripple', 0.0)

            px = cx + (v / v_max) * (pw / 2)
            py = cy - (rpm / rpm_max) * (ph / 2)

            if ripple > 0.5:
                r_half = (ripple * 0.5 / rpm_max) * (ph / 2)
                canvas.create_line(px, py - r_half, px, py + r_half, fill="#546E7A", width=1)
                canvas.create_line(px - 3, py - r_half, px + 3, py - r_half, fill="#546E7A", width=1)
                canvas.create_line(px - 3, py + r_half, px + 3, py + r_half, fill="#546E7A", width=1)

            pt_col = "#00E5FF" if v >= 0 else "#FF4081"
            canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill=pt_col, outline="#FFFFFF", width=1)
    except Exception:
        pass


def draw_symmetry_needle(canvas, fwd_rpm, rev_rpm):
    try:
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 20 or h < 20:
            return

        cx = w / 2
        cy = h - 25
        r = min(w / 2 - 40, h - 45)

        # Arc background
        canvas.create_arc(cx - r, cy - r, cx + r, cy + r, start=30, extent=120, style="arc", outline="#2A374A", width=4)
        # Center target balanced zone (±10 deg arc)
        canvas.create_arc(cx - r, cy - r, cx + r, cy + r, start=82, extent=16, style="arc", outline="#69F0AE", width=6)

        # Center mark
        canvas.create_line(cx, cy - r - 8, cx, cy - r + 8, fill="#69F0AE", width=2)
        canvas.create_text(cx, cy - r - 16, text="PERFECT 1:1 SYMMETRY", fill="#69F0AE", font=("Segoe UI", 8, "bold"))

        # Reverse (left) and Forward (right) markers
        canvas.create_text(cx - r + 15, cy - 10, text="◀ CCW Faster", fill="#FF4081", font=("Segoe UI", 8, "bold"), anchor="w")
        canvas.create_text(cx + r - 15, cy - 10, text="CW Faster ▶", fill="#00E5FF", font=("Segoe UI", 8, "bold"), anchor="e")

        # Balance deflection angle: -45 deg to +45 deg
        delta = fwd_rpm - rev_rpm
        denom = max(fwd_rpm + rev_rpm, 100.0)
        deflection = (delta / denom) * 45.0
        deflection = max(-50.0, min(50.0, deflection))

        needle_angle_rad = math.radians(90.0 - deflection)
        nx = cx + (r - 10) * math.cos(needle_angle_rad)
        ny = cy - (r - 10) * math.sin(needle_angle_rad)

        needle_col = "#69F0AE" if abs(delta) < 15.0 and fwd_rpm > 10 and rev_rpm > 10 else ("#00E5FF" if delta > 0 else "#FF4081")
        canvas.create_line(cx, cy, nx, ny, fill=needle_col, width=3, arrow="last", arrowshape=(10, 12, 4))
        canvas.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill=needle_col, outline="#FFF", width=2)

        diff_str = f"Speed Difference: {delta:+.1f} RPM"
        canvas.create_text(cx, cy + 12, text=diff_str, fill="#FFFFFF", font=("Segoe UI", 9, "bold"))
    except Exception:
        pass
