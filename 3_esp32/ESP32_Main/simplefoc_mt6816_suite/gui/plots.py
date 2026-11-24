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


def draw_harmonic_overlay_plot(canvas, bin_points, h_coeffs):
    """
    Draws the 128-slot discrete drag-free spatial bin error points alongside
    the reconstructed smooth analytical harmonic curve (H1 + H2 + H4).
    Displays diagnostic badges for eccentricity, tilt, clamping distortion, and flux saturation.
    """
    try:
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 40 or h < 40:
            return

        pad_left = 65
        pad_right = 30
        pad_top = 40
        pad_bottom = 35

        pw = w - pad_left - pad_right
        ph = h - pad_top - pad_bottom
        cx = pad_left + pw / 2
        cy = pad_top + ph / 2

        # Dark card plot background
        canvas.create_rectangle(pad_left, pad_top, pad_left + pw, pad_top + ph, fill="#121722", outline="#222D3E", width=1)

        # Determine y-axis scale in mechanical degrees
        y_max = 1.0  # default minimum scale +-1.0 deg
        if bin_points:
            # bin_points are in radians -> convert to deg
            max_pt = max(abs(x * 57.2957795) for x in bin_points)
            y_max = max(1.0, max_pt * 1.25)
        elif h_coeffs and h_coeffs.get("PkPk", 0) > 0:
            y_max = max(1.0, (h_coeffs["PkPk"] * 0.5) * 1.3)

        # Horizontal gridlines & Y-labels
        grid_steps = 4
        for step in range(-grid_steps, grid_steps + 1):
            deg_val = (step / grid_steps) * y_max
            y = cy - (deg_val / y_max) * (ph / 2)
            is_zero = (step == 0)
            col = "#37474F" if is_zero else "#1C2433"
            width = 2 if is_zero else 1
            dash = () if is_zero else (2, 4)
            canvas.create_line(pad_left, y, pad_left + pw, y, fill=col, width=width, dash=dash)
            canvas.create_text(pad_left - 8, y, text=f"{deg_val:+.2f}°", fill="#8A99AD" if not is_zero else "#00E5FF",
                               font=("Segoe UI", 8), anchor="e")

        # Vertical gridlines & X-labels (Mechanical Angle 0 to 360 deg)
        for a in [0, 45, 90, 135, 180, 225, 270, 315, 360]:
            x = pad_left + (a / 360.0) * pw
            canvas.create_line(x, pad_top, x, pad_top + ph, fill="#1C2433", width=1, dash=(2, 4))
            canvas.create_text(x, pad_top + ph + 12, text=f"{a}°", fill="#8A99AD", font=("Segoe UI", 8), anchor="center")

        # X-Axis label
        canvas.create_text(pad_left + pw / 2, pad_top + ph + 24, text="Mechanical Rotor Angle θ (degrees)",
                           fill="#8A99AD", font=("Segoe UI", 8, "bold"))

        # Y-Axis label
        canvas.create_text(16, pad_top + ph / 2, text="Angular Error (deg mech)",
                           fill="#00E5FF", font=("Segoe UI", 8, "bold"), angle=90)

        # Plot 128 discrete spatial bin error points (Cyan circles)
        if bin_points and len(bin_points) >= 128:
            pts_count = len(bin_points)
            for i, rad_err in enumerate(bin_points):
                deg_err = rad_err * 57.2957795
                angle_deg = ((i + 0.5) / float(pts_count)) * 360.0
                px = pad_left + (angle_deg / 360.0) * pw
                py = cy - (deg_err / y_max) * (ph / 2)
                py = max(pad_top + 2, min(pad_top + ph - 2, py))
                canvas.create_oval(px - 2.5, py - 2.5, px + 2.5, py + 2.5, fill="#00E5FF", outline="#FFFFFF", width=1)

        # Plot continuous analytical harmonic curve (Green line)
        if h_coeffs and ("A1" in h_coeffs or "H1" in h_coeffs):
            a1 = h_coeffs.get("A1", 0.0)
            b1 = h_coeffs.get("B1", 0.0)
            a2 = h_coeffs.get("A2", 0.0)
            b2 = h_coeffs.get("B2", 0.0)
            a4 = h_coeffs.get("A4", 0.0)
            b4 = h_coeffs.get("B4", 0.0)

            curve_pts = []
            N_STEPS = 240
            for k in range(N_STEPS + 1):
                th = (float(k) / float(N_STEPS)) * (2.0 * math.pi)
                # Double angle identities evaluation:
                sin1 = math.sin(th)
                cos1 = math.cos(th)
                sin2 = 2.0 * sin1 * cos1
                cos2 = cos1 * cos1 - sin1 * sin1
                sin4 = 2.0 * sin2 * cos2
                cos4 = cos2 * cos2 - sin2 * sin2
                harm_rad = (a1 * cos1 + b1 * sin1) + (a2 * cos2 + b2 * sin2) + (a4 * cos4 + b4 * sin4)
                harm_deg = harm_rad * 57.2957795

                px = pad_left + (float(k) / float(N_STEPS)) * pw
                py = cy - (harm_deg / y_max) * (ph / 2)
                py = max(pad_top + 1, min(pad_top + ph - 1, py))
                curve_pts.append((px, py))

            for k in range(len(curve_pts) - 1):
                canvas.create_line(curve_pts[k][0], curve_pts[k][1], curve_pts[k+1][0], curve_pts[k+1][1],
                                   fill="#69F0AE", width=2)

        # Header Legend & Diagnostic Summary
        legend_x = pad_left + 10
        legend_y = pad_top - 24

        # Legend dots
        canvas.create_oval(legend_x, legend_y - 4, legend_x + 8, legend_y + 4, fill="#00E5FF", outline="#FFF")
        canvas.create_text(legend_x + 12, legend_y, text="128-Bin Drag-Free Samples", fill="#00E5FF", font=("Segoe UI", 8, "bold"), anchor="w")

        canvas.create_line(legend_x + 180, legend_y, legend_x + 200, legend_y, fill="#69F0AE", width=2)
        canvas.create_text(legend_x + 205, legend_y, text="Reconstructed Harmonics (H1+H2+H4)", fill="#69F0AE", font=("Segoe UI", 8, "bold"), anchor="w")

        # Diagnostics Badges (Right-aligned in header)
        if h_coeffs:
            h1 = h_coeffs.get("H1", 0.0)
            h2 = h_coeffs.get("H2", 0.0)
            h3 = h_coeffs.get("H3", 0.0)
            h4 = h_coeffs.get("H4", 0.0)
            pk = h_coeffs.get("PkPk", 0.0)
            r2 = h_coeffs.get("R2", 0.0)
            res_pk = h_coeffs.get("ResPkPk", 0.0)

            clamp_alert = h1 > 0.05 and h3 > 0.20 * h1
            sat_alert = h1 > 0.05 and h4 > 0.25 * h1

            r2_str = f" | R²: {r2*100.0:.1f}%" if r2 > 0 else ""
            res_str = f" | Res: ±{res_pk*0.5:.3f}°" if res_pk > 0 else ""
            stat_txt = f"H1: ±{h1:.3f}° | H2: ±{h2:.3f}° | H3: ±{h3:.3f}°{' ⚠️' if clamp_alert else ''} | H4: ±{h4:.3f}°{' ⚠️' if sat_alert else ''} | Pk-Pk: {pk:.3f}°{r2_str}{res_str}"
            canvas.create_text(w - pad_right, legend_y, text=stat_txt, fill="#FFD600" if (clamp_alert or sat_alert) else "#E0E0E0",
                               font=("Segoe UI", 8, "bold"), anchor="e")
        else:
            if not bin_points:
                canvas.create_text(cx, cy, text="Click '🚀 Continuous Harmonic Calib' to sweep and render 128-slot spatial harmonic model",
                                   fill="#546E7A", font=("Segoe UI", 10, "italic"))
    except Exception:
        pass

