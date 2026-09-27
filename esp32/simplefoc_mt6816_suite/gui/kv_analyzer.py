"""
Closed-Loop Motor KV Analyzer Lab:
Firmware test control, real-time data table formatting, report CSV/text export, and snippet generation.
"""

import time
import tkinter as tk
from tkinter import filedialog, messagebox


class KVAnalyzer:
    def __init__(self, reader, root, progress_var, status_var):
        self.reader = reader
        self.root = root
        self.progress_var = progress_var
        self.status_var = status_var
        self.table_finalized = False

    def start_test(self, btn_run, kv_tree):
        if not self.reader.connected:
            messagebox.showerror("Connection Error", "Please connect to ESP32-S3 first!")
            return

        with self.reader.lock:
            self.reader.kv_points = []
            self.reader.kv_results = {}
            self.reader.kv_test_active = True

        for item in kv_tree.get_children():
            kv_tree.delete(item)

        self.table_finalized = False
        btn_run.config(text="🔄 Running KV Sweep...", bg="#FF9100", fg="#000")
        self.progress_var.set(5.0)
        self.status_var.set("Initiating FOC Alignment & Voltage Sweep...")
        self.reader.send_cmd("k")

    def abort_test(self, btn_run):
        self.reader.send_cmd("x")
        btn_run.config(text="🚀 START 18-Stage KV Sweep (k)", bg="#00E676", fg="#000")
        self.status_var.set("KV Test Aborted by User.")
        with self.reader.lock:
            self.reader.kv_test_active = False

    def apply_kv_to_motor(self, entry_kv, set_param_fn):
        with self.reader.lock:
            res = dict(self.reader.kv_results)
        kv = res.get('KV', 171.9)
        entry_kv.delete(0, "end")
        entry_kv.insert(0, f"{kv:.1f}")
        set_param_fn("q", f"{kv:.1f}")
        messagebox.showinfo("KV Applied", f"Motor KV rating set to {kv:.1f} RPM/V on ESP32-S3 controller and updated in Live Tuning Panel.")

    def copy_simplefoc_snippet(self):
        with self.reader.lock:
            res = dict(self.reader.kv_results)
        kv = res.get('KV', 130.0)
        snippet = f"motor.KV_rating = {kv:.1f}f; // GM3506 Measured Closed-Loop KV"
        self.root.clipboard_clear()
        self.root.clipboard_append(snippet)
        messagebox.showinfo("Copied", f"Copied SimpleFOC configuration to clipboard:\n\n{snippet}")

    def export_report(self):
        with self.reader.lock:
            pts = list(self.reader.kv_points)
            res = dict(self.reader.kv_results)

        if not pts and not res:
            messagebox.showinfo("No Data", "Please run the KV test first before exporting!")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV File", "*.csv"), ("Text Report", "*.txt"), ("All Files", "*.*")],
            title="Export Closed-Loop KV Test Report"
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write("# DA WINNA — CLOSED-LOOP MOTOR KV TEST REPORT\n")
                f.write(f"# Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("# Motor: GM3506 (11 Pole Pairs) @ 12V SpaceVectorPWM\n")
                f.write("# Sensor: MT6816/MT6818 14-Bit SPI Magnetic Encoder\n")
                f.write("# ====================================================================\n")
                if res:
                    f.write(f"# True Motor KV: {res.get('KV', 0):.2f} RPM/V\n")
                    f.write(f"# Forward KV (CW): {res.get('FwdKV', 0):.2f} RPM/V\n")
                    f.write(f"# Reverse KV (CCW): {res.get('RevKV', 0):.2f} RPM/V\n")
                    f.write(f"# Linearity (R^2): {res.get('R2', 0):.5f}\n")
                    f.write(f"# Friction Voltage (V0): {res.get('V0', 0):.3f} V\n")
                    f.write(f"# Back-EMF Constant (Ke): {res.get('Ke', 0):.5f} V*s/rad\n")
                    f.write(f"# Torque Constant (Kt): {res.get('Kt', 0):.5f} N*m/A\n")
                    f.write(f"# Max No-Load Speed @ 12V: {res.get('MaxRPM', 0):.1f} RPM\n")
                    f.write(f"# CW/CCW Symmetry: {res.get('Symmetry', 0):.1f} %\n")
                    f.write("# ====================================================================\n\n")

                f.write("Stage,Voltage_V,Measured_RPM,Speed_Ripple_RPM,StdDev_RPM\n")
                for p in pts:
                    f.write(f"{p['stage']},{p['voltage']:.2f},{p['rpm']:.2f},{p['ripple']:.2f},{p['std_dev']:.3f}\n")

            messagebox.showinfo("Export Success", f"KV test report successfully saved to:\n{file_path}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to save report:\n{str(e)}")

    def update_table_and_metrics(self, kv_tree, kv_pts, kv_res, kv_active, btn_run, cards):
        if kv_active:
            btn_run.config(text="🔄 Running KV Sweep...", bg="#FF9100", fg="#000")
            if kv_pts:
                last_pt = kv_pts[-1]
                pct = (last_pt['stage'] / last_pt['total_stages']) * 100.0
                self.progress_var.set(pct)
                self.status_var.set(
                    f"Stage [{last_pt['stage']}/{last_pt['total_stages']}]: Commanded Uq = {last_pt['voltage']:+.1f}V | "
                    f"Measured Speed = {last_pt['rpm']:+.1f} RPM | Ripple = ±{last_pt['ripple']*0.5:.1f} RPM"
                )
        else:
            if kv_res:
                btn_run.config(text="🚀 START 18-Stage KV Sweep (v)", bg="#00E676", fg="#000")
                self.progress_var.set(100.0)
                self.status_var.set(
                    f"✅ KV Sweep Complete! True Motor KV = {kv_res.get('KV', 0):.1f} RPM/V (R² = {kv_res.get('R2', 0):.5f})"
                )

        if kv_res:
            kv_val = kv_res.get('KV', 0.0)
            r2_val = kv_res.get('R2', 0.0)
            v0_val = kv_res.get('V0', 0.0)
            ke_val = kv_res.get('Ke', 0.0)
            kt_val = kv_res.get('Kt', 0.0)
            sym_val = kv_res.get('Symmetry', 0.0)
            max_rpm = kv_res.get('MaxRPM', 0.0)

            cards["kv_true"].config(text=f"{kv_val:.1f} RPM/V")
            cards["kv_r2"].config(text=f"{r2_val:.5f}")
            cards["kv_v0"].config(text=f"{v0_val:.2f} V")
            cards["kv_ke"].config(text=f"{ke_val:.4f} V·s/rad")
            cards["kv_kt"].config(text=f"{kt_val:.4f} N·m/A")
            cards["kv_sym"].config(text=f"{sym_val:.1f} %")
            cards["kv_max_rpm"].config(text=f"{max_rpm:.1f} RPM")
            cards["kv_code"].config(text=f"motor.KV_rating = {kv_val:.1f}f;")
            cards["lbl_r2_tag"].config(text=f"R² Fit: {r2_val:.5f} | KV: {kv_val:.1f} RPM/V")

        current_rows = len(kv_tree.get_children())
        slope = kv_res.get('KV', 130.0) if kv_res else 130.0
        v0 = kv_res.get('V0', 0.0) if kv_res else 0.0

        if len(kv_pts) > current_rows:
            for i in range(current_rows, len(kv_pts)):
                p = kv_pts[i]
                v = p['voltage']
                rpm = p['rpm']
                if v >= 0:
                    fit_rpm = max(0.0, (v - v0) * slope)
                else:
                    fit_rpm = min(0.0, (v + v0) * slope)
                residual = rpm - fit_rpm

                row_vals = (
                    f"#{p['stage']}",
                    f"{v:+.1f} V",
                    f"{rpm:+.1f} RPM",
                    f"{fit_rpm:+.1f} RPM",
                    f"{residual:+.1f}",
                    f"±{p['ripple']*0.5:.1f}",
                    f"±{p['std_dev']:.2f}"
                )
                kv_tree.insert("", "end", values=row_vals)
            children = kv_tree.get_children()
            if children:
                kv_tree.see(children[-1])
        elif kv_res and not self.table_finalized:
            children = kv_tree.get_children()
            if len(children) == len(kv_pts) and len(kv_pts) > 0:
                for idx, child_id in enumerate(children):
                    p = kv_pts[idx]
                    v = p['voltage']
                    rpm = p['rpm']
                    if v >= 0:
                        fit_rpm = max(0.0, (v - v0) * slope)
                    else:
                        fit_rpm = min(0.0, (v + v0) * slope)
                    residual = rpm - fit_rpm
                    kv_tree.item(child_id, values=(
                        f"#{p['stage']}",
                        f"{v:+.1f} V",
                        f"{rpm:+.1f} RPM",
                        f"{fit_rpm:+.1f} RPM",
                        f"{residual:+.1f}",
                        f"±{p['ripple']*0.5:.1f}",
                        f"±{p['std_dev']:.2f}"
                    ))
                self.table_finalized = True
