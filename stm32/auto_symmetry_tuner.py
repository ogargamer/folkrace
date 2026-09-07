#!/usr/bin/env python3
"""
DA WINNA — Automated Closed-Loop Phase Angle Equalizer & Symmetry Tuner (GM3506 @ 12V)
======================================================================================
Protocol:
  1. Sets gentle alignment voltage (2.5V @ ~0.35A).
  2. Runs baseline 11-Pole Hysteresis-Compensated Calibration ('c').
  3. Establishes baseline zero angle (theta_0) and safety bounds (+/- 0.10 rad / +/- 5.7 deg elec).
  4. Runs iterative closed-loop voltage tests (+/- 4.0V) to find directional asymmetry.
  5. Dynamically nudges motor.zero_electric_angle within safe bounds to maximize symmetry (>98%).
  6. Computes balanced True Motor KV and applies optimal settings to SimpleFOC controller.
"""

import sys
import time
import math
import re
import argparse
import serial
import serial.tools.list_ports


def find_stm32_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if "stm" in desc or "ch340" in desc or "cp210" in desc or "usb" in desc:
            return p.device
    return ports[0].device if ports else None


class AutoSymmetryTuner:
    def __init__(self, port, baud=921600, test_voltage=4.0, max_nudge_rad=0.10):
        self.port = port
        self.baud = baud
        self.test_voltage = float(test_voltage)
        self.max_nudge_rad = float(max_nudge_rad) # Safety bound: +/- 0.10 rad (~ +/- 5.7 deg elec)
        self.ser = None
        self.base_zero_angle = None
        self.optimal_zero_angle = None

    def connect(self):
        print(f"\n[+] Connecting to STM32 on {self.port} @ {self.baud} baud...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        print("[+] Connected successfully.\n")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"u0.0\nx\n")
                self.ser.flush()
                self.ser.close()
            except Exception:
                pass

    def send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open:
            self.ser.write(f"{cmd_str}\n".encode("utf-8"))
            self.ser.flush()

    def read_telemetry_sample(self, duration_s=0.6, settle_s=0.4):
        time.sleep(settle_s)
        t_end = time.time() + duration_s
        rpm_samples = []
        angle_samples = []

        while time.time() < t_end:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Check for stall
            if "SAFETY SHUTDOWN" in line:
                print("\n[!] EMERGENCY STALL DETECTED! Aborting test.")
                self.send_cmd("x")
                return 0.0, 0.0

            # Telemetry format: Ctrl: ... | Target: ... | Actual: +120.4 RPM | Angle: 45.2 deg | ...
            m_rpm = re.search(r'Actual:\s*([+-]?\d+\.?\d*)\s*RPM', line)
            m_ang = re.search(r'Angle:\s*([+-]?\d+\.?\d*)\s*deg', line)

            if m_rpm:
                try:
                    rpm_samples.append(float(m_rpm.group(1)))
                except ValueError:
                    pass
            if m_ang:
                try:
                    angle_samples.append(float(m_ang.group(1)))
                except ValueError:
                    pass

        mean_rpm = sum(rpm_samples) / max(1, len(rpm_samples))
        std_rpm = math.sqrt(sum((x - mean_rpm)**2 for x in rpm_samples) / max(1, len(rpm_samples)))
        return mean_rpm, std_rpm

    def run_baseline_calibration(self):
        print("================================================================================")
        print(" PHASE 1: 11-POLE BASELINE ELECTRICAL ZERO CALIBRATION")
        print("================================================================================")
        print("[*] Setting optimal alignment voltage: 3.50V (Current ~ 0.50A)")
        self.send_cmd("y3.5")
        time.sleep(0.1)

        print("[*] Triggering 11-Pole Multi-Point Calibration ('c')...")
        self.send_cmd("c")

        t_start = time.time()
        mean_zero_found = None

        while time.time() - t_start < 20.0:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if "Pole #" in line or "GRADE" in line or "Alignment" in line or "SimpleFOC" in line:
                print(f"    {line}")

            if "motor.zero_electric_angle =" in line:
                m = re.search(r'motor\.zero_electric_angle\s*=\s*([0-9]+\.[0-9]+)', line)
                if m:
                    mean_zero_found = float(m.group(1))

            if "CALIBRATION REJECTED" in line:
                print("\n[!] Calibration rejected by firmware. Please check wiring / sensor.")
                return False

            if "SimpleFOC Static Code Parameters" in line:
                break

        if mean_zero_found is None:
            # Fallback default
            mean_zero_found = 0.11275
            print(f"[!] Warning: Using default base zero: {mean_zero_found:.5f} rad")

        self.base_zero_angle = mean_zero_found
        self.optimal_zero_angle = mean_zero_found

        print(f"\n[+] Baseline Electrical Zero Angle established: {self.base_zero_angle:.5f} rad ({self.base_zero_angle * 180.0 / math.pi:.2f} deg)")
        print(f"[+] Safety Nudge Range: [{self.base_zero_angle - self.max_nudge_rad:.5f} rad to {self.base_zero_angle + self.max_nudge_rad:.5f} rad] (±{self.max_nudge_rad * 180.0 / math.pi:.2f}°)")
        print("--------------------------------------------------------------------------------\n")
        return True

    def test_bidirectional_speed(self, test_voltage):
        # 1. Forward Spin (+V): 1.5s acceleration + 1.0s sampling
        self.send_cmd(f"u{test_voltage:.2f}")
        fwd_rpm, fwd_std = self.read_telemetry_sample(duration_s=1.0, settle_s=1.5)

        # Coast to zero
        self.send_cmd("u0.0")
        time.sleep(1.0)

        # 2. Reverse Spin (-V): 1.5s acceleration + 1.0s sampling
        self.send_cmd(f"u-{test_voltage:.2f}")
        rev_rpm, rev_std = self.read_telemetry_sample(duration_s=1.0, settle_s=1.5)

        # 3. Stop
        self.send_cmd("u0.0")
        time.sleep(0.8)

        fwd_mag = abs(fwd_rpm)
        rev_mag = abs(rev_rpm)
        delta_rpm = fwd_mag - rev_mag
        max_mag = max(fwd_mag, rev_mag, 1.0)
        symmetry_pct = (min(fwd_mag, rev_mag) / max_mag) * 100.0

        return {
            "fwd_rpm": fwd_mag,
            "rev_rpm": rev_mag,
            "delta_rpm": delta_rpm,
            "symmetry_pct": symmetry_pct,
            "fwd_std": fwd_std,
            "rev_std": rev_std
        }

    def optimize_symmetry(self, max_iterations=7):
        print("================================================================================")
        print(" PHASE 2: AUTOMATED CLOSED-LOOP PHASE ANGLE NUDGING & OPTIMIZATION")
        print(f" Target Test Voltage : ±{self.test_voltage:.2f} V")
        print(f" Target Symmetry     : > 98.0 % (|Δ RPM| < 12.0 RPM)")
        print(f" Maximum Bounds      : ±{self.max_nudge_rad:.4f} rad (±{self.max_nudge_rad * 180.0 / math.pi:.2f}° elec)")
        print("================================================================================")
        print("Iter | Zero Angle (rad) | Nudge (rad) | Forward RPM | Reverse RPM | Delta RPM | Symmetry | Status")
        print("-----+------------------+-------------+-------------+-------------+-----------+----------+-------")

        current_angle = self.base_zero_angle
        best_symmetry = 0.0
        best_angle = current_angle
        best_data = None

        history = []
        for it in range(1, max_iterations + 1):
            # Apply candidate zero electric angle
            self.send_cmd(f"j{current_angle:.5f}")
            time.sleep(0.05)

            data = self.test_bidirectional_speed(self.test_voltage)
            nudge_from_base = current_angle - self.base_zero_angle

            is_valid = (data["fwd_rpm"] > 100.0 and data["rev_rpm"] > 100.0)
            if is_valid:
                history.append({
                    "angle": current_angle,
                    "delta": data["delta_rpm"],
                    "fwd": data["fwd_rpm"],
                    "rev": data["rev_rpm"],
                    "sym": data["symmetry_pct"]
                })

            if is_valid and data["symmetry_pct"] > best_symmetry:
                best_symmetry = data["symmetry_pct"]
                best_angle = current_angle
                best_data = data

            status = "BALANCED" if (is_valid and (data["symmetry_pct"] >= 98.0 or abs(data["delta_rpm"]) < 10.0)) else "TUNING"

            print(f" #{it:02d} | {current_angle:16.5f} | {nudge_from_base:+11.5f} | {data['fwd_rpm']:10.1f}  | {data['rev_rpm']:10.1f}  | {data['delta_rpm']:+9.1f} | {data['symmetry_pct']:7.1f}% | [{status}]")

            if is_valid and (data["symmetry_pct"] >= 98.0 or abs(data["delta_rpm"]) < 10.0) and it >= 2:
                print("\n[+] Perfect 1:1 Directional Symmetry achieved!")
                break

            # Secant root finding interpolation
            pos_pts = [p for p in history if p["delta"] > 0]
            neg_pts = [p for p in history if p["delta"] < 0]

            if pos_pts and neg_pts:
                best_pos = min(pos_pts, key=lambda p: abs(p["delta"]))
                best_neg = min(neg_pts, key=lambda p: abs(p["delta"]))
                d_diff = best_pos["delta"] - best_neg["delta"]
                if abs(d_diff) > 2.0:
                    next_angle = best_neg["angle"] - best_neg["delta"] * (best_pos["angle"] - best_neg["angle"]) / d_diff
                else:
                    next_angle = (best_pos["angle"] + best_neg["angle"]) * 0.5
            else:
                probe_step = - data["delta_rpm"] * 0.000015
                probe_step = max(-0.0020, min(0.0020, probe_step))
                next_angle = current_angle + probe_step

            # Enforce safety bounds relative to base_zero_angle
            min_bound = self.base_zero_angle - self.max_nudge_rad
            max_bound = self.base_zero_angle + self.max_nudge_rad

            if next_angle < min_bound:
                next_angle = min_bound
                print(f"    [!] Reached lower safety bound ({min_bound:.5f} rad). Clamping.")
            elif next_angle > max_bound:
                next_angle = max_bound
                print(f"    [!] Reached upper safety bound ({max_bound:.5f} rad). Clamping.")

            current_angle = next_angle
            time.sleep(0.2)

        self.optimal_zero_angle = best_angle
        # Apply optimal angle
        self.send_cmd(f"j{self.optimal_zero_angle:.5f}")
        time.sleep(0.05)

        # Final Verification
        print("--------------------------------------------------------------------------------")
        print("\n================================================================================")
        print(" PHASE 3: FINAL VERIFICATION & BALANCED KV EXTRACTION")
        print("================================================================================")

        mean_speed = (best_data['fwd_rpm'] + best_data['rev_rpm']) * 0.5
        true_kv = mean_speed / self.test_voltage
        ke = 1.0 / (true_kv * (2.0 * math.pi / 60.0))
        kt = ke

        print(f" Baseline Zero Electric Angle : {self.base_zero_angle:.5f} rad ({self.base_zero_angle * 180.0 / math.pi:.2f}°)")
        print(f" Optimal Neutral Zero Angle   : {self.optimal_zero_angle:.5f} rad ({self.optimal_zero_angle * 180.0 / math.pi:.2f}°)")
        print(f" Optimal Applied Phase Nudge  : {self.optimal_zero_angle - self.base_zero_angle:+.5f} rad ({(self.optimal_zero_angle - self.base_zero_angle) * 180.0 / math.pi:+.2f}° elec)")
        print(f" Forward Speed @ ±{self.test_voltage:.1f}V       : {best_data['fwd_rpm']:.1f} RPM")
        print(f" Reverse Speed @ ±{self.test_voltage:.1f}V       : {best_data['rev_rpm']:.1f} RPM")
        print(f" Final Directional Symmetry   : {best_data['symmetry_pct']:.1f} % (Δ = {best_data['delta_rpm']:+.1f} RPM)")
        print(f" Balanced True Motor KV       : {true_kv:.1f} RPM/V")
        print(f" Back-EMF Constant (Ke)       : {ke:.5f} V*s/rad")
        print(f" Torque Constant (Kt)         : {kt:.5f} N*m/A")
        print("================================================================================")

        # Apply KV to SimpleFOC
        self.send_cmd(f"q{true_kv:.1f}")
        print(f"[+] Applied KV ({true_kv:.1f} RPM/V) and Neutral Zero Angle to STM32 controller.\n")


def main():
    parser = argparse.ArgumentParser(description="Automated Closed-Loop Phase Angle Equalizer & Symmetry Tuner")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate (default: 921600)")
    parser.add_argument("--volt", type=float, default=4.0, help="Test voltage in V (default: 4.0V)")
    parser.add_argument("--max-nudge", type=float, default=0.05, help="Max safe nudge in rad (default: 0.05 rad / ~2.86 deg)")
    args = parser.parse_args()

    port = args.port or find_stm32_port()
    if not port:
        print("[-] Error: No serial port found. Please connect STM32 and try again.")
        sys.exit(1)

    tuner = AutoSymmetryTuner(port=port, baud=args.baud, test_voltage=args.volt, max_nudge_rad=args.max_nudge)
    try:
        tuner.connect()
        if tuner.run_baseline_calibration():
            tuner.optimize_symmetry()
    except KeyboardInterrupt:
        print("\n[!] Tuner interrupted by user.")
    finally:
        tuner.disconnect()
        print("[+] Done.")


if __name__ == "__main__":
    main()
