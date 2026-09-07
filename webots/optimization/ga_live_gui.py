import os
import sys
import json
import csv
import time
from typing import Dict, List, Optional, Any

import numpy as np

from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QFont, QColor, QPalette, QBrush, QPen, QIcon
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QGroupBox,
    QScrollArea,
    QSplitter,
    QFrame,
)

# Matplotlib Qt5 Backend
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

WORKSPACE_ROOT = os.path.dirname(os.path.abspath(__file__))
GEN_SUMMARY_FILE = os.path.join(WORKSPACE_ROOT, "ga_generation_summary.json")
CHECKPOINT_FILE = os.path.join(WORKSPACE_ROOT, "ga_checkpoint.json")
HISTORY_CSV_FILE = os.path.join(WORKSPACE_ROOT, "ga_history.csv")
WINNING_GENOME_FILE = os.path.join(WORKSPACE_ROOT, "winning_mppi_genome.json")


class MetricCard(QFrame):
    """Clean dark-themed KPI summary card."""
    def __init__(self, title: str, initial_value: str = "--", accent_color: str = "#00e5ff"):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: #17202c;
                border: 1px solid #223145;
                border-radius: 8px;
                padding: 6px;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(2)

        self.lbl_title = QLabel(title.upper())
        self.lbl_title.setStyleSheet("color: #8fa0b5; font-size: 10px; font-weight: bold; letter-spacing: 1px;")
        layout.addWidget(self.lbl_title)

        self.lbl_value = QLabel(initial_value)
        self.lbl_value.setStyleSheet(f"color: {accent_color}; font-size: 16px; font-weight: bold; font-family: Consolas, monospace;")
        layout.addWidget(self.lbl_value)

    def set_value(self, text: str):
        self.lbl_value.setText(text)


class LiveConvergenceCanvas(FigureCanvas):
    """4-Panel Matplotlib telemetry dashboard."""
    def __init__(self, parent=None, width=9, height=6, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.fig.patch.set_facecolor("#0f141c")
        super().__init__(self.fig)
        self.setParent(parent)

        self.ax_fit = self.fig.add_subplot(2, 2, 1)
        self.ax_dist = self.fig.add_subplot(2, 2, 2)
        self.ax_safety = self.fig.add_subplot(2, 2, 3)
        self.ax_genes = self.fig.add_subplot(2, 2, 4)

        self.fig.tight_layout(pad=2.8)
        self._format_axes()

    def _format_axes(self):
        for ax in [self.ax_fit, self.ax_dist, self.ax_safety, self.ax_genes]:
            ax.set_facecolor("#151d27")
            ax.tick_params(colors="#8fa0b5", labelsize=8)
            for spine in ax.spines.values():
                spine.set_color("#223145")
            ax.grid(True, linestyle="--", alpha=0.35, color="#2f4058")

    def update_plots(self, history_records: List[Dict[str, Any]], current_summary: Optional[Dict[str, Any]]):
        if not history_records:
            for ax in [self.ax_fit, self.ax_dist, self.ax_safety, self.ax_genes]:
                ax.cla()
            self._format_axes()
            self.ax_fit.set_title("Waiting for Generation 0 Telemetry...", color="#8fa0b5", fontsize=9, pad=6)
            self.ax_dist.set_title("Waiting for Distance Data...", color="#8fa0b5", fontsize=9, pad=6)
            self.ax_safety.set_title("Waiting for Survival Data...", color="#8fa0b5", fontsize=9, pad=6)
            self.ax_genes.set_title("Waiting for Parameter Data...", color="#8fa0b5", fontsize=9, pad=6)
            self.fig.tight_layout(pad=2.5)
            self.draw()
            return

        gens = []
        gen_to_fits = {}
        gen_to_dist = {}
        gen_to_speed = {}
        gen_to_colls = {}
        gen_to_surv = {}

        for r in history_records:
            try:
                g = int(r["generation"])
                fit = float(r["fitness"])
                dist = float(r["distance_m"])
                spd = float(r["mean_speed_mps"])
                colls = int(r["collisions"])
                stuck = str(r.get("stuck", "False")).lower() in ("true", "1")

                if g not in gen_to_fits:
                    gen_to_fits[g] = []
                    gen_to_dist[g] = []
                    gen_to_speed[g] = []
                    gen_to_colls[g] = []
                    gen_to_surv[g] = []
                    gens.append(g)

                gen_to_fits[g].append(fit)
                gen_to_dist[g].append(dist)
                gen_to_speed[g].append(spd)
                gen_to_colls[g].append(colls)
                gen_to_surv[g].append(1 if (colls == 0 and not stuck) else 0)
            except Exception:
                continue

        if not gens:
            return

        unique_gens = sorted(list(set(gens)))
        best_fits = [max(gen_to_fits[g]) for g in unique_gens]
        mean_fits = [float(np.mean(gen_to_fits[g])) for g in unique_gens]
        max_dists = [max(gen_to_dist[g]) for g in unique_gens]
        mean_speeds = [float(np.mean(gen_to_speed[g])) for g in unique_gens]
        surv_pcts = [float(np.mean(gen_to_surv[g])) * 100.0 for g in unique_gens]

        all_time_bests = []
        cur_peak = -1e9
        for b in best_fits:
            cur_peak = max(cur_peak, b)
            all_time_bests.append(cur_peak)

        self.ax_fit.cla()
        self.ax_dist.cla()
        self.ax_safety.cla()
        self.ax_genes.cla()
        self._format_axes()

        # Plot 1: Fitness Convergence
        self.ax_fit.set_title("Fitness Convergence", color="#00e5ff", fontsize=9, fontweight="bold", pad=6)
        self.ax_fit.plot(unique_gens, all_time_bests, color="#ffb300", linewidth=1.8, label="All-Time Peak")
        self.ax_fit.plot(unique_gens, best_fits, color="#76ff03", linewidth=1.4, marker="o", markersize=3, label="Gen Best")
        self.ax_fit.plot(unique_gens, mean_fits, color="#00e5ff", linestyle="--", linewidth=1.0, label="Gen Mean")
        self.ax_fit.set_xlabel("Generation", color="#8fa0b5", fontsize=8)
        self.ax_fit.set_ylabel("Fitness Score", color="#8fa0b5", fontsize=8)
        self.ax_fit.legend(facecolor="#17202c", edgecolor="#223145", labelcolor="#e0e6ed", fontsize=7, loc="lower right")

        # Plot 2: Distance Progression
        self.ax_dist.set_title("Peak Distance Traveled (20s)", color="#76ff03", fontsize=9, fontweight="bold", pad=6)
        self.ax_dist.plot(unique_gens, max_dists, color="#00e5ff", linewidth=1.8, marker="s", markersize=3)
        self.ax_dist.axhline(16.5, color="#ffb300", linestyle=":", linewidth=1.0, label="1 Full Lap (16.5m)")
        self.ax_dist.axhline(33.0, color="#76ff03", linestyle=":", linewidth=1.0, label="2 Full Laps (33.0m)")
        self.ax_dist.set_xlabel("Generation", color="#8fa0b5", fontsize=8)
        self.ax_dist.set_ylabel("Distance (m)", color="#8fa0b5", fontsize=8)
        self.ax_dist.legend(facecolor="#17202c", edgecolor="#223145", labelcolor="#e0e6ed", fontsize=7, loc="upper left")

        # Plot 3: Survival Rate (%) & Mean Speed
        self.ax_safety.set_title("Population Survival Rate & Mean Speed", color="#ff9100", fontsize=9, fontweight="bold", pad=6)
        self.ax_safety.bar(unique_gens, surv_pcts, color="#26a69a", alpha=0.45, width=0.6, label="Survival Rate %")
        self.ax_safety.set_xlabel("Generation", color="#8fa0b5", fontsize=8)
        self.ax_safety.set_ylabel("Survival Rate (%)", color="#8fa0b5", fontsize=8)
        self.ax_safety.set_ylim(0, 105)

        ax_speed = self.ax_safety.twinx()
        ax_speed.set_ylabel("Mean Speed (m/s)", color="#ff5252", fontsize=8)
        ax_speed.plot(unique_gens, mean_speeds, color="#ff5252", linewidth=1.5, marker="^", markersize=3, label="Speed")
        ax_speed.tick_params(colors="#ff5252", labelsize=8)
        ax_speed.spines["right"].set_color("#ff5252")

        # Plot 4: Key Parameter Allocation
        self.ax_genes.set_title("Active Champion Parameter Allocation", color="#e040fb", fontsize=9, fontweight="bold", pad=6)
        if current_summary and "best_genome_genes" in current_summary:
            genes = current_summary["best_genome_genes"]
            labels = ["Friction", "C-Space", "Rev Speed", "Center %", "Straight %", "Left %"]
            mu = (genes.get("mu_friction", 0.68) - 0.55) / (0.80 - 0.55)
            r_inf = (genes.get("r_inflation_m", 0.095) - 0.080) / (0.130 - 0.080)
            rev_v = (genes.get("reverse_speed", 0.85) - 0.40) / (1.50 - 0.40)
            r_c = genes.get("ratio_center", 0.35)
            r_s = genes.get("ratio_straight", 0.20)
            r_l = genes.get("ratio_left", 0.25)
            vals = [mu, r_inf, rev_v, r_c, r_s, r_l]

            colors = ["#00e5ff", "#76ff03", "#ff5252", "#ffb300", "#e040fb", "#26a69a"]
            self.ax_genes.bar(labels, vals, color=colors, width=0.55, edgecolor="#223145")
            self.ax_genes.set_ylim(0, 1.0)
            self.ax_genes.set_ylabel("Normalized Scale [0..1]", color="#8fa0b5", fontsize=8)
            self.ax_genes.tick_params(axis="x", labelrotation=25)

        self.fig.tight_layout(pad=2.5)
        self.draw()


class GALiveGUI(QMainWindow):
    """Main Window displaying the full per-generation optimization state."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Folkrace MPPI Genetic Algorithm - Generation Telemetry")

        screen = QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            right_w = avail.width() // 2
            right_h = avail.height()
            right_x = avail.x() + right_w
            right_y = avail.y()
            self.setGeometry(right_x, right_y, right_w, right_h)
        else:
            self.setGeometry(1280, 0, 1280, 1540)

        self.setStyleSheet("""
            QMainWindow { background-color: #0b0f14; }
            QLabel { color: #e0e6ed; }
            QGroupBox {
                border: 1px solid #223145;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
                color: #00e5ff;
                font-size: 11px;
                padding-top: 10px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QTableWidget {
                background-color: #121820;
                border: 1px solid #223145;
                border-radius: 6px;
                gridline-color: #1e2836;
                color: #e0e6ed;
                font-family: Consolas, monospace;
                font-size: 10px;
            }
            QHeaderView::section {
                background-color: #17202c;
                color: #8fa0b5;
                border: 1px solid #223145;
                padding: 4px;
                font-weight: bold;
                font-size: 10px;
            }
            QPushButton {
                background-color: #17202c;
                border: 1px solid #223145;
                border-radius: 6px;
                color: #e0e6ed;
                font-weight: bold;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: #223145;
                border-color: #00e5ff;
            }
        """)

        self.last_seen_gen = -1
        self.last_history_mtime = 0.0

        self._init_ui()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._check_for_updates)
        self.poll_timer.start(1500)

        self._check_for_updates(force=True)

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(12, 10, 12, 10)
        main_layout.setSpacing(10)

        # Top Row: 6 KPI Metric Cards
        kpi_layout = QHBoxLayout()
        kpi_layout.setSpacing(8)

        self.card_gen = MetricCard("Generation / Evals", "-- / --", "#00e5ff")
        self.card_fit = MetricCard("All-Time Best Fitness", "--", "#76ff03")
        self.card_lap = MetricCard("Fastest Lap Time", "--", "#ffb300")
        self.card_dist = MetricCard("Peak Distance Traveled", "--", "#00e5ff")
        self.card_surv = MetricCard("Gen Survival Rate", "--", "#26a69a")
        self.card_time = MetricCard("Elapsed / ETA", "--", "#e040fb")

        kpi_layout.addWidget(self.card_gen)
        kpi_layout.addWidget(self.card_fit)
        kpi_layout.addWidget(self.card_lap)
        kpi_layout.addWidget(self.card_dist)
        kpi_layout.addWidget(self.card_surv)
        kpi_layout.addWidget(self.card_time)
        main_layout.addLayout(kpi_layout)

        # Middle: Splitter between Graphs (Top) & Tables (Bottom)
        splitter = QSplitter(Qt.Vertical)

        self.canvas = LiveConvergenceCanvas(self, width=10, height=5, dpi=100)
        splitter.addWidget(self.canvas)

        lower_widget = QWidget()
        lower_layout = QHBoxLayout(lower_widget)
        lower_layout.setContentsMargins(0, 0, 0, 0)
        lower_layout.setSpacing(8)

        # Left Column: Candidate Table
        cand_group = QGroupBox("Current Generation Candidate Population")
        cand_layout = QVBoxLayout(cand_group)
        self.table_cands = QTableWidget()
        self.table_cands.setColumnCount(8)
        self.table_cands.setHorizontalHeaderLabels([
            "Rank", "Cand", "Fitness", "Dist (m)", "Speed", "Best Lap", "Collisions", "Status"
        ])
        self.table_cands.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_cands.verticalHeader().setVisible(False)
        cand_layout.addWidget(self.table_cands)
        lower_layout.addWidget(cand_group, stretch=3)

        # Right Column: Champion 27-Gene Breakdown
        gene_group = QGroupBox("Reigning Champion 27-Gene Chromosome")
        gene_layout = QVBoxLayout(gene_group)
        self.table_genes = QTableWidget()
        self.table_genes.setColumnCount(3)
        self.table_genes.setHorizontalHeaderLabels(["Gene Name", "Optimized Value", "Search Bound"])
        self.table_genes.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_genes.verticalHeader().setVisible(False)
        gene_layout.addWidget(self.table_genes)
        lower_layout.addWidget(gene_group, stretch=2)

        splitter.addWidget(lower_widget)
        splitter.setSizes([380, 280])
        main_layout.addWidget(splitter, stretch=1)

        # Bottom Status Bar
        bottom_bar = QHBoxLayout()
        self.lbl_status = QLabel("Ready - Waiting for Generation Update...")
        self.lbl_status.setStyleSheet("color: #76ff03; font-weight: bold; font-size: 11px;")
        bottom_bar.addWidget(self.lbl_status)
        bottom_bar.addStretch()

        btn_refresh = QPushButton("Force Refresh")
        btn_refresh.clicked.connect(lambda: self._check_for_updates(force=True))
        bottom_bar.addWidget(btn_refresh)

        btn_export = QPushButton("Export Champion")
        btn_export.clicked.connect(self._export_champion)
        bottom_bar.addWidget(btn_export)

        main_layout.addLayout(bottom_bar)

    def _check_for_updates(self, force: bool = False):
        summary = None
        new_gen_detected = False

        if os.path.exists(GEN_SUMMARY_FILE):
            try:
                with open(GEN_SUMMARY_FILE, "r", encoding="utf-8") as f:
                    summary = json.load(f)
                cur_g = int(summary.get("generation", -1))
                if cur_g != self.last_seen_gen or force:
                    self.last_seen_gen = cur_g
                    new_gen_detected = True
            except Exception:
                pass

        if os.path.exists(HISTORY_CSV_FILE):
            mtime = os.path.getmtime(HISTORY_CSV_FILE)
            if mtime != self.last_history_mtime:
                self.last_history_mtime = mtime
                new_gen_detected = True

        if new_gen_detected or force:
            self._render_dashboard(summary)

    def _render_dashboard(self, summary: Optional[Dict[str, Any]]):
        history_records = []
        if os.path.exists(HISTORY_CSV_FILE):
            try:
                with open(HISTORY_CSV_FILE, "r", encoding="utf-8") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        history_records.append(row)
            except Exception:
                pass

        if summary:
            gen_idx = summary.get("generation", 0)
            evals = summary.get("evaluations", 0)
            all_best = summary.get("all_time_best_fitness", 0.0)
            surv = summary.get("survived_count", 0)
            tot_gen = summary.get("total_in_gen", 1)
            pct = (surv / max(1, tot_gen)) * 100.0
            elapsed_s = summary.get("elapsed_seconds", 0.0)
            elapsed_h = elapsed_s / 3600.0

            best_m = summary.get("best_trial_metrics", {})
            best_lap = best_m.get("best_lap_time_s", 999.0)
            best_dist = best_m.get("distance_m", 0.0)
            best_spd = best_m.get("mean_speed_mps", 0.0)

            self.card_gen.set_value(f"Gen {gen_idx} ({evals} evals)")
            self.card_fit.set_value(f"{all_best:+.1f} pts")
            self.card_lap.set_value(f"{best_lap:.2f} s" if best_lap < 900 else "--")
            self.card_dist.set_value(f"{best_dist:.2f} m ({best_spd:.2f} m/s)")
            self.card_surv.set_value(f"{surv}/{tot_gen} ({pct:.0f}%)")
            self.card_time.set_value(f"{elapsed_h:.2f} hrs")

            self.lbl_status.setText(f"Updated: Generation {gen_idx} (Total Evals: {evals}) at {time.strftime('%H:%M:%S')}")
            self.lbl_status.setStyleSheet("color: #00e5ff; font-weight: bold;")

            candidates = summary.get("candidates", [])
            self.table_cands.setRowCount(len(candidates))
            for row_idx, cand in enumerate(candidates):
                rank_str = str(cand.get("rank", row_idx + 1))
                cand_id = f"#{cand.get('cand_idx', row_idx)}"
                fit_val = f"{cand.get('fitness', 0.0):+.1f}"
                dist_val = f"{cand.get('distance_m', 0.0):.2f}"
                spd_val = f"{cand.get('mean_speed_mps', 0.0):.2f}"
                lap_val = f"{cand.get('best_lap_time', 999.0):.2f}" if cand.get('best_lap_time', 999.0) < 900 else "--"
                coll_val = str(cand.get("collisions", 0))
                status = cand.get("status", "SURVIVED")

                self.table_cands.setItem(row_idx, 0, QTableWidgetItem(rank_str))
                self.table_cands.setItem(row_idx, 1, QTableWidgetItem(cand_id))
                self.table_cands.setItem(row_idx, 2, QTableWidgetItem(fit_val))
                self.table_cands.setItem(row_idx, 3, QTableWidgetItem(dist_val))
                self.table_cands.setItem(row_idx, 4, QTableWidgetItem(spd_val))
                self.table_cands.setItem(row_idx, 5, QTableWidgetItem(lap_val))
                self.table_cands.setItem(row_idx, 6, QTableWidgetItem(coll_val))

                status_item = QTableWidgetItem(status)
                if status == "SURVIVED":
                    status_item.setForeground(QColor("#76ff03"))
                elif status == "CRASH":
                    status_item.setForeground(QColor("#ff5252"))
                else:
                    status_item.setForeground(QColor("#ffb300"))
                self.table_cands.setItem(row_idx, 7, status_item)

            raw_genes = summary.get("best_genome_genes", {})
            valid_gene_names = [
                "H", "K", "dt", "lambda_temp", "ar1_alpha", "gamma_discount",
                "mu_friction", "w_cte", "w_heading", "w_speed", "w_hazard",
                "w_prob_costmap", "w_unknown", "w_steer_rate", "w_accel_rate",
                "w_terminal", "sigma_steer", "sigma_speed", "lognorm_shape",
                "ratio_center", "ratio_straight", "ratio_left", "pure_pursuit_lookahead",
                "r_inflation_m", "w_reverse", "reverse_speed", "t_rev_steps"
            ]
            genes = {k: raw_genes[k] for k in valid_gene_names if k in raw_genes}
            if genes:
                self.table_genes.setRowCount(len(genes))
                for g_idx, (g_name, g_val) in enumerate(genes.items()):
                    val_str = f"{g_val:.4f}" if isinstance(g_val, float) else str(g_val)
                    self.table_genes.setItem(g_idx, 0, QTableWidgetItem(g_name))
                    self.table_genes.setItem(g_idx, 1, QTableWidgetItem(val_str))
                    self.table_genes.setItem(g_idx, 2, QTableWidgetItem(self._get_gene_bound_str(g_name)))
        else:
            self.card_gen.set_value("-- / --")
            self.card_fit.set_value("--")
            self.card_lap.set_value("--")
            self.card_dist.set_value("--")
            self.card_surv.set_value("--")
            self.card_time.set_value("--")
            self.table_cands.setRowCount(0)
            self.table_genes.setRowCount(0)
            self.lbl_status.setText("Ready - Waiting for Generation 0 evaluations to complete...")
            self.lbl_status.setStyleSheet("color: #76ff03; font-weight: bold;")

        self.canvas.update_plots(history_records, summary)

    def _get_gene_bound_str(self, gene_name: str) -> str:
        bounds_map = {
            "H": "[18, 35]", "K": "[128, 512]", "dt": "[0.015, 0.035]",
            "lambda_temp": "[0.05, 1.20]", "ar1_alpha": "[0.40, 0.95]",
            "gamma_discount": "[0.80, 1.00]", "mu_friction": "[0.55, 0.80]",
            "w_cte": "[2.0, 30.0]", "w_heading": "[1.0, 20.0]", "w_speed": "[0.5, 15.0]",
            "w_hazard": "[20k, 500k]", "w_prob_costmap": "[20.0, 150.0]", "w_unknown": "[0.0, 30.0]",
            "w_steer_rate": "[0.10, 3.0]", "w_accel_rate": "[0.05, 3.0]", "w_terminal": "[2.0, 15.0]",
            "sigma_steer": "[0.04, 0.25]", "sigma_speed": "[0.10, 0.80]", "lognorm_shape": "[0.10, 0.60]",
            "ratio_center": "[0.15, 0.60]", "ratio_straight": "[0.05, 0.35]", "ratio_left": "[0.10, 0.40]",
            "pure_pursuit_lookahead": "[0.20, 0.65]", "r_inflation_m": "[0.080, 0.130]",
            "w_reverse": "[50.0, 800.0]", "reverse_speed": "[0.40, 1.50]", "t_rev_steps": "[2, 7]"
        }
        return bounds_map.get(gene_name, "--")

    def _export_champion(self):
        if os.path.exists(WINNING_GENOME_FILE):
            try:
                with open(WINNING_GENOME_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                out_name = f"champion_export_{time.strftime('%Y%m%d_%H%M%S')}.json"
                out_path = os.path.join(WORKSPACE_ROOT, out_name)
                with open(out_path, "w", encoding="utf-8") as out_f:
                    json.dump(data, out_f, indent=2)
                self.lbl_status.setText(f"Exported Champion to {out_name}")
                self.lbl_status.setStyleSheet("color: #76ff03; font-weight: bold;")
            except Exception as e:
                self.lbl_status.setText(f"Export failed: {e}")
                self.lbl_status.setStyleSheet("color: #ff5252; font-weight: bold;")


def launch_ga_gui():
    app = QApplication.instance() or QApplication(sys.argv)
    gui = GALiveGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_ga_gui()
