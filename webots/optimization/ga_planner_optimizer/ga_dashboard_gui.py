"""
================================================================================
  HEADLESS GENETIC ALGORITHM REAL-TIME DASHBOARD GUI
================================================================================
  Features:
    - Real-time generational monitoring with embedded dark-themed Matplotlib plots
    - Pause / Resume / Stop execution controls
    - Live multi-planner leaderboard & parameter inspector tabs
    - Multi-threaded execution keeping UI smooth and responsive
================================================================================
"""

import os
import sys
import time
import json
import csv
import threading
from typing import Dict, List, Any, Optional

import numpy as np

# Discover controller paths
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNERS_DIR = os.path.join(CURR_DIR, "planners")
WORKSPACE_ROOT = os.path.dirname(CURR_DIR)

for p in [CURR_DIR, PLANNERS_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QSpinBox,
    QDoubleSpinBox,
    QCheckBox,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QTabWidget,
    QTextEdit,
    QGroupBox,
    QFrame,
    QProgressBar,
    QSplitter,
)

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from ga_planner_config import (
    NUM_CPU_CORES,
    POPULATION_SIZE,
    NUM_GENERATIONS,
    EVALUATION_DURATION_S,
    ACTIVE_PLANNER,
    ALGORITHM,
    BasePlannerGenome,
    get_genome_class,
    compute_efficiency_multiplier,
)
from webots/optimization/ga_optimizer import GAController, run_genetic_algorithm, HISTORY_CSV_FILE, BEST_GENOME_FILE, CHECKPOINT_FILE

try:
    from cpp_planner_option3_septic import CppSepticSplineTractionProfiler, DLL_PATH as SEPTIC_DLL_PATH
    CPP_SEPTIC_AVAILABLE = os.path.exists(SEPTIC_DLL_PATH)
except Exception:
    CPP_SEPTIC_AVAILABLE = False
    SEPTIC_DLL_PATH = ""


PLANNER_COLORS = {
    "OPTION_1_BVP": "#10B981",    # Emerald Green
    "OPTION_2_SL_MPC": "#3B82F6",  # Electric Blue
    "OPTION_3_SEPTIC": "#EC4899",  # Neon Pink
}


class GAWorkerThread(QThread):
    """Background execution thread running the parallel GA / CMA-ES optimizer."""
    gen_completed = pyqtSignal(int, str, dict, dict, list, float)
    status_changed = pyqtSignal(str)
    finished_all = pyqtSignal()

    def __init__(
        self,
        planner_name: str,
        cores: int,
        pop_size: int,
        generations: int,
        duration: float,
        resume: bool,
        preview: bool = False,
        algorithm: str = ALGORITHM,
    ):
        super().__init__()
        self.planner_name = planner_name
        self.cores = cores
        self.pop_size = pop_size
        self.generations = generations
        self.duration = duration
        self.resume = resume
        self.preview = preview
        self.algorithm = algorithm
        self.controller = GAController()
        self.controller.on_generation_done = self._on_gen_done
        self.controller.on_status_change = self._on_status_change

    def _on_gen_done(self, gen: int, planner: str, metrics: dict, genome: dict, fits: list, elapsed: float):
        self.gen_completed.emit(gen, planner, metrics, genome, fits, elapsed)

    def _on_status_change(self, status: str):
        self.status_changed.emit(status)

    def run(self):
        try:
            self.status_changed.emit("RUNNING")
            run_genetic_algorithm(
                planner_name=self.planner_name,
                num_cores=self.cores,
                pop_size=self.pop_size,
                num_generations=self.generations,
                eval_duration_s=self.duration,
                resume=self.resume,
                preview_rendered=self.preview,
                controller=self.controller,
                algorithm=self.algorithm,
            )
            if self.controller.abort_requested:
                self.status_changed.emit("ABORTED")
            elif self.controller.stop_requested:
                self.status_changed.emit("STOPPED & SAVED")
            else:
                self.status_changed.emit("COMPLETED")
        except Exception as e:
            if self.controller and self.controller.abort_requested:
                self.status_changed.emit("ABORTED")
            else:
                print(f"[Worker Thread Error] {e}", flush=True)
                self.status_changed.emit(f"ERROR: {str(e)[:30]}")
        finally:
            self.finished_all.emit()

    def pause(self):
        self.controller.pause()

    def resume_opt(self):
        self.controller.resume()

    def stop_and_save(self):
        self.controller.stop_and_save()

    def abort(self):
        self.controller.abort()

    def stop(self):
        self.controller.stop_and_save()


class MatplotlibCanvas(FigureCanvas):
    """Dark-themed multi-curve real-time Matplotlib canvas."""

    def __init__(self, parent=None, width=9, height=5, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor="#0F172A")
        super().__init__(self.fig)
        self.setParent(parent)

        self.axs = self.fig.subplots(2, 2)
        self.fig.tight_layout(pad=3.0)
        self._setup_dark_axes()

    def _setup_dark_axes(self):
        for ax_row in self.axs:
            for ax in ax_row:
                ax.set_facecolor("#1E293B")
                ax.tick_params(colors="#94A3B8", labelsize=8)
                for spine in ax.spines.values():
                    spine.set_color("#334155")
                ax.grid(True, linestyle=":", alpha=0.3, color="#64748B")

    def update_plots(self, data_by_planner: Dict[str, Dict[str, List[Any]]]):
        for ax_row in self.axs:
            for ax in ax_row:
                ax.clear()
        self._setup_dark_axes()

        # Titles
        self.axs[0, 0].set_title("Fitness Score Progression", color="#F8FAFC", fontsize=10, fontweight="bold")
        self.axs[0, 1].set_title("Distance in 20.0s (m)", color="#F8FAFC", fontsize=10, fontweight="bold")
        self.axs[1, 0].set_title("Average Velocity (m/s)", color="#F8FAFC", fontsize=10, fontweight="bold")
        self.axs[1, 1].set_title("Fatal Failure / Crash Count", color="#F8FAFC", fontsize=10, fontweight="bold")

        # Reference lines on distance plot
        self.axs[0, 1].axhline(y=14.109, color="#F59E0B", linestyle="--", alpha=0.7, label="1 Lap (14.1m)")
        self.axs[0, 1].axhline(y=28.218, color="#EF4444", linestyle=":", alpha=0.7, label="2 Laps (28.2m)")

        n_planners = len(data_by_planner)
        for i, (p_name, p_data) in enumerate(data_by_planner.items()):
            c_main = PLANNER_COLORS.get(p_name, "#38BDF8")
            gens = p_data["gen"]
            short_name = p_name.replace("OPTION_", "Opt").replace("_BVP", "-BVP").replace("_SL_MPC", "-MPC").replace("_SEPTIC", "-Septic")

            # 1. Fitness
            self.axs[0, 0].plot(gens, p_data["best_fit"], "-o", color=c_main, label=f"{short_name}", markersize=4, linewidth=1.8)

            # 2. Distance
            self.axs[0, 1].plot(gens, p_data["dist"], "-s", color=c_main, label=f"{short_name}", markersize=4, linewidth=1.8)

            # 3. Speed
            self.axs[1, 0].plot(gens, p_data["speed"], "-^", color=c_main, label=f"{short_name}", markersize=4, linewidth=1.8)

            # 4. Crashes
            width = 0.25
            offset = (i - (n_planners - 1) / 2.0) * width
            self.axs[1, 1].bar(np.array(gens) + offset, p_data["crashes"], width=width, color=c_main, alpha=0.8, label=f"{short_name}")

        for ax_row in self.axs:
            for ax in ax_row:
                if ax.has_data():
                    ax.legend(facecolor="#0F172A", edgecolor="#334155", labelcolor="#E2E8F0", fontsize=7)

        self.fig.tight_layout(pad=2.2)
        self.draw()


class GADashboardGUI(QMainWindow):
    """Main Application Window for Headless GA Path Planner Optimization."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Autonomous ToF Path Planner - Genetic Algorithm Dashboard")
        self.resize(1380, 880)
        self.setMinimumSize(1100, 700)

        self.worker: Optional[GAWorkerThread] = None
        self.data_by_planner: Dict[str, Dict[str, List[Any]]] = {}
        self.champions: Dict[str, Dict[str, Any]] = {}
        self.start_time = 0.0

        self._setup_dark_theme()
        self._init_ui()
        self._load_initial_champion_previews()

    def _setup_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0F172A;
            }
            QWidget {
                background-color: #0F172A;
                color: #F8FAFC;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QGroupBox {
                background-color: #1E293B;
                border: 1px solid #334155;
                border-radius: 8px;
                margin-top: 12px;
                font-weight: bold;
                color: #38BDF8;
                padding-top: 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
            }
            QLabel {
                color: #CBD5E1;
            }
            QPushButton {
                background-color: #334155;
                color: #F8FAFC;
                border: 1px solid #475569;
                border-radius: 6px;
                padding: 6px 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #475569;
            }
            QPushButton:pressed {
                background-color: #1E293B;
            }
            QPushButton#btnStart {
                background-color: #059669;
                border-color: #10B981;
            }
            QPushButton#btnStart:hover {
                background-color: #10B981;
            }
            QPushButton#btnPause {
                background-color: #D97706;
                border-color: #F59E0B;
            }
            QPushButton#btnPause:hover {
                background-color: #F59E0B;
            }
            QPushButton#btnResume {
                background-color: #2563EB;
                border-color: #3B82F6;
            }
            QPushButton#btnStop {
                background-color: #D97706;
                border-color: #F59E0B;
            }
            QPushButton#btnStop:hover {
                background-color: #F59E0B;
            }
            QPushButton#btnAbort {
                background-color: #991B1B;
                border-color: #DC2626;
            }
            QPushButton#btnAbort:hover {
                background-color: #DC2626;
            }
            QComboBox, QSpinBox, QDoubleSpinBox {
                background-color: #1E293B;
                border: 1px solid #475569;
                border-radius: 5px;
                padding: 4px 8px;
                color: #F8FAFC;
            }
            QTableWidget {
                background-color: #1E293B;
                border: 1px solid #334155;
                border-radius: 6px;
                gridline-color: #334155;
                color: #E2E8F0;
                selection-background-color: #334155;
            }
            QHeaderView::section {
                background-color: #0F172A;
                color: #38BDF8;
                font-weight: bold;
                border: 1px solid #334155;
                padding: 4px;
            }
            QTabWidget::pane {
                border: 1px solid #334155;
                background-color: #1E293B;
                border-radius: 6px;
            }
            QTabBar::tab {
                background-color: #0F172A;
                color: #94A3B8;
                border: 1px solid #334155;
                padding: 6px 14px;
                margin-right: 2px;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }
            QTabBar::tab:selected {
                background-color: #1E293B;
                color: #38BDF8;
                font-weight: bold;
            }
            QTextEdit {
                background-color: #0F172A;
                border: 1px solid #334155;
                border-radius: 5px;
                color: #A7F3D0;
                font-family: 'Consolas', monospace;
                font-size: 11px;
            }
            QProgressBar {
                background-color: #1E293B;
                border: 1px solid #334155;
                border-radius: 5px;
                text-align: center;
                color: #F8FAFC;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #10B981;
                border-radius: 4px;
            }
        """)

    def _init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setContentsMargins(12, 10, 12, 10)
        root_layout.setSpacing(10)

        # ----------------------------------------------------------------------
        # 1. HEADER & TOP CONTROL BAR
        # ----------------------------------------------------------------------
        header_frame = QFrame()
        header_frame.setStyleSheet("background-color: #1E293B; border: 1px solid #334155; border-radius: 8px; padding: 6px;")
        header_layout = QHBoxLayout(header_frame)
        header_layout.setContentsMargins(10, 6, 10, 6)

        # Title & Badge
        title_box = QVBoxLayout()
        title_lbl = QLabel("AUTONOMOUS TOF PATH PLANNER OPTIMIZER")
        title_lbl.setStyleSheet("font-size: 15px; font-weight: bold; color: #38BDF8;")
        sub_lbl = QLabel("Headless Multi-Core Parallel Genetic Algorithm Dashboard")
        sub_lbl.setStyleSheet("font-size: 11px; color: #94A3B8;")
        title_box.addWidget(title_lbl)
        title_box.addWidget(sub_lbl)
        header_layout.addLayout(title_box)

        header_layout.addSpacing(20)

        # Status Pill
        self.status_pill = QLabel(" READY ")
        self.status_pill.setAlignment(Qt.AlignCenter)
        self.status_pill.setStyleSheet("background-color: #334155; color: #94A3B8; font-weight: bold; font-size: 12px; border-radius: 12px; padding: 4px 12px;")
        header_layout.addWidget(self.status_pill)

        # C++ Engine Status Badge
        if CPP_SEPTIC_AVAILABLE:
            self.cpp_badge = QLabel(" C++ SEPTIC ENGINE: ACTIVE (AVX2, ~3.5ms) ")
            self.cpp_badge.setStyleSheet("background-color: #064E3B; color: #34D399; font-weight: bold; font-size: 11px; border: 1px solid #059669; border-radius: 12px; padding: 4px 10px;")
            self.cpp_badge.setToolTip(f"Native 64-bit DLL Loaded:\n{SEPTIC_DLL_PATH}\nExecution Budget: ~3.5 ms\nCandidates: 210-280 per cycle\nSamples: 36 points\n28 GA Tunable Genes (Anti-Oscillation & Speed Tuning)")
        else:
            self.cpp_badge = QLabel(" C++ ENGINE: NOT DETECTED (PYTHON FALLBACK) ")
            self.cpp_badge.setStyleSheet("background-color: #451A03; color: #FBBF24; font-weight: bold; font-size: 11px; border: 1px solid #D97706; border-radius: 12px; padding: 4px 10px;")
            self.cpp_badge.setToolTip("Running Python fallback")
        header_layout.addWidget(self.cpp_badge)

        header_layout.addStretch()

        # 1. Execution Mode Selector (Single Planner vs All Planners)
        mode_box = QVBoxLayout()
        mode_lbl = QLabel("Execution Mode:")
        mode_lbl.setStyleSheet("font-size: 10px; color: #94A3B8;")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "Single Planner Only",
            "ALL Planners (Interleaved 3 Planners)",
        ])
        self.mode_combo.setToolTip("Choose whether to optimize only one chosen path planner or run all 3 interleaved each generation")
        mode_box.addWidget(mode_lbl)
        mode_box.addWidget(self.mode_combo)
        header_layout.addLayout(mode_box)

        # 2. Planner Target Selector
        p_box = QVBoxLayout()
        p_lbl = QLabel("Target Planner:")
        p_lbl.setStyleSheet("font-size: 10px; color: #94A3B8;")
        self.planner_combo = QComboBox()
        self.planner_combo.addItem("Option 3: Septic Spline (C++ Native AVX2, ~3.5ms)", "OPTION_3_SEPTIC")
        self.planner_combo.addItem("Option 1: BVP Dynamic State Lattice", "OPTION_1_BVP")
        self.planner_combo.addItem("Option 2: Curvilinear SL-MPC (OSQP)", "OPTION_2_SL_MPC")
        self.planner_combo.setToolTip("Select which path planner to optimize when in Single Planner mode")
        p_box.addWidget(p_lbl)
        p_box.addWidget(self.planner_combo)
        header_layout.addLayout(p_box)

        # 3. Optimizer Algorithm Selector (CMA-ES vs Standard GA)
        algo_box = QVBoxLayout()
        algo_lbl = QLabel("Optimizer Engine:")
        algo_lbl.setStyleSheet("font-size: 10px; color: #94A3B8;")
        self.algo_combo = QComboBox()
        self.algo_combo.addItem("CMA-ES (Covariance Matrix Adaptation)", "CMA_ES")
        self.algo_combo.addItem("Standard Genetic Algorithm (GA)", "GA")
        if str(ALGORITHM).upper().startswith("CMA"):
            self.algo_combo.setCurrentIndex(0)
        else:
            self.algo_combo.setCurrentIndex(1)
        self.algo_combo.setToolTip("CMA-ES adapts continuous covariance matrices for sample efficiency; GA uses tournament selection/crossover/mutation.")
        algo_box.addWidget(algo_lbl)
        algo_box.addWidget(self.algo_combo)
        header_layout.addLayout(algo_box)

        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)

        # Cores Selector
        cores_box = QVBoxLayout()
        cores_lbl = QLabel("CPU Workers:")
        cores_lbl.setStyleSheet("font-size: 10px; color: #94A3B8;")
        self.cores_spin = QSpinBox()
        self.cores_spin.setRange(1, 32)
        self.cores_spin.setValue(NUM_CPU_CORES)
        cores_box.addWidget(cores_lbl)
        cores_box.addWidget(self.cores_spin)
        header_layout.addLayout(cores_box)

        # Population Size
        pop_box = QVBoxLayout()
        pop_lbl = QLabel("Population:")
        pop_lbl.setStyleSheet("font-size: 10px; color: #94A3B8;")
        self.pop_spin = QSpinBox()
        self.pop_spin.setRange(4, 128)
        self.pop_spin.setValue(POPULATION_SIZE)
        pop_box.addWidget(pop_lbl)
        pop_box.addWidget(self.pop_spin)
        header_layout.addLayout(pop_box)

        # Generations
        gen_box = QVBoxLayout()
        gen_lbl = QLabel("Generations:")
        gen_lbl.setStyleSheet("font-size: 10px; color: #94A3B8;")
        self.gen_spin = QSpinBox()
        self.gen_spin.setRange(1, 9999)
        self.gen_spin.setValue(NUM_GENERATIONS)
        gen_box.addWidget(gen_lbl)
        gen_box.addWidget(self.gen_spin)
        header_layout.addLayout(gen_box)

        # Resume Checkbox
        self.chk_resume = QCheckBox("Resume")
        self.chk_resume.setStyleSheet("color: #CBD5E1; font-weight: bold;")
        header_layout.addWidget(self.chk_resume)

        # 20s Rendered Visual Preview Checkbox
        self.chk_preview = QCheckBox("20s Rendered Preview")
        self.chk_preview.setStyleSheet("color: #38BDF8; font-weight: bold;")
        self.chk_preview.setToolTip("Runs 1 rendered 20-second trial in Webots GUI per planner before parallel headless GA.")
        header_layout.addWidget(self.chk_preview)

        header_layout.addSpacing(10)

        # Action Buttons
        self.btn_start = QPushButton("START GA")
        self.btn_start.setObjectName("btnStart")
        self.btn_start.clicked.connect(self.start_ga)

        self.btn_pause = QPushButton("PAUSE")
        self.btn_pause.setObjectName("btnPause")
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self.pause_ga)

        self.btn_resume = QPushButton("RESUME")
        self.btn_resume.setObjectName("btnResume")
        self.btn_resume.setEnabled(False)
        self.btn_resume.clicked.connect(self.resume_ga)

        self.btn_stop = QPushButton("STOP & SAVE")
        self.btn_stop.setObjectName("btnStop")
        self.btn_stop.setToolTip("Finishes current generation/evaluations smoothly, saves checkpoints, and stops.")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.stop_ga)

        self.btn_abort = QPushButton("ABORT")
        self.btn_abort.setObjectName("btnAbort")
        self.btn_abort.setToolTip("Immediately halts all processes, kills Webots instances, and does NOT save.")
        self.btn_abort.setEnabled(False)
        self.btn_abort.clicked.connect(self.abort_ga)

        header_layout.addWidget(self.btn_start)
        header_layout.addWidget(self.btn_pause)
        header_layout.addWidget(self.btn_resume)
        header_layout.addWidget(self.btn_stop)
        header_layout.addWidget(self.btn_abort)

        root_layout.addWidget(header_frame)

        # ----------------------------------------------------------------------
        # 2. PROGRESS BAR & STATS ROW
        # ----------------------------------------------------------------------
        progress_layout = QHBoxLayout()
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        progress_layout.addWidget(self.progress_bar, stretch=3)

        self.lbl_stats = QLabel("Gen: 0 / 30 | Best Distance: 0.00 m | Best Speed: 0.00 m/s | Elapsed: 0.0s")
        self.lbl_stats.setStyleSheet("font-size: 11px; color: #38BDF8; font-weight: bold;")
        progress_layout.addWidget(self.lbl_stats, stretch=2)

        root_layout.addLayout(progress_layout)

        # ----------------------------------------------------------------------
        # 3. SPLIT MAIN BODY: PLOTS (TOP/LEFT) + TABLE & PARAMS (BOTTOM/RIGHT)
        # ----------------------------------------------------------------------
        splitter = QSplitter(Qt.Vertical)

        # Top Splitter: Embedded Matplotlib Real-Time Canvas
        self.canvas = MatplotlibCanvas(self)
        splitter.addWidget(self.canvas)

        # Bottom Splitter: Table and Champion Tabs
        bottom_frame = QWidget()
        bottom_layout = QHBoxLayout(bottom_frame)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        bottom_layout.setSpacing(10)

        # Bottom Left: Generational History Table
        table_group = QGroupBox("Generational Performance Log (Updates Once Per Gen)")
        table_layout = QVBoxLayout(table_group)
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels([
            "Gen", "Planner", "Best Fit", "Mean Fit", "Dist (m)", "Speed (m/s)", "Crashes", "Time (s)"
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        table_layout.addWidget(self.table)
        bottom_layout.addWidget(table_group, stretch=3)

        # Bottom Right: Champion Genome Parameter Cards
        params_group = QGroupBox("Live Leaderboard & Champion Parameters")
        params_layout = QVBoxLayout(params_group)

        self.tabs = QTabWidget()
        self.txt_opt1 = QTextEdit()
        self.txt_opt1.setReadOnly(True)
        self.txt_opt2 = QTextEdit()
        self.txt_opt2.setReadOnly(True)
        self.txt_opt3 = QTextEdit()
        self.txt_opt3.setReadOnly(True)

        self.tabs.addTab(self.txt_opt3, "Option 3: C++ Septic (28 Genes)")
        self.tabs.addTab(self.txt_opt1, "Option 1 (BVP)")
        self.tabs.addTab(self.txt_opt2, "Option 2 (SL-MPC)")

        params_layout.addWidget(self.tabs)
        bottom_layout.addWidget(params_group, stretch=2)

        splitter.addWidget(bottom_frame)
        splitter.setSizes([450, 300])

        root_layout.addWidget(splitter)

    def _on_mode_changed(self, idx: int):
        """Toggles planner combobox availability based on Single vs All Planners mode."""
        is_single = (idx == 0)
        self.planner_combo.setEnabled(is_single)
        if is_single:
            self.planner_combo.setStyleSheet("color: #F8FAFC; background-color: #1E293B;")
        else:
            self.planner_combo.setStyleSheet("color: #64748B; background-color: #0F172A;")

    # --------------------------------------------------------------------------
    # CONTROLLER SLOTS
    # --------------------------------------------------------------------------
    def start_ga(self):
        is_single = (self.mode_combo.currentIndex() == 0)
        if is_single:
            p_name = str(self.planner_combo.currentData())
        else:
            p_name = "ALL"

        cores = self.cores_spin.value()
        pop_size = self.pop_spin.value()
        generations = self.gen_spin.value()
        resume = self.chk_resume.isChecked()
        preview = self.chk_preview.isChecked()
        algo_choice = str(self.algo_combo.currentData())

        self.data_by_planner.clear()
        self.table.setRowCount(0)
        self.start_time = time.time()

        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_resume.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_abort.setEnabled(True)
        self.mode_combo.setEnabled(False)
        self.planner_combo.setEnabled(False)
        self.algo_combo.setEnabled(False)
        self.cores_spin.setEnabled(False)
        self.pop_spin.setEnabled(False)
        self.gen_spin.setEnabled(False)
        self.chk_resume.setEnabled(False)
        self.chk_preview.setEnabled(False)

        self.worker = GAWorkerThread(
            planner_name=p_name,
            cores=cores,
            pop_size=pop_size,
            generations=generations,
            duration=EVALUATION_DURATION_S,
            resume=resume,
            preview=preview,
            algorithm=algo_choice,
        )
        self.worker.gen_completed.connect(self.on_generation_update)
        self.worker.status_changed.connect(self.on_status_change)
        self.worker.finished_all.connect(self.on_ga_finished)
        self.worker.start()

    def pause_ga(self):
        if self.worker:
            self.worker.pause()
            self.btn_pause.setEnabled(False)
            self.btn_resume.setEnabled(True)

    def resume_ga(self):
        if self.worker:
            self.worker.resume_opt()
            self.btn_pause.setEnabled(True)
            self.btn_resume.setEnabled(False)

    def stop_ga(self):
        """Smoothly completes current generation, saves checkpoint & champions, and stops."""
        if self.worker:
            self.worker.stop_and_save()
            self.btn_pause.setEnabled(False)
            self.btn_resume.setEnabled(False)
            self.btn_stop.setEnabled(False)

    def abort_ga(self):
        """Immediately force-kills all Webots processes, discards unsaved data, and terminates."""
        if self.worker:
            self.worker.abort()
            self.btn_pause.setEnabled(False)
            self.btn_resume.setEnabled(False)
            self.btn_stop.setEnabled(False)
            self.btn_abort.setEnabled(False)

    def on_status_change(self, status: str):
        self.status_pill.setText(f" {status} ")
        if status == "RUNNING":
            self.status_pill.setStyleSheet("background-color: #065F46; color: #34D399; font-weight: bold; font-size: 12px; border-radius: 12px; padding: 4px 12px;")
        elif status == "PAUSED":
            self.status_pill.setStyleSheet("background-color: #78350F; color: #FBBF24; font-weight: bold; font-size: 12px; border-radius: 12px; padding: 4px 12px;")
        elif status == "COMPLETED":
            self.status_pill.setStyleSheet("background-color: #1E3A8A; color: #60A5FA; font-weight: bold; font-size: 12px; border-radius: 12px; padding: 4px 12px;")
        elif "PREVIEW" in status:
            self.status_pill.setStyleSheet("background-color: #3730A3; color: #A5B4FC; font-weight: bold; font-size: 12px; border-radius: 12px; padding: 4px 12px;")
        elif "STOPPING" in status:
            self.status_pill.setStyleSheet("background-color: #78350F; color: #FDBA74; font-weight: bold; font-size: 12px; border-radius: 12px; padding: 4px 12px;")
        elif "ABORT" in status:
            self.status_pill.setStyleSheet("background-color: #7F1D1D; color: #FCA5A5; font-weight: bold; font-size: 12px; border-radius: 12px; padding: 4px 12px;")
        elif "STOPPED" in status:
            self.status_pill.setStyleSheet("background-color: #78350F; color: #FDBA74; font-weight: bold; font-size: 12px; border-radius: 12px; padding: 4px 12px;")

    def on_ga_finished(self):
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_resume.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.btn_abort.setEnabled(False)
        self.mode_combo.setEnabled(True)
        self._on_mode_changed(self.mode_combo.currentIndex())
        self.cores_spin.setEnabled(True)
        self.pop_spin.setEnabled(True)
        self.gen_spin.setEnabled(True)
        self.chk_resume.setEnabled(True)
        self.chk_preview.setEnabled(True)
        self.algo_combo.setEnabled(True)

    def on_generation_update(self, gen: int, planner: str, metrics: dict, genome: dict, fits: list, elapsed: float):
        # 1. Update Data Structure
        if planner not in self.data_by_planner:
            self.data_by_planner[planner] = {
                "gen": [],
                "best_fit": [],
                "mean_fit": [],
                "dist": [],
                "speed": [],
                "crashes": [],
            }

        best_fit = float(max(fits)) if fits else 0.0
        mean_fit = float(np.mean(fits)) if fits else 0.0
        dist_m = float(metrics.get("distance_m", 0.0))
        speed_mps = float(metrics.get("mean_speed_mps", 0.0))
        crashes = sum(1 for f in fits if f < -2000.0)

        self.data_by_planner[planner]["gen"].append(gen)
        self.data_by_planner[planner]["best_fit"].append(best_fit)
        self.data_by_planner[planner]["mean_fit"].append(mean_fit)
        self.data_by_planner[planner]["dist"].append(dist_m)
        self.data_by_planner[planner]["speed"].append(speed_mps)
        self.data_by_planner[planner]["crashes"].append(crashes)

        # 2. Update Table
        row_idx = self.table.rowCount()
        self.table.insertRow(row_idx)
        self.table.setItem(row_idx, 0, QTableWidgetItem(str(gen)))
        self.table.setItem(row_idx, 1, QTableWidgetItem(planner.replace("OPTION_", "Opt").replace("_BVP", "-BVP").replace("_SL_MPC", "-MPC").replace("_SEPTIC", "-Septic")))
        self.table.setItem(row_idx, 2, QTableWidgetItem(f"{best_fit:.1f}"))
        self.table.setItem(row_idx, 3, QTableWidgetItem(f"{mean_fit:.1f}"))
        self.table.setItem(row_idx, 4, QTableWidgetItem(f"{dist_m:.2f}"))
        self.table.setItem(row_idx, 5, QTableWidgetItem(f"{speed_mps:.2f}"))
        self.table.setItem(row_idx, 6, QTableWidgetItem(str(crashes)))
        self.table.setItem(row_idx, 7, QTableWidgetItem(f"{elapsed:.1f}"))
        self.table.scrollToBottom()

        # 3. Update Plots
        self.canvas.update_plots(self.data_by_planner)

        # 4. Update Champion JSON Card
        self.champions[planner] = {
            "gen": gen,
            "best_fit": best_fit,
            "dist_m": dist_m,
            "speed_mps": speed_mps,
            "genes": genome,
        }

        json_obj = {
            "generation": gen,
            "planner": planner,
            "fitness": round(best_fit, 1),
            "distance_m": round(dist_m, 2),
            "mean_speed_mps": round(speed_mps, 2),
            "genes": genome,
        }
        if "OPTION_3" in planner or "SEPTIC" in planner:
            json_obj["engine"] = "C++ Native AVX2 (septic_traction_profiler.dll)"
            compute_mult = compute_efficiency_multiplier(genome)
            json_obj["compute_efficiency_multiplier"] = round(compute_mult, 4)
            probe_pts = int((1.0 + float(genome.get("max_extra_targets", 2))) * float(genome.get("num_lat_offsets", 7)) * float(genome.get("num_head_offsets", 5)) * float(genome.get("num_blend_weights", 3)) * float(genome.get("probe_samples", 36)))
            json_obj["estimated_eval_points"] = probe_pts
            json_obj["genes_count"] = len(genome)

        json_str = json.dumps(json_obj, indent=2)

        if "OPTION_1" in planner or "BVP" in planner:
            self.txt_opt1.setPlainText(json_str)
        elif "OPTION_2" in planner or "MPC" in planner:
            self.txt_opt2.setPlainText(json_str)
        elif "OPTION_3" in planner or "SEPTIC" in planner:
            self.txt_opt3.setPlainText(json_str)

        # 5. Update Progress Bar & Overall Stats
        total_gens = self.gen_spin.value()
        pct = min(100, int((gen / max(1, total_gens)) * 100))
        self.progress_bar.setValue(pct)

        tot_elapsed = time.time() - self.start_time
        max_dist_overall = max([max(p["dist"]) for p in self.data_by_planner.values()] or [0.0])
        max_speed_overall = max([max(p["speed"]) for p in self.data_by_planner.values()] or [0.0])

        self.lbl_stats.setText(
            f"Gen: {gen} / {total_gens} | Max Dist: {max_dist_overall:.2f} m | Max Speed: {max_speed_overall:.2f} m/s | Elapsed: {tot_elapsed:.1f}s"
        )

    def _load_initial_champion_previews(self):
        """Pre-populates the parameter cards with champion genomes or defaults on startup."""
        for p_name, txt_widget in [
            ("OPTION_1_BVP", self.txt_opt1),
            ("OPTION_2_SL_MPC", self.txt_opt2),
            ("OPTION_3_SEPTIC", self.txt_opt3),
        ]:
            try:
                ckpt_path = os.path.join(CURR_DIR, f"best_genome_{p_name.lower()}.json")
                if not os.path.exists(ckpt_path):
                    ckpt_path = os.path.join(CURR_DIR, f"ga_checkpoint_{p_name.lower()}.json")
                genome_cls = get_genome_class(p_name)
                if os.path.exists(ckpt_path):
                    with open(ckpt_path, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    # Expand genes to current full specification
                    raw_genes = data.get("genes", data.get("best_genome", {}))
                    full_genes = genome_cls(raw_genes).to_dict()
                    data["genes"] = full_genes
                    data["total_genes"] = len(full_genes)
                    if "OPTION_3" in p_name:
                        data["engine"] = "C++ Native AVX2 (septic_traction_profiler.dll)"
                        data["compute_budget"] = "~3.5 ms execution window (210-280 candidates, 36 spatial points)"
                    txt_widget.setPlainText(json.dumps(data, indent=2))
                else:
                    default_genes = genome_cls().to_dict()
                    preview = {
                        "planner": p_name,
                        "status": "Ready for initial evaluation",
                        "total_genes": len(default_genes),
                        "genes": default_genes,
                    }
                    if "OPTION_3" in p_name:
                        preview["engine"] = "C++ Native AVX2 (septic_traction_profiler.dll)"
                        preview["compute_budget"] = "~3.5 ms execution window (210-280 candidates, 36 spatial points)"
                    txt_widget.setPlainText(json.dumps(preview, indent=2))
            except Exception as e:
                txt_widget.setPlainText(f"Error loading {p_name} preview: {e}")

    def closeEvent(self, event):
        """Cleanly abort and kill all Webots processes if window is closed."""
        if self.worker:
            self.worker.abort()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = GADashboardGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
