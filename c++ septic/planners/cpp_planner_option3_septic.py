"""
================================================================================
  OPTION 3 (C++ SCALED COMPUTE): G^3 SEPTIC SPLINE POOL + TRACTION PROFILER
  - High-performance native AVX2 C++ implementation via ctypes
  - Scalable compute parameters exposed for Genetic Algorithm (GA) optimization
  - Dense candidate exploration (e.g. 7x5x4 = 140+ candidates per target)
  - Fine spatial resolution (e.g. 40-64 samples per curve)
  - Multi-objective trajectory scoring: time, clearance, jerk, heading, steering effort
  - Tuned to target ~3.5 ms compute budget for maximum track performance
================================================================================
"""

import ctypes
import math
import os
import sys
from typing import Dict, List, Optional, Tuple, Union
import numpy as np

try:
    from .planner_config import (
        WHEELBASE,
        VEHICLE_MASS,
        UNDERSTEER_K_US,
        TIRE_MU,
        GRAVITY_G,
        MAX_STEERING_RAD,
        MAX_SERVO_RATE_RAD_S,
        OPT3_NUM_SPLINES,
        OPT3_ACCEL_MAX,
        OPT3_DECEL_MAX,
        OPT3_DS,
        OPT3_TOP_SPEED_MPS,
        OPT3_PROBE_SAMPLES,
        OPT3_MU_FRICTION,
        OPT3_V_ENTRY_MAX,
        OPT3_V_CRUISE_MIN,
        OPT3_NUM_LAT_OFFSETS,
        OPT3_LAT_OFFSET_MAX,
        OPT3_NUM_HEAD_OFFSETS,
        OPT3_HEAD_OFFSET_MAX,
        OPT3_NUM_BLEND_WEIGHTS,
        OPT3_BLEND_WEIGHTS,
        OPT3_MAX_EXTRA_TARGETS,
        OPT3_W_TIME,
        OPT3_W_CLEARANCE,
        OPT3_W_CURVATURE_SMOOTH,
        OPT3_W_CHORD_ALIGN,
        OPT3_W_STEER_EFFORT,
        OPT3_STEER_KP,
        OPT3_PP_LOOKAHEAD_GAIN,
        OPT3_INITIAL_SLEW_SCALE,
        OPT3_STRAIGHT_LOCK_DIST_M,
        OPT3_STRAIGHT_LOCK_SPEED_MPS,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
        DT,
        DT_PLANNER,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        VEHICLE_MASS,
        UNDERSTEER_K_US,
        TIRE_MU,
        GRAVITY_G,
        MAX_STEERING_RAD,
        MAX_SERVO_RATE_RAD_S,
        OPT3_NUM_SPLINES,
        OPT3_ACCEL_MAX,
        OPT3_DECEL_MAX,
        OPT3_DS,
        OPT3_TOP_SPEED_MPS,
        OPT3_PROBE_SAMPLES,
        OPT3_MU_FRICTION,
        OPT3_V_ENTRY_MAX,
        OPT3_V_CRUISE_MIN,
        OPT3_NUM_LAT_OFFSETS,
        OPT3_LAT_OFFSET_MAX,
        OPT3_NUM_HEAD_OFFSETS,
        OPT3_HEAD_OFFSET_MAX,
        OPT3_NUM_BLEND_WEIGHTS,
        OPT3_BLEND_WEIGHTS,
        OPT3_MAX_EXTRA_TARGETS,
        OPT3_W_TIME,
        OPT3_W_CLEARANCE,
        OPT3_W_CURVATURE_SMOOTH,
        OPT3_W_CHORD_ALIGN,
        OPT3_W_STEER_EFFORT,
        OPT3_STEER_KP,
        OPT3_PP_LOOKAHEAD_GAIN,
        OPT3_INITIAL_SLEW_SCALE,
        OPT3_STRAIGHT_LOCK_DIST_M,
        OPT3_STRAIGHT_LOCK_SPEED_MPS,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
        DT,
        DT_PLANNER,
    )

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_PATH = os.path.join(CURR_DIR, "septic_traction_profiler_v5.dll")
if not os.path.exists(DLL_PATH):
    DLL_PATH = os.path.join(CURR_DIR, "septic_traction_profiler_v4.dll")
if not os.path.exists(DLL_PATH):
    DLL_PATH = os.path.join(CURR_DIR, "septic_traction_profiler_v3.dll")
if not os.path.exists(DLL_PATH):
    DLL_PATH = os.path.join(CURR_DIR, "septic_traction_profiler_v2.dll")
if not os.path.exists(DLL_PATH):
    DLL_PATH = os.path.join(CURR_DIR, "septic_traction_profiler.dll")

if not os.path.exists(DLL_PATH):
    raise FileNotFoundError(f"Cannot find compiled DLL at {DLL_PATH}")

_lib = ctypes.CDLL(DLL_PATH)
CPP_AVAILABLE = True

if hasattr(_lib, "septic_profiler_set_initial_slew_scale"):
    _lib.septic_profiler_set_initial_slew_scale.argtypes = [ctypes.c_void_p, ctypes.c_float]
    _lib.septic_profiler_set_initial_slew_scale.restype = None

if hasattr(_lib, "septic_profiler_set_straight_lock"):
    _lib.septic_profiler_set_straight_lock.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    _lib.septic_profiler_set_straight_lock.restype = None

PROBE_CALLBACK_TYPE = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_int,
)

# V3 Interface with Scalable Compute, Multi-Objective Cost Weights, and GA Steering Gains
if hasattr(_lib, "septic_profiler_create_v3"):
    _lib.septic_profiler_create_v3.argtypes = [
        ctypes.c_float,  # wheelbase
        ctypes.c_float,  # vehicle_mass
        ctypes.c_float,  # understeer_k_us
        ctypes.c_float,  # tire_mu
        ctypes.c_float,  # gravity_g
        ctypes.c_float,  # max_steering_rad
        ctypes.c_float,  # max_servo_rate
        ctypes.c_float,  # accel_max
        ctypes.c_float,  # decel_max
        ctypes.c_float,  # ds_nominal
        ctypes.c_float,  # top_speed_mps
        ctypes.c_float,  # v_min_cruise
        ctypes.c_float,  # v_max_cruise
        ctypes.c_float,  # dt_loop
        ctypes.c_float,  # v_entry_max
        ctypes.c_float,  # v_cruise_min
        ctypes.c_float,  # mu_friction
        # Scalable Compute Parameters (GA)
        ctypes.c_int,    # probe_samples
        ctypes.c_int,    # num_lat_offsets
        ctypes.c_float,  # lat_offset_max
        ctypes.c_int,    # num_head_offsets
        ctypes.c_float,  # head_offset_max
        ctypes.c_int,    # num_blend_weights
        ctypes.POINTER(ctypes.c_float), # blend_weights array
        ctypes.c_int,    # max_extra_targets
        # Multi-Objective Cost Weights (GA)
        ctypes.c_float,  # w_time
        ctypes.c_float,  # w_clearance
        ctypes.c_float,  # w_curvature_smooth
        ctypes.c_float,  # w_chord_align
        ctypes.c_float,  # w_steer_effort
        ctypes.c_float,  # steer_kp
        ctypes.c_float,  # pp_lookahead_gain
        ctypes.c_void_p, # costmap_handle
        ctypes.c_void_p, # probe_fn
    ]
    _lib.septic_profiler_create_v3.restype = ctypes.c_void_p

# V2 Interface
if hasattr(_lib, "septic_profiler_create_v2"):
    _lib.septic_profiler_create_v2.argtypes = [
        ctypes.c_float,  # wheelbase
        ctypes.c_float,  # vehicle_mass
        ctypes.c_float,  # understeer_k_us
        ctypes.c_float,  # tire_mu
        ctypes.c_float,  # gravity_g
        ctypes.c_float,  # max_steering_rad
        ctypes.c_float,  # max_servo_rate
        ctypes.c_float,  # accel_max
        ctypes.c_float,  # decel_max
        ctypes.c_float,  # ds_nominal
        ctypes.c_float,  # top_speed_mps
        ctypes.c_float,  # v_min_cruise
        ctypes.c_float,  # v_max_cruise
        ctypes.c_float,  # dt_loop
        ctypes.c_float,  # v_entry_max
        ctypes.c_float,  # v_cruise_min
        ctypes.c_float,  # mu_friction
        # Scalable Compute Parameters (GA)
        ctypes.c_int,    # probe_samples
        ctypes.c_int,    # num_lat_offsets
        ctypes.c_float,  # lat_offset_max
        ctypes.c_int,    # num_head_offsets
        ctypes.c_float,  # head_offset_max
        ctypes.c_int,    # num_blend_weights
        ctypes.POINTER(ctypes.c_float), # blend_weights array
        ctypes.c_int,    # max_extra_targets
        # Multi-Objective Cost Weights (GA)
        ctypes.c_float,  # w_time
        ctypes.c_float,  # w_clearance
        ctypes.c_float,  # w_curvature_smooth
        ctypes.c_float,  # w_chord_align
        ctypes.c_float,  # w_steer_effort
        ctypes.c_void_p, # costmap_handle
        ctypes.c_void_p, # probe_fn
    ]
    _lib.septic_profiler_create_v2.restype = ctypes.c_void_p

_lib.septic_profiler_destroy.argtypes = [ctypes.c_void_p]
_lib.septic_profiler_destroy.restype = None

_lib.septic_profiler_set_costmap.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
_lib.septic_profiler_set_costmap.restype = None

_lib.septic_profiler_plan.argtypes = [
    ctypes.c_void_p,
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,  # xg, yg, theta_g, aperture_width
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,  # v_current, steer_current, yaw_rate, steer_rate
    ctypes.POINTER(ctypes.c_float), ctypes.c_int,                    # extra_targets_flat, num_extra_targets
    ctypes.POINTER(ctypes.c_float),                                  # out_cmd_steer_rad
    ctypes.POINTER(ctypes.c_float),                                  # out_cmd_speed_mps
    ctypes.POINTER(ctypes.c_float),                                  # out_best_lap_time
    ctypes.POINTER(ctypes.c_int),                                    # out_surviving_count
    ctypes.POINTER(ctypes.c_int),                                    # out_candidate_count
]
_lib.septic_profiler_plan.restype = ctypes.c_int

_lib.septic_profiler_get_best_line.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_int,
]
_lib.septic_profiler_get_best_line.restype = ctypes.c_int

_lib.septic_profiler_get_candidate_lines.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_int,
    ctypes.c_int,
]
_lib.septic_profiler_get_candidate_lines.restype = ctypes.c_int


class CppSepticSplineTractionProfiler:
    """
    High-Performance Native C++ Option 3 Septic Spline Planner & Dynamic Profiler.
    Scalable compute architecture configurable for Genetic Algorithm (GA) optimization.
    """

    @property
    def costmap(self):
        return getattr(self, "_costmap", None)

    @costmap.setter
    def costmap(self, cm):
        self._costmap = cm
        if hasattr(self, "_handle") and self._handle:
            cm_handle, probe_fn_ptr = self._resolve_costmap()
            if hasattr(_lib, "septic_profiler_set_costmap"):
                _lib.septic_profiler_set_costmap(self._handle, cm_handle, probe_fn_ptr)

    def __init__(
        self,
        costmap_engine: any = None,
        v_entry_max: float = OPT3_V_ENTRY_MAX,
        v_cruise_min: float = OPT3_V_CRUISE_MIN,
        mu_friction: float = OPT3_MU_FRICTION,
        # Compute Scaling Parameters (Exposed for GA Tuning)
        probe_samples: int = OPT3_PROBE_SAMPLES,
        num_lat_offsets: int = OPT3_NUM_LAT_OFFSETS,
        lat_offset_max: float = OPT3_LAT_OFFSET_MAX,
        num_head_offsets: int = OPT3_NUM_HEAD_OFFSETS,
        head_offset_max: float = OPT3_HEAD_OFFSET_MAX,
        num_blend_weights: int = OPT3_NUM_BLEND_WEIGHTS,
        blend_weights: Optional[List[float]] = None,
        max_extra_targets: int = OPT3_MAX_EXTRA_TARGETS,
        # Multi-Objective Cost Weights (Exposed for GA Tuning)
        w_time: float = OPT3_W_TIME,
        w_clearance: float = OPT3_W_CLEARANCE,
        w_curvature_smooth: float = OPT3_W_CURVATURE_SMOOTH,
        w_chord_align: float = OPT3_W_CHORD_ALIGN,
        w_steer_effort: float = OPT3_W_STEER_EFFORT,
        # Control parameters
        dt_loop: float = DT_PLANNER,
        **kwargs,
    ):
        self._costmap = costmap_engine
        self.v_entry_max = float(v_entry_max)
        self.v_cruise_min = float(v_cruise_min)
        self.mu_friction = float(mu_friction)
        self.dt_loop = float(dt_loop)

        # Scalable compute settings
        self.probe_samples = int(probe_samples)
        self.num_lat_offsets = int(num_lat_offsets)
        self.lat_offset_max = float(lat_offset_max)
        self.num_head_offsets = int(num_head_offsets)
        self.head_offset_max = float(head_offset_max)
        self.num_blend_weights = int(num_blend_weights)

        if blend_weights is not None:
            self.blend_weights = [float(b) for b in blend_weights]
            self.num_blend_weights = len(self.blend_weights)
        elif "blend_w0" in kwargs and "blend_w1" in kwargs and "blend_w2" in kwargs:
            self.blend_weights = [float(kwargs["blend_w0"]), float(kwargs["blend_w1"]), float(kwargs["blend_w2"])]
            self.num_blend_weights = 3
        else:
            # High-performance default 4-blend progression with 0.30 floor from planner_config.py
            self.blend_weights = [float(b) for b in OPT3_BLEND_WEIGHTS]
            self.num_blend_weights = len(self.blend_weights)

        self.max_extra_targets = int(max_extra_targets)

        # Multi-objective cost weights
        self.w_time = float(w_time)
        self.w_clearance = float(w_clearance)
        self.w_curvature_smooth = float(w_curvature_smooth)
        self.w_chord_align = float(w_chord_align)
        self.w_steer_effort = float(w_steer_effort)
        self.steer_kp = float(kwargs.get("steer_kp", OPT3_STEER_KP))
        self.pp_lookahead_gain = float(kwargs.get("pp_lookahead_gain", OPT3_PP_LOOKAHEAD_GAIN))
        self.straight_lock_dist_m = float(kwargs.get("straight_lock_dist_m", OPT3_STRAIGHT_LOCK_DIST_M))
        self.straight_lock_speed_mps = float(kwargs.get("straight_lock_speed_mps", OPT3_STRAIGHT_LOCK_SPEED_MPS))

        # Preallocated buffers for C-API output
        self._out_steer = ctypes.c_float(0.0)
        self._out_speed = ctypes.c_float(0.0)
        self._out_lap_time = ctypes.c_float(0.0)
        self._out_surviving = ctypes.c_int(0)
        self._out_candidates = ctypes.c_int(0)

        self._best_xs = (ctypes.c_float * self.probe_samples)()
        self._best_ys = (ctypes.c_float * self.probe_samples)()

        self._max_cands = 512
        self._cand_xs = (ctypes.c_float * (self._max_cands * self.probe_samples))()
        self._cand_ys = (ctypes.c_float * (self._max_cands * self.probe_samples))()

        # Connect costmap handle & probe function
        cm_handle, probe_fn_ptr = self._resolve_costmap()

        # Pack blend weights into ctypes array
        blends_arr = (ctypes.c_float * len(self.blend_weights))(*self.blend_weights)

        # Slew rate limit in accordance with physical servo limit (10.5 rad/s)
        if "slew_deg_max" in kwargs:
            self.slew_deg_max = float(kwargs["slew_deg_max"])
            self.max_servo_rate = math.radians(self.slew_deg_max) / max(1e-4, self.dt_loop)
            self.max_servo_rate = min(float(MAX_SERVO_RATE_RAD_S), self.max_servo_rate)
        else:
            self.max_servo_rate = float(MAX_SERVO_RATE_RAD_S)
            self.slew_deg_max = math.degrees(self.max_servo_rate * self.dt_loop)

        if hasattr(_lib, "septic_profiler_create_v3"):
            self._handle = _lib.septic_profiler_create_v3(
                float(WHEELBASE),
                float(VEHICLE_MASS),
                float(UNDERSTEER_K_US),
                float(TIRE_MU),
                float(GRAVITY_G),
                float(MAX_STEERING_RAD),
                float(self.max_servo_rate),
                float(OPT3_ACCEL_MAX),
                float(OPT3_DECEL_MAX),
                float(OPT3_DS),
                float(OPT3_TOP_SPEED_MPS),
                float(V_MIN_CRUISE),
                float(V_MAX_CRUISE),
                float(self.dt_loop),
                float(self.v_entry_max),
                float(self.v_cruise_min),
                float(self.mu_friction),
                int(self.probe_samples),
                int(self.num_lat_offsets),
                float(self.lat_offset_max),
                int(self.num_head_offsets),
                float(self.head_offset_max),
                int(self.num_blend_weights),
                blends_arr,
                int(self.max_extra_targets),
                float(self.w_time),
                float(self.w_clearance),
                float(self.w_curvature_smooth),
                float(self.w_chord_align),
                float(self.w_steer_effort),
                float(self.steer_kp),
                float(self.pp_lookahead_gain),
                cm_handle,
                probe_fn_ptr,
            )
        else:
            self._handle = _lib.septic_profiler_create_v2(
                float(WHEELBASE),
                float(VEHICLE_MASS),
                float(UNDERSTEER_K_US),
                float(TIRE_MU),
                float(GRAVITY_G),
                float(MAX_STEERING_RAD),
                float(self.max_servo_rate),
                float(OPT3_ACCEL_MAX),
                float(OPT3_DECEL_MAX),
                float(OPT3_DS),
                float(OPT3_TOP_SPEED_MPS),
                float(V_MIN_CRUISE),
                float(V_MAX_CRUISE),
                float(self.dt_loop),
                float(self.v_entry_max),
                float(self.v_cruise_min),
                float(self.mu_friction),
                int(self.probe_samples),
                int(self.num_lat_offsets),
                float(self.lat_offset_max),
                int(self.num_head_offsets),
                float(self.head_offset_max),
                int(self.num_blend_weights),
                blends_arr,
                int(self.max_extra_targets),
                float(self.w_time),
                float(self.w_clearance),
                float(self.w_curvature_smooth),
                float(self.w_chord_align),
                float(self.w_steer_effort),
                cm_handle,
                probe_fn_ptr,
            )

        if not self._handle:
            raise RuntimeError("Failed to create C++ SepticTractionProfiler instance")

        if hasattr(_lib, "septic_profiler_set_initial_slew_scale"):
            _lib.septic_profiler_set_initial_slew_scale(
                self._handle,
                float(getattr(self, "initial_slew_scale", OPT3_INITIAL_SLEW_SCALE))
            )

        if hasattr(_lib, "septic_profiler_set_straight_lock"):
            _lib.septic_profiler_set_straight_lock(
                self._handle,
                float(self.straight_lock_dist_m),
                float(self.straight_lock_speed_mps)
            )

    def _resolve_costmap(self):
        """Extracts native costmap pointer and direct C-ABI probe function pointer."""
        if self.costmap is None:
            return None, None

        if hasattr(self.costmap, "_handle") and self.costmap._handle:
            try:
                import cpp_smart_hybrid_costmap
                lib_cm = cpp_smart_hybrid_costmap._lib
                probe_addr = ctypes.cast(lib_cm.costmap_probe_path, ctypes.c_void_p).value
                return ctypes.c_void_p(self.costmap._handle), ctypes.c_void_p(probe_addr)
            except Exception:
                pass

        if hasattr(self.costmap, "probe_path"):
            def _py_probe_cb(h, xs_ptr, ys_ptr, count):
                xs_arr = np.ctypeslib.as_array(xs_ptr, shape=(count,))
                ys_arr = np.ctypeslib.as_array(ys_ptr, shape=(count,))
                return 1 if self.costmap.probe_path(xs_arr, ys_arr) else 0

            self._cb_ref = PROBE_CALLBACK_TYPE(_py_probe_cb)
            fn_addr = ctypes.cast(self._cb_ref, ctypes.c_void_p).value
            return None, ctypes.c_void_p(fn_addr)

        return None, None

    def __del__(self):
        if hasattr(self, "_handle") and self._handle:
            _lib.septic_profiler_destroy(self._handle)
            self._handle = None

    def plan(
        self,
        xg: float,
        yg: float,
        theta_g: float,
        aperture_width: float,
        v_current: float,
        steer_current: float,
        yaw_rate_current: float,
        extra_targets: Optional[List[Tuple[float, float, float]]] = None,
        steer_rate_current: float = 0.0,
    ) -> Dict[str, any]:
        """
        Executes one native C++ G^3 Septic planning and dynamic profiling cycle.
        """
        num_extras = 0
        extras_ptr = None
        if extra_targets:
            num_extras = len(extra_targets)
            flat_targets = []
            for t in extra_targets:
                flat_targets.extend([float(t[0]), float(t[1]), float(t[2])])
            arr = (ctypes.c_float * len(flat_targets))(*flat_targets)
            extras_ptr = ctypes.cast(arr, ctypes.POINTER(ctypes.c_float))

        status_code = _lib.septic_profiler_plan(
            self._handle,
            float(xg), float(yg), float(theta_g), float(aperture_width),
            float(v_current), float(steer_current), float(yaw_rate_current), float(steer_rate_current),
            extras_ptr, num_extras,
            ctypes.byref(self._out_steer),
            ctypes.byref(self._out_speed),
            ctypes.byref(self._out_lap_time),
            ctypes.byref(self._out_surviving),
            ctypes.byref(self._out_candidates),
        )

        surviving_count = self._out_surviving.value
        candidate_count = self._out_candidates.value

        if status_code == 0 or surviving_count == 0:
            return {
                "cmd_steer_rad": float(self._out_steer.value),
                "cmd_speed_mps": float(self._out_speed.value),
                "surviving_count": 0,
                "candidate_count": candidate_count,
                "status": "FALLBACK_NO_SURVIVORS",
            }

        # Extract best line
        n_best = _lib.septic_profiler_get_best_line(
            self._handle,
            self._best_xs,
            self._best_ys,
            self.probe_samples
        )
        best_line = [[float(self._best_xs[i]), float(self._best_ys[i])] for i in range(n_best)]

        # Extract candidate lines
        total_cands = _lib.septic_profiler_get_candidate_lines(
            self._handle,
            self._cand_xs,
            self._cand_ys,
            self._max_cands,
            self.probe_samples
        )
        candidate_lines = []
        for c in range(total_cands):
            line = []
            base_idx = c * self.probe_samples
            for p in range(self.probe_samples):
                line.append([float(self._cand_xs[base_idx + p]), float(self._cand_ys[base_idx + p])])
            candidate_lines.append(line)

        return {
            "cmd_steer_rad": float(self._out_steer.value),
            "cmd_speed_mps": float(self._out_speed.value),
            "best_lap_time": float(self._out_lap_time.value),
            "surviving_count": surviving_count,
            "candidate_count": candidate_count,
            "status": "OPTIMAL_SEPTIC_TRACTION",
            "best_line": best_line,
            "candidate_lines": candidate_lines,
        }
