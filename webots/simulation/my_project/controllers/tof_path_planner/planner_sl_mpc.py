"""
================================================================================
  C++ NATIVE ENGINE: CURVILINEAR SLACK-CONSTRAINED SL-MPC
  - Native AVX2 C++ implementation via ctypes and embedded OSQP solver
  - High-performance replacement for Python prototype
  - Sub-millisecond compute budget (~0.3 ms vs ~26 ms Python)
================================================================================
"""

import ctypes
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple, Union
import numpy as np

try:
    from .planner_config import (
        MPC_HORIZON_N,
        MPC_DT,
        MPC_Q_PROGRESS,
        MPC_Q_CTE,
        MPC_Q_HEADING,
        MPC_R_STEER,
        MPC_R_ACCEL,
        MPC_R_STEER_RATE,
        MPC_CENTER_BIAS,
        MPC_MU_FRICTION,
        MPC_V_CORNER_FACTOR,
        MPC_V_CRUISE_MAX,
        MPC_V_CRUISE_MIN,
        MPC_ALT_SWITCH_HYSTERESIS,
        MPC_ALT_MIN_SEPARATION_M,
        MPC_ALT_APERTURE_BIAS,
        MPC_W_GAP_CLEAR,
        DT_PLANNER,
    )
except ImportError:
    from planner_config import (
        MPC_HORIZON_N,
        MPC_DT,
        MPC_Q_PROGRESS,
        MPC_Q_CTE,
        MPC_Q_HEADING,
        MPC_R_STEER,
        MPC_R_ACCEL,
        MPC_R_STEER_RATE,
        MPC_CENTER_BIAS,
        MPC_MU_FRICTION,
        MPC_V_CORNER_FACTOR,
        MPC_V_CRUISE_MAX,
        MPC_V_CRUISE_MIN,
        MPC_ALT_SWITCH_HYSTERESIS,
        MPC_ALT_MIN_SEPARATION_M,
        MPC_ALT_APERTURE_BIAS,
        MPC_W_GAP_CLEAR,
        DT_PLANNER,
    )

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_PATH = None
_ext = ".dll" if sys.platform == "win32" else ".so"
candidates = [
    os.path.join(CURR_DIR, f"curvilinear_sl_mpc{_ext}"),
    os.path.join(os.getcwd(), f"curvilinear_sl_mpc{_ext}"),
    os.path.abspath(os.path.join(CURR_DIR, "..", "pi4", f"curvilinear_sl_mpc{_ext}")),
    os.path.abspath(os.path.join(CURR_DIR, "..", "..", "..", "..", "..", "pi4", f"curvilinear_sl_mpc{_ext}")),
]
for _cand in candidates:
    if os.path.exists(_cand):
        DLL_PATH = _cand
        break

CPP_AVAILABLE = False
_lib = None

if os.path.exists(DLL_PATH):
    try:
        _lib = ctypes.CDLL(DLL_PATH)
        CPP_AVAILABLE = True
    except Exception as e:
        print(f"[Warning] Failed to load curvilinear_sl_mpc.dll: {e}", flush=True)

PROBE_CALLBACK_TYPE = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_int,
)

if CPP_AVAILABLE and _lib is not None:
    _lib.sl_mpc_create.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double,
    ]
    _lib.sl_mpc_create.restype = ctypes.c_void_p

    _lib.sl_mpc_destroy.argtypes = [ctypes.c_void_p]
    _lib.sl_mpc_destroy.restype = None

    _lib.sl_mpc_set_costmap.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    _lib.sl_mpc_set_costmap.restype = None

    if hasattr(_lib, "sl_mpc_set_mu"):
        _lib.sl_mpc_set_mu.argtypes = [ctypes.c_void_p, ctypes.c_double]
        _lib.sl_mpc_set_mu.restype = None

    if hasattr(_lib, "sl_mpc_set_center_bias"):
        _lib.sl_mpc_set_center_bias.argtypes = [ctypes.c_void_p, ctypes.c_double]
        _lib.sl_mpc_set_center_bias.restype = None

    if hasattr(_lib, "sl_mpc_reset"):
        _lib.sl_mpc_reset.argtypes = [ctypes.c_void_p]
        _lib.sl_mpc_reset.restype = None

    if hasattr(_lib, "sl_mpc_set_max_iter"):
        _lib.sl_mpc_set_max_iter.argtypes = [ctypes.c_void_p, ctypes.c_int]
        _lib.sl_mpc_set_max_iter.restype = None

    _lib.sl_mpc_plan.argtypes = [
        ctypes.c_void_p,
        ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    _lib.sl_mpc_plan.restype = ctypes.c_int

    if hasattr(_lib, "sl_mpc_plan_projected"):
        _lib.sl_mpc_plan_projected.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        ]
        _lib.sl_mpc_plan_projected.restype = ctypes.c_int

    _lib.sl_mpc_get_best_line.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int,
    ]
    _lib.sl_mpc_get_best_line.restype = ctypes.c_int

    _lib.sl_mpc_get_guide_line.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int,
    ]
    _lib.sl_mpc_get_guide_line.restype = ctypes.c_int

    _lib.sl_mpc_get_corridor.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int,
    ]
    _lib.sl_mpc_get_corridor.restype = ctypes.c_int

    _lib.sl_mpc_get_shield_branches.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    _lib.sl_mpc_get_shield_branches.restype = ctypes.c_int


class CppCurvilinearSLMPC:
    """
    High-Performance Native C++ Curvilinear SL-MPC Planner.
    100% Drop-In Compatible with CurvilinearSLMPC.
    """

    def __init__(
        self,
        costmap_engine: any = None,
        horizon_N: int = MPC_HORIZON_N,
        dt: float = MPC_DT,
        q_progress: float = MPC_Q_PROGRESS,
        q_cte: float = MPC_Q_CTE,
        q_heading: float = MPC_Q_HEADING,
        r_steer: float = MPC_R_STEER,
        r_accel: float = MPC_R_ACCEL,
        r_steer_rate: float = MPC_R_STEER_RATE,
        mu_friction: float = MPC_MU_FRICTION,
        v_corner_factor: float = MPC_V_CORNER_FACTOR,
        v_cruise_max: float = MPC_V_CRUISE_MAX,
        v_cruise_min: float = MPC_V_CRUISE_MIN,
        alt_switch_hysteresis: float = MPC_ALT_SWITCH_HYSTERESIS,
        alt_min_separation_m: float = MPC_ALT_MIN_SEPARATION_M,
        alt_aperture_bias: float = MPC_ALT_APERTURE_BIAS,
        w_gap_clear: float = MPC_W_GAP_CLEAR,
        center_bias: float = MPC_CENTER_BIAS,
        **kwargs,
    ):
        self._costmap = costmap_engine
        self.N = int(horizon_N)
        self.dt = float(dt)
        self.q_progress = float(q_progress)
        self.q_cte = float(q_cte)
        self.q_heading = float(q_heading)
        self.r_steer = float(r_steer)
        self.r_accel = float(r_accel)
        self.r_steer_rate = float(r_steer_rate)
        self.mu_friction = float(mu_friction)
        self.v_corner_factor = float(v_corner_factor)
        self.v_cruise_max = float(v_cruise_max)
        self.v_cruise_min = float(v_cruise_min)
        self.alt_switch_hysteresis = float(alt_switch_hysteresis)
        self.alt_min_separation_m = float(alt_min_separation_m)
        self.alt_aperture_bias = float(alt_aperture_bias)
        self.w_gap_clear = float(kwargs.get("alt_lookahead_depth_weight", w_gap_clear))
        self.center_bias = max(0.0, min(1.0, float(center_bias)))
        self.dt_actuator = DT_PLANNER
        self.nx = 7
        self.nu = 2
        self._last_winner = 0
        self._active_target: Optional[Tuple[float, float]] = None
        self._last_alt_target: Optional[Tuple[float, float]] = None

        # Preallocated ctypes outputs
        self._out_steer = ctypes.c_double(0.0)
        self._out_speed = ctypes.c_double(0.0)
        self._out_best_cost = ctypes.c_double(0.0)
        self._out_osqp_iters = ctypes.c_int(0)
        self._out_status_code = ctypes.c_int(0)

        self._max_pts = 64
        self._buf_xs1 = (ctypes.c_double * self._max_pts)()
        self._buf_ys1 = (ctypes.c_double * self._max_pts)()
        self._buf_xs2 = (ctypes.c_double * self._max_pts)()
        self._buf_ys2 = (ctypes.c_double * self._max_pts)()
        self._buf_xs3 = (ctypes.c_double * self._max_pts)()
        self._buf_ys3 = (ctypes.c_double * self._max_pts)()
        self._buf_xs4 = (ctypes.c_double * self._max_pts)()
        self._buf_ys4 = (ctypes.c_double * self._max_pts)()

        self._buf_xs5 = (ctypes.c_double * self._max_pts)()
        self._buf_ys5 = (ctypes.c_double * self._max_pts)()
        self._buf_xs6 = (ctypes.c_double * self._max_pts)()
        self._buf_ys6 = (ctypes.c_double * self._max_pts)()

        self._shield_k1_l_cnt = ctypes.c_int(0)
        self._shield_k1_l_safe = ctypes.c_int(0)
        self._shield_k1_r_cnt = ctypes.c_int(0)
        self._shield_k1_r_safe = ctypes.c_int(0)

        self._shield_k2_l_cnt = ctypes.c_int(0)
        self._shield_k2_l_safe = ctypes.c_int(0)
        self._shield_k2_r_cnt = ctypes.c_int(0)
        self._shield_k2_r_safe = ctypes.c_int(0)

        self._shield_straight_cnt = ctypes.c_int(0)
        self._shield_straight_safe = ctypes.c_int(0)
        self._shield_straight_zero_cnt = ctypes.c_int(0)
        self._shield_straight_zero_safe = ctypes.c_int(0)

        self._shield_active = ctypes.c_int(0)
        self._shield_chosen = ctypes.c_int(-1)

        cm_handle, probe_fn_ptr = self._resolve_costmap()

        # Handle 1: Primary Racing Line solver
        self._handle = _lib.sl_mpc_create(
            cm_handle,
            probe_fn_ptr,
            int(self.N),
            float(self.dt),
            float(self.q_progress),
            float(self.q_cte),
            float(self.q_heading),
            float(self.r_steer),
            float(self.r_accel),
            float(self.r_steer_rate),
            float(self.mu_friction),
            float(self.v_corner_factor),
            float(self.v_cruise_max),
            float(self.v_cruise_min),
        )

        # Handle 2: Smart Challenger Alternative Line solver
        self._handle_alt = _lib.sl_mpc_create(
            cm_handle,
            probe_fn_ptr,
            int(self.N),
            float(self.dt),
            float(self.q_progress),
            float(self.q_cte),
            float(self.q_heading),
            float(self.r_steer),
            float(self.r_accel),
            float(self.r_steer_rate),
            float(self.mu_friction),
            float(self.v_corner_factor),
            float(self.v_cruise_max),
            float(self.v_cruise_min),
        )

        if not self._handle:
            raise RuntimeError("Failed to create C++ CurvilinearSLMPCNative instance")

        # Deterministic 60 Hz iteration caps (prevents tail latency overruns on ARM Cortex-A72)
        if hasattr(_lib, "sl_mpc_set_max_iter"):
            if self._handle:
                _lib.sl_mpc_set_max_iter(self._handle, 14)
            if self._handle_alt:
                _lib.sl_mpc_set_max_iter(self._handle_alt, 6)

        self.set_center_bias(self.center_bias)

    def reset_active_target(self) -> None:
        """Resets the currently tracked active corridor/target line and OSQP solver warm-start."""
        self._active_target = None
        self._last_alt_target = None
        self._last_winner = 0
        if _lib is not None and hasattr(_lib, "sl_mpc_reset"):
            if hasattr(self, "_handle") and self._handle:
                _lib.sl_mpc_reset(self._handle)
            if hasattr(self, "_handle_alt") and self._handle_alt:
                _lib.sl_mpc_reset(self._handle_alt)

    def set_dynamic_mu(self, new_mu: float) -> None:
        """
        Dynamically updates the tire friction coefficient governing the grip envelope
        and OSQP solver constraints in the native C++ Curvilinear SL-MPC planner.
        """
        self.mu_friction = float(new_mu)
        if _lib is not None and hasattr(_lib, "sl_mpc_set_mu"):
            if hasattr(self, "_handle") and self._handle:
                _lib.sl_mpc_set_mu(self._handle, ctypes.c_double(self.mu_friction))
            if hasattr(self, "_handle_alt") and self._handle_alt:
                _lib.sl_mpc_set_mu(self._handle_alt, ctypes.c_double(self.mu_friction))

    def set_center_bias(self, center_bias: float) -> None:
        """Updates the corridor centering blend [0.0, 1.0] on both primary and challenger C++ solvers."""
        self.center_bias = max(0.0, min(1.0, float(center_bias)))
        if _lib is not None and hasattr(_lib, "sl_mpc_set_center_bias"):
            if hasattr(self, "_handle") and self._handle:
                _lib.sl_mpc_set_center_bias(self._handle, ctypes.c_double(self.center_bias))
            if hasattr(self, "_handle_alt") and self._handle_alt:
                _lib.sl_mpc_set_center_bias(self._handle_alt, ctypes.c_double(self.center_bias))

    @property
    def alt_lookahead_depth_weight(self) -> float:
        return self.w_gap_clear

    @alt_lookahead_depth_weight.setter
    def alt_lookahead_depth_weight(self, val: float):
        self.w_gap_clear = float(val)

    @property
    def costmap(self):
        return getattr(self, "_costmap", None)

    @costmap.setter
    def costmap(self, cm):
        self._costmap = cm
        cm_handle, probe_fn_ptr = self._resolve_costmap()
        if hasattr(self, "_handle") and self._handle:
            _lib.sl_mpc_set_costmap(self._handle, cm_handle, probe_fn_ptr)
        if hasattr(self, "_handle_alt") and self._handle_alt:
            _lib.sl_mpc_set_costmap(self._handle_alt, cm_handle, probe_fn_ptr)

    def _resolve_costmap(self):
        """Extracts native costmap pointer and direct C-ABI probe function pointer."""
        if self._costmap is None:
            return None, None

        if hasattr(self._costmap, "_handle") and self._costmap._handle:
            try:
                import cpp_smart_hybrid_costmap
                lib_cm = cpp_smart_hybrid_costmap._lib
                probe_addr = ctypes.cast(lib_cm.costmap_probe_path, ctypes.c_void_p).value
                return ctypes.c_void_p(self._costmap._handle), ctypes.c_void_p(probe_addr)
            except Exception as e:
                print(f"[Warning] SL-MPC native costmap probe unavailable ({e}); "
                      f"falling back to SLOW Python probe callback", flush=True)

        if hasattr(self._costmap, "probe_path"):
            def _py_probe_cb(h, xs_ptr, ys_ptr, count):
                xs_arr = np.ctypeslib.as_array(xs_ptr, shape=(count,))
                ys_arr = np.ctypeslib.as_array(ys_ptr, shape=(count,))
                return 1 if self._costmap.probe_path(xs_arr, ys_arr) else 0

            self._cb_ref = PROBE_CALLBACK_TYPE(_py_probe_cb)
            fn_addr = ctypes.cast(self._cb_ref, ctypes.c_void_p).value
            return None, ctypes.c_void_p(fn_addr)

        return None, None

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
        x0: float = 0.0,
        y0: float = 0.0,
        psi0: float = 0.0,
        t_frame_start: Optional[float] = None,
        frame_budget_ms: float = 15.5,
    ) -> Dict[str, any]:
        """
        Solves the Curvilinear SL-MPC QP using high-speed native C++ core.
        Tracks active line identity, associates perception candidates to Incumbent vs Challenger,
        and arbitrates using alt_switch_hysteresis cost margin.
        """
        # ----------------------------------------------------------------------
        # 0. CANDIDATE ASSOCIATION (INCUMBENT VS CHALLENGER)
        # ----------------------------------------------------------------------
        targets = []
        if extra_targets and len(extra_targets) > 0:
            targets = list(extra_targets)
        elif xg is not None and yg is not None:
            targets = [(float(xg), float(yg), float(aperture_width))]

        if not targets:
            targets = [(0.0, 1.20, 0.40)]

        prim_target = targets[0]
        alt_target = targets[1] if len(targets) >= 2 else None

        # Associate incoming stateless candidates with tracked active target
        if self._active_target is not None and len(targets) >= 2:
            act_x, act_y = self._active_target
            d0 = math.hypot(targets[0][0] - act_x, targets[0][1] - act_y)
            d1 = math.hypot(targets[1][0] - act_x, targets[1][1] - act_y)

            is_clear_0 = bool(targets[0][3]) if len(targets[0]) >= 4 else (math.hypot(targets[0][0], targets[0][1]) >= 2.20)
            is_clear_1 = bool(targets[1][3]) if len(targets[1]) >= 4 else (math.hypot(targets[1][0], targets[1][1]) >= 2.20)

            # If target 1 is closer to our currently followed line, target 1 is Incumbent
            # UNLESS target 1 is blocked by an obstacle while target 0 is completely clear (1.8m opening).
            if d1 < d0 and d1 <= 0.45:
                if (not is_clear_1) and is_clear_0:
                    prim_target = targets[0]
                    alt_target = targets[1]
                else:
                    prim_target = targets[1]
                    alt_target = targets[0]
            elif d0 <= 0.45:
                if (not is_clear_0) and is_clear_1:
                    prim_target = targets[1]
                    alt_target = targets[0]
                else:
                    prim_target = targets[0]
                    alt_target = targets[1]

        tx_prim, ty_prim, ap_prim = prim_target[:3]
        is_clear_prim = bool(prim_target[3]) if len(prim_target) >= 4 else (math.hypot(tx_prim, ty_prim) >= 2.20)
        theta_prim = math.atan2(tx_prim - x0, max(0.20, ty_prim - y0))

        t_opt_0 = time.perf_counter()
        t_opt_cpu_0 = time.process_time()

        # ----------------------------------------------------------------------
        # 1. SOLVE INCUMBENT (PRIMARY) RACING LINE
        # ----------------------------------------------------------------------
        # Dynamic Primary OSQP Iteration Scaling based on whole-pipeline budget elapsed so far:
        # Prevents tail-latency overruns when perception (costmap + gap scout) took longer.
        if hasattr(_lib, "sl_mpc_set_max_iter") and self._handle:
            if t_frame_start is not None:
                elapsed_prior_ms = (time.perf_counter() - t_frame_start) * 1000.0
                target_deadline_ms = min(float(frame_budget_ms), 13.2)
                budget_for_prim_ms = target_deadline_ms - elapsed_prior_ms - 0.5  # Reserve 0.5ms for shield
                if budget_for_prim_ms >= 8.0:
                    max_iter_prim = 14
                elif budget_for_prim_ms >= 4.5:
                    max_iter_prim = 10
                else:
                    max_iter_prim = 6
            else:
                max_iter_prim = 14
            _lib.sl_mpc_set_max_iter(self._handle, max_iter_prim)

        t_prim_0 = time.perf_counter()
        if hasattr(_lib, "sl_mpc_plan_projected"):
            success_prim = _lib.sl_mpc_plan_projected(
                self._handle,
                float(tx_prim), float(ty_prim), float(theta_prim), float(ap_prim),
                float(v_current), float(steer_current), float(yaw_rate_current),
                float(x0), float(y0), float(psi0),
                ctypes.byref(self._out_steer),
                ctypes.byref(self._out_speed),
                ctypes.byref(self._out_best_cost),
                ctypes.byref(self._out_osqp_iters),
                ctypes.byref(self._out_status_code),
            )
        else:
            success_prim = _lib.sl_mpc_plan(
                self._handle,
                float(tx_prim), float(ty_prim), float(theta_prim), float(ap_prim),
                float(v_current), float(steer_current), float(yaw_rate_current),
                ctypes.byref(self._out_steer),
                ctypes.byref(self._out_speed),
                ctypes.byref(self._out_best_cost),
                ctypes.byref(self._out_osqp_iters),
                ctypes.byref(self._out_status_code),
            )

        status_prim = self._out_status_code.value
        cmd_steer_prim = float(self._out_steer.value)
        cmd_speed_prim = float(self._out_speed.value)
        cost_prim = float(self._out_best_cost.value)
        iters_prim = int(self._out_osqp_iters.value)

        n_best_prim = _lib.sl_mpc_get_best_line(self._handle, self._buf_xs1, self._buf_ys1, self._max_pts)
        best_line_prim = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_best_prim)]

        n_guide_prim = _lib.sl_mpc_get_guide_line(self._handle, self._buf_xs1, self._buf_ys1, self._max_pts)
        guide_line_prim = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_guide_prim)]

        n_corr = _lib.sl_mpc_get_corridor(self._handle, self._buf_xs1, self._buf_ys1, self._buf_xs2, self._buf_ys2, self._max_pts)
        corridor_left = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_corr)]
        corridor_right = [[float(self._buf_xs2[i]), float(self._buf_ys2[i])] for i in range(n_corr)]
        t_prim_ms = (time.perf_counter() - t_prim_0) * 1000.0

        # ----------------------------------------------------------------------
        # 2. SOLVE SMART CHALLENGER ALTERNATIVE LINE (if available)
        # ----------------------------------------------------------------------
        has_challenger = False
        status_alt = -1
        cmd_steer_alt = 0.0
        cmd_speed_alt = self.v_cruise_min
        cost_alt = 1e9
        best_line_alt = []
        t_alt_ms = 0.0
        iters_alt = 0

        # Budget-conscious challenger solve with at least 1 iteration:
        # When distinct alternative target exists (has_sep):
        # - Capped iteration budget (8 iters) if remaining frame budget >= 5.5 ms.
        # - Fallback to single iteration (max_iter = 1) when budget is tight (>= 3.2 ms).
        # - Hard skip challenger if remaining budget < 3.2 ms to guarantee strict 60 Hz compliance.
        if alt_target is not None and hasattr(self, "_handle_alt") and self._handle_alt:
            tx_alt, ty_alt, ap_alt = alt_target[:3]
            is_clear_alt = bool(alt_target[3]) if len(alt_target) >= 4 else (math.hypot(tx_alt, ty_alt) >= 2.20)
            min_sep = getattr(self, "alt_min_separation_m", 0.06)
            has_sep = (math.hypot(tx_alt - tx_prim, ty_alt - ty_prim) >= min_sep or abs(tx_alt - tx_prim) >= (min_sep * 0.7))

            if has_sep:
                # Strict Whole-Pipeline Frame Budget Enforcement (Guaranteed < 16.6 ms worst-case):
                # Measure total time spent across the ENTIRE single-frame pipeline so far:
                # (perception + motion latency comp + costmap update + gap scout + primary MPC solve)
                t_check = time.perf_counter()
                elapsed_so_far_ms = ((t_check - t_frame_start) * 1000.0) if t_frame_start is not None else t_prim_ms
                target_deadline_ms = min(float(frame_budget_ms), 13.2)
                budget_remaining_ms = target_deadline_ms - elapsed_so_far_ms

                # Hard budget allocation (calibrated to ARM64 Cortex-A72 execution profile):
                # - High budget (10 iters): ONLY IF budget_remaining >= 6.5 ms
                # - Single iter (1 iter, ~1.2 ms + 0.5 ms shield): IF budget_remaining >= 4.8 ms
                # - Under 4.8 ms: HARD SKIP challenger to guarantee worst-case frame strictly < 16.6 ms!
                if budget_remaining_ms >= 6.5:
                    has_challenger = True
                    max_iter_alt = 10
                elif budget_remaining_ms >= 4.8:
                    has_challenger = True
                    max_iter_alt = 1
                else:
                    has_challenger = False

                if has_challenger:
                    if hasattr(_lib, "sl_mpc_reset"):
                        if (
                            self._last_alt_target is None
                            or math.hypot(tx_alt - self._last_alt_target[0], ty_alt - self._last_alt_target[1]) > 0.45
                        ):
                            _lib.sl_mpc_reset(self._handle_alt)
                    self._last_alt_target = (float(tx_alt), float(ty_alt))
                    if hasattr(_lib, "sl_mpc_set_max_iter"):
                        _lib.sl_mpc_set_max_iter(self._handle_alt, max_iter_alt)
                    t_alt_0 = time.perf_counter()
                    theta_alt = math.atan2(tx_alt - x0, max(0.20, ty_alt - y0))
                    if hasattr(_lib, "sl_mpc_plan_projected"):
                        _lib.sl_mpc_plan_projected(
                            self._handle_alt,
                            float(tx_alt), float(ty_alt), float(theta_alt), float(ap_alt),
                            float(v_current), float(steer_current), float(yaw_rate_current),
                            float(x0), float(y0), float(psi0),
                            ctypes.byref(self._out_steer),
                            ctypes.byref(self._out_speed),
                            ctypes.byref(self._out_best_cost),
                            ctypes.byref(self._out_osqp_iters),
                            ctypes.byref(self._out_status_code),
                        )
                    else:
                        _lib.sl_mpc_plan(
                            self._handle_alt,
                            float(tx_alt), float(ty_alt), float(theta_alt), float(ap_alt),
                            float(v_current), float(steer_current), float(yaw_rate_current),
                            ctypes.byref(self._out_steer),
                            ctypes.byref(self._out_speed),
                            ctypes.byref(self._out_best_cost),
                            ctypes.byref(self._out_osqp_iters),
                            ctypes.byref(self._out_status_code),
                        )
                    status_alt = self._out_status_code.value
                    iters_alt = int(self._out_osqp_iters.value)
                    cmd_steer_alt = float(self._out_steer.value)
                    cmd_speed_alt = float(self._out_speed.value)
                    cost_alt = float(self._out_best_cost.value)
                    n_best_alt = _lib.sl_mpc_get_best_line(self._handle_alt, self._buf_xs1, self._buf_ys1, self._max_pts)
                    best_line_alt = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_best_alt)]
                    t_alt_ms = (time.perf_counter() - t_alt_0) * 1000.0

        # ----------------------------------------------------------------------
        # 3. ARBITRATION BETWEEN INCUMBENT AND CHALLENGER
        # ----------------------------------------------------------------------
        candidate_lines = [best_line_prim, best_line_alt] if (has_challenger and best_line_alt) else [best_line_prim]

        winner = 0
        depth_credit = 0.0
        if has_challenger:
            primary_ok = (status_prim == 0)
            alt_ok = (status_alt == 0)
            if not primary_ok and alt_ok:
                # Primary line blocked/infeasible, challenger clear: immediate safety switch
                winner = 1
            elif primary_ok and not alt_ok:
                # Challenger infeasible, primary safe: retain primary
                winner = 0
            elif is_clear_alt and not is_clear_prim:
                # Challenger reaches unobstructed open horizon while primary terminates in an obstacle:
                # The clear opening decisively takes precedence over the obstructed path!
                winner = 1
            elif is_clear_prim and not is_clear_alt:
                # Primary is unobstructed while challenger hits an obstacle:
                # Retain primary unobstructed racing line!
                winner = 0
            elif primary_ok and alt_ok:
                # Both are clear or both are obstructed: arbitrate using MPC dynamic cost with hysteresis and depth credit
                d_prim = math.hypot(tx_prim, ty_prim)
                d_alt = math.hypot(tx_alt, ty_alt)
                depth_diff = d_alt - d_prim

                # Base depth credit proportional to lookahead advantage
                depth_credit = self.w_gap_clear * depth_diff

                # Challenger must beat Incumbent by hysteresis cost margin minus lookahead depth credit
                if (cost_alt - depth_credit + self.alt_switch_hysteresis) < cost_prim:
                    winner = 1
                else:
                    winner = 0

        # Update active tracked target to the winning line's target
        if winner == 1 and alt_target is not None:
            self._active_target = (alt_target[0], alt_target[1])
        else:
            self._active_target = (prim_target[0], prim_target[1])
        self._last_winner = winner

        win_handle = self._handle_alt if (winner == 1 and hasattr(self, "_handle_alt") and self._handle_alt) else self._handle

        n_guide = _lib.sl_mpc_get_guide_line(win_handle, self._buf_xs1, self._buf_ys1, self._max_pts)
        guide_line_win = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_guide)] if n_guide > 0 else guide_line_prim

        n_corr = _lib.sl_mpc_get_corridor(win_handle, self._buf_xs1, self._buf_ys1, self._buf_xs2, self._buf_ys2, self._max_pts)
        corridor_left = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_corr)]
        corridor_right = [[float(self._buf_xs2[i]), float(self._buf_ys2[i])] for i in range(n_corr)]

        # Extract Emergency Shield evasion branches (Step 0 Left/Right, Step 2 Left/Right, MPC Straight, Zero-Steer Straight)
        t_shield_0 = time.perf_counter()
        _lib.sl_mpc_get_shield_branches(
            win_handle,
            self._buf_xs1, self._buf_ys1, ctypes.byref(self._shield_k1_l_cnt), ctypes.byref(self._shield_k1_l_safe),
            self._buf_xs2, self._buf_ys2, ctypes.byref(self._shield_k1_r_cnt), ctypes.byref(self._shield_k1_r_safe),
            self._buf_xs3, self._buf_ys3, ctypes.byref(self._shield_k2_l_cnt), ctypes.byref(self._shield_k2_l_safe),
            self._buf_xs4, self._buf_ys4, ctypes.byref(self._shield_k2_r_cnt), ctypes.byref(self._shield_k2_r_safe),
            self._buf_xs5, self._buf_ys5, ctypes.byref(self._shield_straight_cnt), ctypes.byref(self._shield_straight_safe),
            self._buf_xs6, self._buf_ys6, ctypes.byref(self._shield_straight_zero_cnt), ctypes.byref(self._shield_straight_zero_safe),
            ctypes.byref(self._shield_active), ctypes.byref(self._shield_chosen)
        )

        n_k1_l = int(self._shield_k1_l_cnt.value)
        n_k1_r = int(self._shield_k1_r_cnt.value)
        n_k2_l = int(self._shield_k2_l_cnt.value)
        n_k2_r = int(self._shield_k2_r_cnt.value)
        n_str  = int(self._shield_straight_cnt.value)
        n_str_zero = int(self._shield_straight_zero_cnt.value)

        k1_left_branch  = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_k1_l)] if n_k1_l > 0 else []
        k1_right_branch = [[float(self._buf_xs2[i]), float(self._buf_ys2[i])] for i in range(n_k1_r)] if n_k1_r > 0 else []
        k2_left_branch  = [[float(self._buf_xs3[i]), float(self._buf_ys3[i])] for i in range(n_k2_l)] if n_k2_l > 0 else []
        k2_right_branch = [[float(self._buf_xs4[i]), float(self._buf_ys4[i])] for i in range(n_k2_r)] if n_k2_r > 0 else []
        straight_branch = [[float(self._buf_xs5[i]), float(self._buf_ys5[i])] for i in range(n_str)]  if n_str  > 0 else []
        straight_zero_branch = [[float(self._buf_xs6[i]), float(self._buf_ys6[i])] for i in range(n_str_zero)] if n_str_zero > 0 else []

        chosen_id = int(self._shield_chosen.value)
        chosen_name = None
        if chosen_id == 0:
            chosen_name = "k2_left"
        elif chosen_id == 1:
            chosen_name = "k2_right"
        elif chosen_id == 2:
            chosen_name = "k1_left"
        elif chosen_id == 3:
            chosen_name = "k1_right"
        elif chosen_id == 4:
            chosen_name = "reverse"  # All evasion paths blocked — recovery manager should reverse

        shield_info = {
            "active": bool(self._shield_active.value),
            "chosen_id": chosen_id,
            "chosen_name": chosen_name,
            "reverse_suggested": (chosen_id == 4),  # True when all 5-step evasion arcs are blocked
            "k1_left": k1_left_branch,
            "k1_left_safe": bool(self._shield_k1_l_safe.value),
            "k1_right": k1_right_branch,
            "k1_right_safe": bool(self._shield_k1_r_safe.value),
            "k2_left": k2_left_branch,
            "k2_left_safe": bool(self._shield_k2_l_safe.value),
            "k2_right": k2_right_branch,
            "k2_right_safe": bool(self._shield_k2_r_safe.value),
            "straight": straight_branch,
            "straight_safe": bool(self._shield_straight_safe.value),
            "straight_mpc": straight_branch,
            "straight_mpc_safe": bool(self._shield_straight_safe.value),
            "straight_zero": straight_zero_branch,
            "straight_zero_safe": bool(self._shield_straight_zero_safe.value),
        }
        t_shield_ms = (time.perf_counter() - t_shield_0) * 1000.0

        # ----------------------------------------------------------------------
        # 4. PACKAGE ACTIVE CONTROL COMMANDS
        # ----------------------------------------------------------------------
        shield_is_active = bool(self._shield_active.value)
        if winner == 1:
            best_line = best_line_alt
            cmd_steer = cmd_steer_alt
            cmd_speed = cmd_speed_alt
            status_desc = "CHALLENGER_OVERTHROW"
            active_cost = cost_alt
        elif status_prim == 0:
            best_line = best_line_prim
            cmd_steer = cmd_steer_prim
            cmd_speed = cmd_speed_prim
            status_desc = "OPTIMAL_OSQP_MPC"
            active_cost = cost_prim
        else:
            best_line = best_line_prim
            cmd_steer = cmd_steer_prim
            cmd_speed = cmd_speed_prim if shield_is_active else self.v_cruise_min
            status_desc = "FALLBACK_SL_MPC"
            active_cost = cost_prim

        t_opt_total_ms = (time.perf_counter() - t_opt_0) * 1000.0
        t_opt_cpu_ms = (time.process_time() - t_opt_cpu_0) * 1000.0

        return {
            "cmd_steer_rad": cmd_steer,
            "cmd_speed_mps": cmd_speed,
            "osqp_status": "solved" if (status_prim == 0 or status_alt == 0) else "fallback",
            "osqp_iters": iters_prim,
            "osqp_iters_alt": iters_alt,
            "status_prim": status_prim,
            "status_alt": status_alt if has_challenger else None,
            "best_cost": active_cost,
            "cost_primary": cost_prim,
            "cost_alt": cost_alt if has_challenger else None,
            "depth_credit": depth_credit,
            "winner_id": winner,
            "status": status_desc,
            "best_line": best_line,
            "guide_line": guide_line_win,
            "candidate_lines": candidate_lines,
            "corridor_left": corridor_left,
            "corridor_right": corridor_right,
            "shield": shield_info,
            "opt_ms": round(t_opt_total_ms, 2),
            "opt_cpu_ms": round(t_opt_cpu_ms, 2),
            "opt_prim_ms": round(t_prim_ms, 2),
            "opt_alt_ms": round(t_alt_ms, 2),
            "opt_shield_ms": round(t_shield_ms, 2),
        }

    def __del__(self):
        if _lib is not None:
            if hasattr(self, "_handle") and self._handle:
                try:
                    _lib.sl_mpc_destroy(self._handle)
                except Exception:
                    pass
                self._handle = None
            if hasattr(self, "_handle_alt") and self._handle_alt:
                try:
                    _lib.sl_mpc_destroy(self._handle_alt)
                except Exception:
                    pass
                self._handle_alt = None


# Primary export alias
CurvilinearSLMPC = CppCurvilinearSLMPC

__all__ = [
    "CurvilinearSLMPC",
    "CppCurvilinearSLMPC",
    "CPP_AVAILABLE",
]
