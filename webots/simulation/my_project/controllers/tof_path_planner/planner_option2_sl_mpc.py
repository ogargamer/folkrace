"""
================================================================================
  OPTION 2 (C++ NATIVE ENGINE): CURVILINEAR SLACK-CONSTRAINED SL-MPC
  - Native AVX2 C++ implementation via ctypes and embedded OSQP solver
  - High-performance replacement for planner_option2_sl_mpc.py
  - Sub-millisecond compute budget (~0.3 ms vs ~26 ms Python)
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
        OPT2_HORIZON_N,
        OPT2_DT,
        OPT2_Q_PROGRESS,
        OPT2_Q_CTE,
        OPT2_Q_HEADING,
        OPT2_R_STEER,
        OPT2_R_ACCEL,
        OPT2_R_STEER_RATE,
        OPT2_MU_FRICTION,
        OPT2_V_CORNER_FACTOR,
        OPT2_PP_BLEND_GAIN,
        OPT2_LOOKAHEAD_BLEND_M,
        OPT2_V_CRUISE_MAX,
        OPT2_V_CRUISE_MIN,
        OPT2_STRAIGHT_LOCK_DIST_M,
        OPT2_STRAIGHT_LOCK_SPEED_MPS,
        OPT2_ALT_SWITCH_HYSTERESIS,
        OPT2_ALT_MIN_SEPARATION_M,
        OPT2_ALT_APERTURE_BIAS,
        DT_PLANNER,
    )
except ImportError:
    from planner_config import (
        OPT2_HORIZON_N,
        OPT2_DT,
        OPT2_Q_PROGRESS,
        OPT2_Q_CTE,
        OPT2_Q_HEADING,
        OPT2_R_STEER,
        OPT2_R_ACCEL,
        OPT2_R_STEER_RATE,
        OPT2_MU_FRICTION,
        OPT2_V_CORNER_FACTOR,
        OPT2_PP_BLEND_GAIN,
        OPT2_LOOKAHEAD_BLEND_M,
        OPT2_V_CRUISE_MAX,
        OPT2_V_CRUISE_MIN,
        OPT2_STRAIGHT_LOCK_DIST_M,
        OPT2_STRAIGHT_LOCK_SPEED_MPS,
        OPT2_ALT_SWITCH_HYSTERESIS,
        OPT2_ALT_MIN_SEPARATION_M,
        OPT2_ALT_APERTURE_BIAS,
        DT_PLANNER,
    )

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_PATH = os.path.join(CURR_DIR, "curvilinear_sl_mpc.dll")

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
        ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double,
    ]
    _lib.sl_mpc_create.restype = ctypes.c_void_p

    _lib.sl_mpc_destroy.argtypes = [ctypes.c_void_p]
    _lib.sl_mpc_destroy.restype = None

    _lib.sl_mpc_set_costmap.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    _lib.sl_mpc_set_costmap.restype = None

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
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    _lib.sl_mpc_get_shield_branches.restype = ctypes.c_int


class CppCurvilinearSLMPC:
    """
    High-Performance Native C++ Option 2 Curvilinear SL-MPC Planner.
    100% Drop-In Compatible with CurvilinearSLMPC.
    """

    def __init__(
        self,
        costmap_engine: any = None,
        horizon_N: int = OPT2_HORIZON_N,
        dt: float = OPT2_DT,
        q_progress: float = OPT2_Q_PROGRESS,
        q_cte: float = OPT2_Q_CTE,
        q_heading: float = OPT2_Q_HEADING,
        r_steer: float = OPT2_R_STEER,
        r_accel: float = OPT2_R_ACCEL,
        r_steer_rate: float = OPT2_R_STEER_RATE,
        mu_friction: float = OPT2_MU_FRICTION,
        v_corner_factor: float = OPT2_V_CORNER_FACTOR,
        pure_pursuit_blend_gain: float = OPT2_PP_BLEND_GAIN,
        lookahead_blend_m: float = OPT2_LOOKAHEAD_BLEND_M,
        v_cruise_max: float = OPT2_V_CRUISE_MAX,
        v_cruise_min: float = OPT2_V_CRUISE_MIN,
        straight_lock_dist_m: float = OPT2_STRAIGHT_LOCK_DIST_M,
        straight_lock_speed_mps: float = OPT2_STRAIGHT_LOCK_SPEED_MPS,
        alt_switch_hysteresis: float = OPT2_ALT_SWITCH_HYSTERESIS,
        alt_min_separation_m: float = OPT2_ALT_MIN_SEPARATION_M,
        alt_aperture_bias: float = OPT2_ALT_APERTURE_BIAS,
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
        self.pure_pursuit_blend_gain = float(pure_pursuit_blend_gain)
        self.lookahead_blend_m = float(lookahead_blend_m)
        self.v_cruise_max = float(v_cruise_max)
        self.v_cruise_min = float(v_cruise_min)
        self.straight_lock_dist_m = float(straight_lock_dist_m)
        self.straight_lock_speed_mps = float(straight_lock_speed_mps)
        self.alt_switch_hysteresis = float(alt_switch_hysteresis)
        self.alt_min_separation_m = float(alt_min_separation_m)
        self.alt_aperture_bias = float(alt_aperture_bias)
        self.dt_actuator = DT_PLANNER
        self.nx = 7
        self.nu = 2
        self._last_winner = 0
        self._active_target: Optional[Tuple[float, float]] = None

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

        self._shield_k1_l_cnt = ctypes.c_int(0)
        self._shield_k1_l_safe = ctypes.c_int(0)
        self._shield_k1_r_cnt = ctypes.c_int(0)
        self._shield_k1_r_safe = ctypes.c_int(0)

        self._shield_k2_l_cnt = ctypes.c_int(0)
        self._shield_k2_l_safe = ctypes.c_int(0)
        self._shield_k2_r_cnt = ctypes.c_int(0)
        self._shield_k2_r_safe = ctypes.c_int(0)

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
            float(self.pure_pursuit_blend_gain),
            float(self.lookahead_blend_m),
            float(self.v_cruise_max),
            float(self.v_cruise_min),
            float(self.straight_lock_dist_m),
            float(self.straight_lock_speed_mps),
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
            float(self.pure_pursuit_blend_gain),
            float(self.lookahead_blend_m),
            float(self.v_cruise_max),
            float(self.v_cruise_min),
            float(self.straight_lock_dist_m),
            float(self.straight_lock_speed_mps),
        )

        if not self._handle:
            raise RuntimeError("Failed to create C++ CurvilinearSLMPCNative instance")

    def reset_active_target(self) -> None:
        """Resets the currently tracked active corridor/target line."""
        self._active_target = None
        self._last_winner = 0

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
            except Exception:
                pass

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

            # If target 1 is closer to our currently followed line, target 1 is Incumbent
            if d1 < d0 and d1 <= 0.45:
                prim_target = targets[1]
                alt_target = targets[0]
            elif d0 <= 0.45:
                prim_target = targets[0]
                alt_target = targets[1]

        tx_prim, ty_prim, ap_prim = prim_target
        theta_prim = math.atan2(tx_prim, max(0.20, ty_prim))

        # ----------------------------------------------------------------------
        # 1. SOLVE INCUMBENT (PRIMARY) RACING LINE
        # ----------------------------------------------------------------------
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

        # ----------------------------------------------------------------------
        # 2. SOLVE SMART CHALLENGER ALTERNATIVE LINE (if available)
        # ----------------------------------------------------------------------
        has_challenger = False
        status_alt = -1
        cmd_steer_alt = 0.0
        cmd_speed_alt = self.v_cruise_min
        cost_alt = 1e9
        best_line_alt = []

        if alt_target is not None and hasattr(self, "_handle_alt") and self._handle_alt:
            tx_alt, ty_alt, ap_alt = alt_target
            if math.hypot(tx_alt - tx_prim, ty_alt - ty_prim) >= 0.12 or abs(tx_alt - tx_prim) >= 0.08:
                has_challenger = True
                theta_alt = math.atan2(tx_alt, max(0.20, ty_alt))
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
                cmd_steer_alt = float(self._out_steer.value)
                cmd_speed_alt = float(self._out_speed.value)
                cost_alt = float(self._out_best_cost.value)
                n_best_alt = _lib.sl_mpc_get_best_line(self._handle_alt, self._buf_xs1, self._buf_ys1, self._max_pts)
                best_line_alt = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_best_alt)]

        # ----------------------------------------------------------------------
        # 3. ARBITRATION BETWEEN INCUMBENT AND CHALLENGER
        # ----------------------------------------------------------------------
        candidate_lines = [best_line_prim, best_line_alt] if (has_challenger and best_line_alt) else [best_line_prim]

        winner = 0
        if has_challenger:
            primary_ok = (status_prim in (0, 1))
            alt_ok = (status_alt in (0, 1))
            if not primary_ok and alt_ok:
                # Primary line blocked/infeasible, challenger clear: immediate safety switch
                winner = 1
            elif primary_ok and not alt_ok:
                # Challenger infeasible, primary safe: retain primary
                winner = 0
            elif primary_ok and alt_ok and status_prim == 0:
                # Challenger must strictly beat Incumbent by the hysteresis cost margin
                if (cost_alt + self.alt_switch_hysteresis) < cost_prim:
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

        # Extract Emergency Shield evasion branches (Step 1 Left/Right, Step 2 Left/Right)
        _lib.sl_mpc_get_shield_branches(
            win_handle,
            self._buf_xs1, self._buf_ys1, ctypes.byref(self._shield_k1_l_cnt), ctypes.byref(self._shield_k1_l_safe),
            self._buf_xs2, self._buf_ys2, ctypes.byref(self._shield_k1_r_cnt), ctypes.byref(self._shield_k1_r_safe),
            self._buf_xs3, self._buf_ys3, ctypes.byref(self._shield_k2_l_cnt), ctypes.byref(self._shield_k2_l_safe),
            self._buf_xs4, self._buf_ys4, ctypes.byref(self._shield_k2_r_cnt), ctypes.byref(self._shield_k2_r_safe),
            ctypes.byref(self._shield_active), ctypes.byref(self._shield_chosen)
        )

        n_k1_l = int(self._shield_k1_l_cnt.value)
        n_k1_r = int(self._shield_k1_r_cnt.value)
        n_k2_l = int(self._shield_k2_l_cnt.value)
        n_k2_r = int(self._shield_k2_r_cnt.value)

        k1_left_branch = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_k1_l)] if n_k1_l > 0 else []
        k1_right_branch = [[float(self._buf_xs2[i]), float(self._buf_ys2[i])] for i in range(n_k1_r)] if n_k1_r > 0 else []
        k2_left_branch = [[float(self._buf_xs3[i]), float(self._buf_ys3[i])] for i in range(n_k2_l)] if n_k2_l > 0 else []
        k2_right_branch = [[float(self._buf_xs4[i]), float(self._buf_ys4[i])] for i in range(n_k2_r)] if n_k2_r > 0 else []

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

        shield_info = {
            "active": bool(self._shield_active.value),
            "chosen_id": chosen_id,
            "chosen_name": chosen_name,
            "k1_left": k1_left_branch,
            "k1_left_safe": bool(self._shield_k1_l_safe.value),
            "k1_right": k1_right_branch,
            "k1_right_safe": bool(self._shield_k1_r_safe.value),
            "k2_left": k2_left_branch,
            "k2_left_safe": bool(self._shield_k2_l_safe.value),
            "k2_right": k2_right_branch,
            "k2_right_safe": bool(self._shield_k2_r_safe.value),
        }

        # ----------------------------------------------------------------------
        # 4. PACKAGE ACTIVE CONTROL COMMANDS
        # ----------------------------------------------------------------------
        if winner == 1:
            best_line = best_line_alt
            cmd_steer = cmd_steer_alt
            cmd_speed = cmd_speed_alt
            status_desc = "CHALLENGER_OVERTHROW"
            active_cost = cost_alt
        elif status_prim == 1:
            best_line = best_line_prim
            cmd_steer = 0.0
            cmd_speed = self.straight_lock_speed_mps
            status_desc = "STRAIGHT_LOCKED"
            active_cost = 0.0
        elif status_prim == 0:
            best_line = best_line_prim
            cmd_steer = cmd_steer_prim
            cmd_speed = cmd_speed_prim
            status_desc = "OPTIMAL_OSQP_MPC"
            active_cost = cost_prim
        else:
            best_line = best_line_prim
            cmd_steer = cmd_steer_prim
            cmd_speed = self.v_cruise_min
            status_desc = "FALLBACK_SL_MPC"
            active_cost = cost_prim

        return {
            "cmd_steer_rad": cmd_steer,
            "cmd_speed_mps": cmd_speed,
            "osqp_status": "solved" if (status_prim == 0 or status_alt == 0) else "fallback",
            "osqp_iters": iters_prim,
            "best_cost": active_cost,
            "cost_primary": cost_prim,
            "cost_alt": cost_alt if has_challenger else None,
            "winner_id": winner,
            "status": status_desc,
            "best_line": best_line,
            "guide_line": guide_line_win,
            "candidate_lines": candidate_lines,
            "corridor_left": corridor_left,
            "corridor_right": corridor_right,
            "shield": shield_info,
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
