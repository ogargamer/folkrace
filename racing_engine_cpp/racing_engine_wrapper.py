import os
import ctypes
import numpy as np

DLL_PATH = os.path.join(os.path.dirname(__file__), "racing_engine.dll")

class TUM_RacingEngine_CPP_Wrapper:
    def __init__(
        self,
        weight_curvature=3.20,
        weight_path_smoothness=0.055,
        weight_jerk=0.38,
        robot_safety_margin=0.10,
        num_progression_stages=7,
        max_solver_sub_iters=300,
        friction_coeff=0.90,
        max_speed=4.00,
        max_accel=2.00,
        max_brake=3.00,
        num_waypoints=360
    ):
        self.num_waypoints = num_waypoints
        if not os.path.exists(DLL_PATH):
            raise FileNotFoundError(f"C++ DLL not found at: {DLL_PATH}")

        self.lib = ctypes.CDLL(DLL_PATH)

        # void* create_racing_engine(...)
        self.lib.create_racing_engine.restype = ctypes.c_void_p
        self.lib.create_racing_engine.argtypes = [
            ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_int, ctypes.c_int,
            ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_int
        ]

        # void destroy_racing_engine(void*)
        self.lib.destroy_racing_engine.restype = None
        self.lib.destroy_racing_engine.argtypes = [ctypes.c_void_p]

        # void optimize_track_cpp(...)
        self.lib.optimize_track_cpp.restype = None
        self.lib.optimize_track_cpp.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double)
        ]

        self.engine_handle = self.lib.create_racing_engine(
            weight_curvature, weight_path_smoothness, weight_jerk,
            robot_safety_margin, num_progression_stages, max_solver_sub_iters,
            friction_coeff, max_speed, max_accel, max_brake, num_waypoints
        )

    def __del__(self):
        if hasattr(self, "lib") and hasattr(self, "engine_handle") and self.engine_handle:
            self.lib.destroy_racing_engine(self.engine_handle)
            self.engine_handle = None

    def optimize_racing_line(self, track_data, input_centerline=None):
        l_wall = np.array(track_data["left_wall"], dtype=np.float64)
        r_wall = np.array(track_data["right_wall"], dtype=np.float64)
        
        lx_arr = np.ascontiguousarray(l_wall[:, 0], dtype=np.float64)
        ly_arr = np.ascontiguousarray(l_wall[:, 1], dtype=np.float64)
        rx_arr = np.ascontiguousarray(r_wall[:, 0], dtype=np.float64)
        ry_arr = np.ascontiguousarray(r_wall[:, 1], dtype=np.float64)

        lx = lx_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        ly = ly_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        rx = rx_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        ry = ry_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

        # Handle optional input centerline
        if input_centerline is not None:
            c_in = np.array(input_centerline, dtype=np.float64)
        elif "centerline" in track_data:
            c_in = np.array(track_data["centerline"], dtype=np.float64)
        else:
            c_in = None

        if c_in is not None and len(c_in) > 0:
            cx_arr = np.ascontiguousarray(c_in[:, 0], dtype=np.float64)
            cy_arr = np.ascontiguousarray(c_in[:, 1], dtype=np.float64)
            cx_ptr = cx_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
            cy_ptr = cy_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
            c_count = len(c_in)
        else:
            cx_ptr = None
            cy_ptr = None
            c_count = 0

        obstacles = track_data.get("obstacles", [])
        obs_count = len(obstacles)
        obs_types = np.zeros(obs_count, dtype=np.int32)
        obs_params = np.zeros(obs_count * 5, dtype=np.float64)

        for i, obs in enumerate(obstacles):
            if obs["type"] == "circle":
                obs_types[i] = 0
                obs_params[i * 5 + 0] = obs["cx"]
                obs_params[i * 5 + 1] = obs["cy"]
                obs_params[i * 5 + 2] = obs["radius"]
            elif obs["type"] == "wall_rectangle":
                obs_types[i] = 1
                obs_params[i * 5 + 0] = obs["center_x"]
                obs_params[i * 5 + 1] = obs["center_y"]
                obs_params[i * 5 + 2] = obs.get("width", 0.30)
                obs_params[i * 5 + 3] = obs.get("extrusion", obs.get("height", 0.30))
                obs_params[i * 5 + 4] = obs.get("angle_rad", 0.0)

        c_obs_types = obs_types.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
        c_obs_params = obs_params.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

        N = self.num_waypoints
        out_rl_x = np.zeros(N, dtype=np.float64)
        out_rl_y = np.zeros(N, dtype=np.float64)
        out_c_x = np.zeros(N, dtype=np.float64)
        out_c_y = np.zeros(N, dtype=np.float64)
        out_vels = np.zeros(N, dtype=np.float64)
        out_metrics = np.zeros(6, dtype=np.float64)

        self.lib.optimize_track_cpp(
            self.engine_handle,
            lx, ly, len(l_wall),
            rx, ry, len(r_wall),
            cx_ptr, cy_ptr, c_count,
            c_obs_types, c_obs_params, obs_count,
            out_rl_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            out_rl_y.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            out_c_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            out_c_y.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            out_vels.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            out_metrics.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        )

        rl_pts = np.column_stack([out_rl_x, out_rl_y])
        c_pts = np.column_stack([out_c_x, out_c_y])
        
        diffs = np.diff(rl_pts, axis=0)
        diffs = np.vstack([diffs, rl_pts[0] - rl_pts[-1]])
        ds = np.hypot(diffs[:, 0], diffs[:, 1])

        return {
            "racing_line": rl_pts,
            "centerline": c_pts,
            "velocities": out_vels,
            "ds": ds,
            "min_apex_speed": float(out_metrics[0]),
            "top_speed": float(out_metrics[1]),
            "lap_time": float(out_metrics[2]),
            "min_clearance_cm": float(out_metrics[3]),
            "solve_time_ms": float(out_metrics[4]),
            "solve_time_us": float(out_metrics[5])
        }
