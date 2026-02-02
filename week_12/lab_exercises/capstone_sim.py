#!/usr/bin/env python3
"""
Week 12: Capstone Integration Module
=====================================
TC70045E Robotics & Drone Engineering - Dr. Abdul Manan Khan, UWL

Self-contained integration module combining concepts from all 12 weeks:
- Quadrotor dynamics (Week 6)
- EKF state estimation (Week 2)
- A* path planning (Week 8)
- B-spline trajectory generation (Week 9)
- PID/LQR controller (Week 11)
- Obstacle detection (Week 10)
"""

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.linalg import solve_continuous_are
from scipy.ndimage import binary_dilation
import heapq
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)


@dataclass
class QuadrotorParams:
    mass: float = 1.5
    Ixx: float = 0.0348
    Iyy: float = 0.0459
    Izz: float = 0.0977
    arm_length: float = 0.25
    g: float = 9.81
    max_thrust: float = 30.0
    max_torque: float = 5.0


class QuadrotorDynamics:
    """6-DOF quadrotor simulator. State: [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r]."""

    def __init__(self, params=None):
        self.params = params or QuadrotorParams()
        self.state = np.zeros(12)

    def reset(self, position=None):
        self.state = np.zeros(12)
        if position is not None:
            self.state[0:3] = position

    def get_position(self):
        return self.state[0:3].copy()

    def get_velocity(self):
        return self.state[3:6].copy()

    def get_attitude(self):
        return self.state[6:9].copy()

    def get_angular_velocity(self):
        return self.state[9:12].copy()

    def step(self, control, dt):
        p = self.params
        T = np.clip(control[0], 0, p.max_thrust)
        # control[1:4] are desired angles (phi_d, theta_d, psi_d)
        # Inner-loop PD attitude controller converts to torques
        phi_d = np.clip(control[1], -np.pi/4, np.pi/4)
        theta_d = np.clip(control[2], -np.pi/4, np.pi/4)
        psi_d = control[3]
        phi, theta, psi = self.state[6:9]
        pp, qq, rr = self.state[9:12]
        kp_att, kd_att = 8.0, 3.0
        tau_x = kp_att*(phi_d - phi) - kd_att*pp
        tau_y = kp_att*(theta_d - theta) - kd_att*qq
        tau_z = 2.0*(psi_d - psi) - 1.0*rr
        tau = np.clip(np.array([tau_x, tau_y, tau_z]), -p.max_torque, p.max_torque)
        u = np.array([T, tau[0], tau[1], tau[2]])

        def f(s, u):
            _x, _y, _z, vx, vy, vz, phi, th, psi, pp, qq, rr = s
            cphi, sphi = np.cos(phi), np.sin(phi)
            cth, sth = np.cos(th), np.sin(th)
            cpsi, spsi = np.cos(psi), np.sin(psi)
            ax = (u[0]/p.mass)*(cpsi*sth*cphi + spsi*sphi)
            ay = (u[0]/p.mass)*(spsi*sth*cphi - cpsi*sphi)
            az = (u[0]/p.mass)*(cth*cphi) - p.g
            dp = (u[1] + (p.Iyy-p.Izz)*qq*rr)/p.Ixx
            dq = (u[2] + (p.Izz-p.Ixx)*pp*rr)/p.Iyy
            dr = (u[3] + (p.Ixx-p.Iyy)*pp*qq)/p.Izz
            cth_s = max(abs(cth), 1e-6)*np.sign(cth) if abs(cth) > 1e-10 else 1e-6
            dphi = pp + (sphi*sth/cth_s)*qq + (cphi*sth/cth_s)*rr
            dtheta = cphi*qq - sphi*rr
            dpsi = (sphi/cth_s)*qq + (cphi/cth_s)*rr
            return np.array([vx, vy, vz, ax, ay, az, dphi, dtheta, dpsi, dp, dq, dr])

        k1 = f(self.state, u)
        k2 = f(self.state + 0.5*dt*k1, u)
        k3 = f(self.state + 0.5*dt*k2, u)
        k4 = f(self.state + dt*k3, u)
        self.state += (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        # Clamp angles and angular rates for stability
        self.state[6:9] = np.clip(self.state[6:9], -np.pi/2, np.pi/2)
        self.state[9:12] = np.clip(self.state[9:12], -10.0, 10.0)
        if self.state[2] < 0:
            self.state[2] = 0
            self.state[5] = max(0, self.state[5])
        return self.state.copy()

    def add_noise(self, std=0.001):
        n = np.random.randn(12)*std
        n[0:3] *= 0.1
        n[6:9] *= 0.05
        self.state += n


class IMUSensor:
    def __init__(self, accel_noise=0.1, gyro_noise=0.01, accel_bias=0.02, gyro_bias=0.005):
        self.an = accel_noise
        self.gn = gyro_noise
        self.ab = np.random.randn(3)*accel_bias
        self.gb = np.random.randn(3)*gyro_bias

    def measure(self, true_accel, true_gyro):
        return (true_accel + self.ab + np.random.randn(3)*self.an,
                true_gyro + self.gb + np.random.randn(3)*self.gn)


class GPSSensor:
    def __init__(self, noise_std=0.5, dropout_prob=0.02):
        self.ns = noise_std
        self.dp = dropout_prob

    def measure(self, true_pos):
        if np.random.random() < self.dp:
            return None
        return true_pos + np.random.randn(3)*self.ns


class ObstacleDetector:
    def __init__(self, max_range=8.0, noise_std=0.1, fp_rate=0.01):
        self.mr = max_range
        self.ns = noise_std
        self.fp = fp_rate

    def detect(self, drone_pos, obstacles):
        dets = []
        for c, r in obstacles:
            if np.linalg.norm(drone_pos - c) < self.mr:
                nc = c + np.random.randn(3)*self.ns
                nr = max(0.1, r + np.random.randn()*self.ns*0.5)
                dets.append((nc, nr))
        if np.random.random() < self.fp:
            dets.append((drone_pos + np.random.randn(3)*3.0, 0.3))
        return dets


class EKF:
    """Extended Kalman Filter. State: [x,y,z,vx,vy,vz,phi,theta,psi]."""

    def __init__(self):
        self.n = 9
        self.x = np.zeros(self.n)
        self.P = np.eye(self.n)*0.1
        self.Q = np.diag([0.01]*3 + [0.1]*3 + [0.01]*3)
        self.R_gps = np.diag([0.25, 0.25, 0.25])

    def predict(self, dt, accel=None, gyro=None):
        x = self.x.copy()
        if accel is not None:
            phi, th, psi = x[6], x[7], x[8]
            cp, sp = np.cos(phi), np.sin(phi)
            ct, st = np.cos(th), np.sin(th)
            cps, sps = np.cos(psi), np.sin(psi)
            R = np.array([[cps*ct, cps*st*sp-sps*cp, cps*st*cp+sps*sp],
                          [sps*ct, sps*st*sp+cps*cp, sps*st*cp-cps*sp],
                          [-st, ct*sp, ct*cp]])
            a_w = R @ accel + np.array([0, 0, -9.81])
            x[0:3] += x[3:6]*dt + 0.5*a_w*dt**2
            x[3:6] += a_w*dt
        else:
            x[0:3] += x[3:6]*dt
        if gyro is not None:
            x[6:9] += gyro*dt
        F = np.eye(self.n)
        F[0,3] = F[1,4] = F[2,5] = dt
        self.x = x
        self.P = F @ self.P @ F.T + self.Q*dt

    def update_gps(self, gps_pos):
        H = np.zeros((3, self.n))
        H[0,0] = H[1,1] = H[2,2] = 1
        y = gps_pos - H @ self.x
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(self.n) - K @ H) @ self.P

    def get_position(self):
        return self.x[0:3].copy()

    def get_velocity(self):
        return self.x[3:6].copy()


class OccupancyGrid3D:
    def __init__(self, bounds, resolution=0.5):
        self.bmin = np.array(bounds[0], dtype=float)
        self.bmax = np.array(bounds[1], dtype=float)
        self.res = resolution
        self.shape = np.ceil((self.bmax - self.bmin)/resolution).astype(int)
        self.grid = np.zeros(self.shape, dtype=float)

    def w2g(self, pos):
        return np.clip(((pos - self.bmin)/self.res).astype(int), 0, self.shape-1)

    def g2w(self, idx):
        return self.bmin + (idx + 0.5)*self.res

    def is_occupied(self, pos):
        i = self.w2g(pos)
        return self.grid[i[0], i[1], i[2]] > 0.5

    def add_obstacle_sphere(self, center, radius):
        rc = int(np.ceil(radius/self.res))
        ci = self.w2g(center)
        for dx in range(-rc, rc+1):
            for dy in range(-rc, rc+1):
                for dz in range(-rc, rc+1):
                    idx = ci + np.array([dx, dy, dz])
                    if np.all(idx >= 0) and np.all(idx < self.shape):
                        if np.linalg.norm(self.g2w(idx) - center) <= radius:
                            self.grid[idx[0], idx[1], idx[2]] = 1.0


class AStarPlanner:
    def __init__(self, grid, safety_margin=0.5):
        self.grid = grid
        self.sm = safety_margin
        self._inflated = None
        self._inflate()

    def _inflate(self):
        r = int(np.ceil(self.sm/self.grid.res))
        struct = np.zeros((2*r+1,)*3)
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                for dz in range(-r, r+1):
                    if dx*dx+dy*dy+dz*dz <= r*r:
                        struct[dx+r, dy+r, dz+r] = 1
        self._inflated = binary_dilation(self.grid.grid > 0.5, structure=struct).astype(float)

    def plan(self, start, goal, timeout=5.0):
        si = tuple(self.grid.w2g(start))
        gi = tuple(self.grid.w2g(goal))
        if self._inflated[si] > 0.5 or self._inflated[gi] > 0.5:
            return []
        t0 = time.time()
        nbrs = []
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                for dz in [-1,0,1]:
                    if dx==0 and dy==0 and dz==0:
                        continue
                    nbrs.append((dx,dy,dz,np.sqrt(dx*dx+dy*dy+dz*dz)))
        def h(a, b):
            return np.sqrt(sum((ai-bi)**2 for ai,bi in zip(a,b)))*self.grid.res
        oset = [(h(si,gi), 0, si)]
        came = {}
        gs = {si: 0}
        closed = set()
        while oset:
            if time.time()-t0 > timeout:
                return []
            _, g, cur = heapq.heappop(oset)
            if cur == gi:
                path = [self.grid.g2w(np.array(gi))]
                n = gi
                while n in came:
                    n = came[n]
                    path.append(self.grid.g2w(np.array(n)))
                path.reverse()
                return path
            if cur in closed:
                continue
            closed.add(cur)
            for dx,dy,dz,c in nbrs:
                nx = (cur[0]+dx, cur[1]+dy, cur[2]+dz)
                if not (0<=nx[0]<self.grid.shape[0] and 0<=nx[1]<self.grid.shape[1] and 0<=nx[2]<self.grid.shape[2]):
                    continue
                if self._inflated[nx] > 0.5 or nx in closed:
                    continue
                ng = g + c*self.grid.res
                if nx not in gs or ng < gs[nx]:
                    gs[nx] = ng
                    heapq.heappush(oset, (ng + h(nx,gi), ng, nx))
                    came[nx] = cur
        return []


class BSplineTrajectory:
    def __init__(self, degree=3):
        self.degree = degree
        self.splines = None
        self.total_time = 0.0

    def generate(self, waypoints, total_time=None, velocity=2.0):
        if len(waypoints) < 2:
            return False
        pts = np.array(waypoints)
        dists = [0.0]
        for i in range(1, len(pts)):
            dists.append(dists[-1] + np.linalg.norm(pts[i]-pts[i-1]))
        td = dists[-1]
        if total_time is None:
            total_time = max(td/velocity, 1.0)
        self.total_time = total_time
        t_p = np.array(dists)/max(td, 1e-6)*total_time
        k = min(self.degree, len(pts)-1)
        self.splines = []
        for d in range(3):
            try:
                self.splines.append(make_interp_spline(t_p, pts[:, d], k=k))
            except Exception:
                self.splines.append(make_interp_spline(t_p, pts[:, d], k=1))
        return True

    def evaluate(self, t):
        t = np.clip(t, 0, self.total_time)
        return np.array([float(s(t)) for s in self.splines])

    def evaluate_derivative(self, t, order=1):
        t = np.clip(t, 0, self.total_time)
        try:
            return np.array([float(s.derivative(order)(t)) for s in self.splines])
        except Exception:
            dt = 0.001
            return (self.evaluate(min(t+dt, self.total_time)) - self.evaluate(t))/dt

    def get_total_time(self):
        return self.total_time


class PIDController:
    def __init__(self, kp=None, ki=None, kd=None):
        self.kp = kp if kp is not None else np.array([6.0, 6.0, 8.0])
        self.ki = ki if ki is not None else np.array([0.5, 0.5, 1.0])
        self.kd = kd if kd is not None else np.array([4.0, 4.0, 5.0])
        self.integral = np.zeros(3)
        self.params = QuadrotorParams()

    def reset(self):
        self.integral = np.zeros(3)

    def compute(self, pos, vel, pos_des, vel_des, yaw, dt):
        err = pos_des - pos
        derr = vel_des - vel
        self.integral = np.clip(self.integral + err*dt, -5, 5)
        a_des = self.kp*err + self.ki*self.integral + self.kd*derr
        # Clamp lateral accelerations to prevent aggressive tilting
        a_des[0] = np.clip(a_des[0], -3.0, 3.0)
        a_des[1] = np.clip(a_des[1], -3.0, 3.0)
        a_des[2] = np.clip(a_des[2], -5.0, 5.0)
        a_des[2] += self.params.g
        thrust = np.clip(self.params.mass*np.linalg.norm(a_des), 0, self.params.max_thrust)
        if thrust > 0.1:
            phi_d = np.arcsin(np.clip(self.params.mass*(a_des[0]*np.sin(yaw)-a_des[1]*np.cos(yaw))/thrust, -0.5, 0.5))
            theta_d = np.arcsin(np.clip(self.params.mass*(a_des[0]*np.cos(yaw)+a_des[1]*np.sin(yaw))/(thrust*np.cos(phi_d+1e-10)), -0.5, 0.5))
        else:
            phi_d = theta_d = 0.0
        return np.array([thrust, phi_d, theta_d, 0.0])


class LQRController:
    def __init__(self):
        self.params = QuadrotorParams()
        A = np.zeros((6,6)); A[0,1]=A[2,3]=A[4,5]=1
        B = np.zeros((6,3)); B[1,0]=B[3,1]=B[5,2]=1
        Q = np.diag([10,2,10,2,15,3]); R = np.diag([1,1,1])
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P

    def compute(self, pos, vel, pos_des, vel_des, yaw, dt):
        s = np.array([pos[0],vel[0],pos[1],vel[1],pos[2],vel[2]])
        r = np.array([pos_des[0],vel_des[0],pos_des[1],vel_des[1],pos_des[2],vel_des[2]])
        a_des = self.K @ (r - s)
        # Clamp lateral accelerations to prevent aggressive tilting
        a_des[0] = np.clip(a_des[0], -3.0, 3.0)
        a_des[1] = np.clip(a_des[1], -3.0, 3.0)
        a_des[2] = np.clip(a_des[2], -5.0, 5.0)
        a_des[2] += self.params.g
        thrust = np.clip(self.params.mass*np.linalg.norm(a_des), 0, self.params.max_thrust)
        if thrust > 0.1:
            phi_d = np.arcsin(np.clip(self.params.mass*(a_des[0]*np.sin(yaw)-a_des[1]*np.cos(yaw))/thrust, -0.5, 0.5))
            theta_d = np.arcsin(np.clip(self.params.mass*(a_des[0]*np.cos(yaw)+a_des[1]*np.sin(yaw))/(thrust*np.cos(phi_d+1e-10)), -0.5, 0.5))
        else:
            phi_d = theta_d = 0.0
        return np.array([thrust, phi_d, theta_d, 0.0])


class HealthMonitor:
    def __init__(self):
        self.components = {}
        self.log = []

    def register(self, name, timeout):
        self.components[name] = {'timeout': timeout, 'last_seen': 0.0, 'status': 'OK'}

    def heartbeat(self, name, t):
        if name in self.components:
            self.components[name]['last_seen'] = t

    def check(self, t):
        status = {}
        for name, c in self.components.items():
            el = t - c['last_seen']
            c['status'] = 'CRITICAL' if el > c['timeout'] else ('WARNING' if el > c['timeout']*0.8 else 'OK')
            status[name] = c['status']
        self.log.append({'time': t, 'status': dict(status)})
        return status


class MissionState:
    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    NAVIGATE = 'NAVIGATE'
    REPLAN = 'REPLAN'
    HOVER = 'HOVER'
    LAND = 'LAND'
    COMPLETE = 'COMPLETE'
    EMERGENCY = 'EMERGENCY'


class MissionManager:
    def __init__(self, waypoints, takeoff_alt=2.0):
        self.waypoints = waypoints
        self.takeoff_alt = takeoff_alt
        self.state = MissionState.IDLE
        self.wp_idx = 0
        self.hover_pos = None
        self.log = []

    def get_state(self):
        return self.state

    def get_current_target(self, drone_pos):
        if self.state == MissionState.TAKEOFF:
            return np.array([drone_pos[0], drone_pos[1], self.takeoff_alt])
        elif self.state == MissionState.NAVIGATE and self.wp_idx < len(self.waypoints):
            return self.waypoints[self.wp_idx]
        elif self.state == MissionState.LAND:
            return np.array([drone_pos[0], drone_pos[1], 0.0])
        elif self.state == MissionState.HOVER and self.hover_pos is not None:
            return self.hover_pos
        return drone_pos

    def update(self, drone_pos, t):
        prev = self.state
        if self.state == MissionState.IDLE:
            self.state = MissionState.TAKEOFF
        elif self.state == MissionState.TAKEOFF:
            if abs(drone_pos[2] - self.takeoff_alt) < 0.3:
                self.state = MissionState.NAVIGATE
                self.wp_idx = 0
        elif self.state == MissionState.NAVIGATE:
            if self.wp_idx < len(self.waypoints):
                if np.linalg.norm(drone_pos - self.waypoints[self.wp_idx]) < 0.5:
                    self.wp_idx += 1
                    if self.wp_idx >= len(self.waypoints):
                        self.state = MissionState.LAND
            else:
                self.state = MissionState.LAND
        elif self.state == MissionState.LAND:
            if drone_pos[2] < 0.15:
                self.state = MissionState.COMPLETE
        if prev != self.state:
            self.log.append({'time': t, 'from': prev, 'to': self.state})
            logger.info(f"Mission: {prev} -> {self.state} at t={t:.1f}s")


class PerformanceLogger:
    def __init__(self):
        self.times = []
        self.positions = []
        self.desired_positions = []
        self.velocities = []
        self.controls = []
        self.tracking_errors = []
        self.estimation_errors = []
        self.mission_states = []
        self.planning_times = []

    def log(self, t, pos, pos_des, vel, control, est_error, mission_state):
        self.times.append(t)
        self.positions.append(pos.copy())
        self.desired_positions.append(pos_des.copy())
        self.velocities.append(vel.copy())
        self.controls.append(control.copy())
        self.tracking_errors.append(np.linalg.norm(pos - pos_des))
        self.estimation_errors.append(np.linalg.norm(est_error))
        self.mission_states.append(mission_state)

    def log_planning_time(self, dt):
        self.planning_times.append(dt)

    def compute_metrics(self):
        if not self.tracking_errors:
            return {}
        e = np.array(self.tracking_errors)
        return {
            'rmse': float(np.sqrt(np.mean(e**2))),
            'max_error': float(np.max(e)),
            'mean_error': float(np.mean(e)),
            'total_time': self.times[-1] - self.times[0] if len(self.times) > 1 else 0,
            'avg_planning_time': float(np.mean(self.planning_times)) if self.planning_times else 0,
            'mission_complete': self.mission_states[-1] == MissionState.COMPLETE if self.mission_states else False,
        }

    def get_arrays(self):
        return {k: np.array(v) for k, v in [
            ('times', self.times), ('positions', self.positions),
            ('desired', self.desired_positions), ('velocities', self.velocities),
            ('controls', self.controls), ('tracking_errors', self.tracking_errors),
            ('estimation_errors', self.estimation_errors)]}


class AutonomousDroneSystem:
    """Complete integrated autonomous drone system."""

    def __init__(self, controller_type='lqr', enable_replanning=True, sensor_noise_scale=1.0):
        self.drone = QuadrotorDynamics()
        self.imu = IMUSensor(accel_noise=0.1*sensor_noise_scale, gyro_noise=0.01*sensor_noise_scale)
        self.gps = GPSSensor(noise_std=0.5*sensor_noise_scale)
        self.detector = ObstacleDetector(max_range=8.0, noise_std=0.1*sensor_noise_scale)
        self.ekf = EKF()
        self.grid_map = None
        self.planner = None
        self.trajectory = BSplineTrajectory()
        self.controller_type = controller_type
        self.controller = PIDController() if controller_type == 'pid' else LQRController()
        self.mission = None
        self.enable_replanning = enable_replanning
        self.health = HealthMonitor()
        for n, to in [('ekf',0.1),('planner',2.0),('controller',0.05),('gps',1.0)]:
            self.health.register(n, to)
        self.perf = PerformanceLogger()
        self.t = 0.0
        self.dt = 0.01
        self.obstacles = []
        self.detected_obstacles = []
        self.traj_start_time = 0.0
        self.needs_replan = False

    def setup_environment(self, bounds, obstacles, waypoints, takeoff_alt=2.0):
        self.grid_map = OccupancyGrid3D(bounds)
        for c, r in obstacles:
            self.obstacles.append((np.array(c), r))
            self.grid_map.add_obstacle_sphere(np.array(c), r)
        self.planner = AStarPlanner(self.grid_map, safety_margin=0.5)
        self.mission = MissionManager([np.array(w) for w in waypoints], takeoff_alt)

    def plan_path(self, start, goal):
        t0 = time.time()
        path = self.planner.plan(start, goal)
        self.perf.log_planning_time(time.time()-t0)
        if len(path) < 2:
            return False
        self.trajectory.generate(path, velocity=1.5)
        self.traj_start_time = self.t
        self.health.heartbeat('planner', self.t)
        return True

    def run_mission(self, max_time=120.0, verbose=True):
        self.t = 0.0
        self.drone.reset(np.array([0.0, 0.0, 0.0]))
        self.ekf.x[0:3] = self.drone.get_position()
        if isinstance(self.controller, PIDController):
            self.controller.reset()
        takeoff_tgt = np.array([0.0, 0.0, self.mission.takeoff_alt])
        self.trajectory.generate([self.drone.get_position(), takeoff_tgt], velocity=1.0)
        self.traj_start_time = 0.0
        gps_timer = replan_cd = 0.0

        while self.t < max_time:
            tp = self.drone.get_position()
            tv = self.drone.get_velocity()
            ta = self.drone.get_attitude()
            tav = self.drone.get_angular_velocity()
            am, gm = self.imu.measure(np.array([0,0,9.81]), tav)
            gps_timer += self.dt
            gps_m = None
            if gps_timer >= 0.2:
                gps_m = self.gps.measure(tp)
                gps_timer = 0.0
                if gps_m is not None:
                    self.health.heartbeat('gps', self.t)
            # Use constant-velocity predict (more stable); GPS corrections handle position
            self.ekf.predict(self.dt, accel=None, gyro=None)
            if gps_m is not None:
                self.ekf.update_gps(gps_m)
            self.health.heartbeat('ekf', self.t)
            ep = self.ekf.get_position()
            ev = self.ekf.get_velocity()
            ee = ep - tp
            self.mission.update(ep, self.t)
            st = self.mission.get_state()
            if st == MissionState.COMPLETE:
                self.perf.log(self.t, tp, tp, tv, np.zeros(4), ee, st)
                break
            if int(self.t*2) > int((self.t-self.dt)*2):
                dets = self.detector.detect(ep, self.obstacles)
                if dets and self.enable_replanning:
                    new_o = []
                    for dc, dr in dets:
                        if not any(np.linalg.norm(dc-kc)<1.0 for kc,_ in self.detected_obstacles):
                            new_o.append((dc, dr))
                            self.detected_obstacles.append((dc, dr))
                    if new_o and replan_cd <= 0:
                        self.needs_replan = True
                        replan_cd = 2.0
            replan_cd -= self.dt
            if st == MissionState.NAVIGATE:
                tt = self.t - self.traj_start_time
                if self.needs_replan and self.enable_replanning:
                    tgt = self.mission.get_current_target(ep)
                    for c, r in self.detected_obstacles[-5:]:
                        self.grid_map.add_obstacle_sphere(c, r)
                    self.planner._inflate()
                    if self.plan_path(ep, tgt):
                        self.needs_replan = False
                        tt = 0.0
                if tt > self.trajectory.get_total_time():
                    tgt = self.mission.get_current_target(ep)
                    if np.linalg.norm(ep - tgt) > 0.5:
                        if not self.plan_path(ep, tgt):
                            # Fallback: direct trajectory if A* fails
                            self.trajectory.generate([ep, tgt], velocity=1.5)
                            self.traj_start_time = self.t
                        tt = 0.0
            if st in [MissionState.TAKEOFF, MissionState.NAVIGATE]:
                tt = self.t - self.traj_start_time
                pd = self.trajectory.evaluate(tt)
                vd = self.trajectory.evaluate_derivative(tt)
            elif st == MissionState.LAND:
                pd = self.mission.get_current_target(ep)
                vd = np.array([0,0,-0.5])
            else:
                pd = self.mission.get_current_target(ep)
                vd = np.zeros(3)
            yaw = self.drone.get_attitude()[2]
            # Inner-loop uses high-rate onboard state (tp, tv); outer-loop uses EKF (ep)
            ctrl = self.controller.compute(tp, tv, pd, vd, yaw, self.dt)
            self.health.heartbeat('controller', self.t)
            self.drone.add_noise(0.0005)
            self.drone.step(ctrl, self.dt)
            self.perf.log(self.t, tp, pd, tv, ctrl, ee, st)
            self.t += self.dt

        metrics = self.perf.compute_metrics()
        if verbose:
            logger.info(f"Metrics: {metrics}")
        return metrics


def demo_mission():
    system = AutonomousDroneSystem(controller_type='lqr', enable_replanning=True)
    bounds = ([-2,-2,-0.5], [15,15,8])
    obstacles = [([5,3,2],1.0), ([8,7,3],1.0), ([11,5,2.5],0.8)]
    waypoints = [[3,2,2], [6,5,3], [9,9,2.5], [12,10,2]]
    system.setup_environment(bounds, obstacles, waypoints)
    metrics = system.run_mission(max_time=80.0)
    data = system.perf.get_arrays()
    fig = plt.figure(figsize=(14,10))
    ax3d = fig.add_subplot(2,2,1, projection='3d')
    p = data['positions']; d = data['desired']
    ax3d.plot(p[:,0],p[:,1],p[:,2],'b-',lw=1,label='Actual')
    ax3d.plot(d[:,0],d[:,1],d[:,2],'r--',lw=1,alpha=0.7,label='Desired')
    for c,r in obstacles:
        u,v = np.mgrid[0:2*np.pi:10j,0:np.pi:6j]
        ax3d.plot_surface(c[0]+r*np.cos(u)*np.sin(v),c[1]+r*np.sin(u)*np.sin(v),c[2]+r*np.cos(v),alpha=0.3,color='red')
    for w in waypoints:
        ax3d.scatter(*w,c='green',s=80,marker='*')
    ax3d.set_xlabel('X'); ax3d.set_ylabel('Y'); ax3d.set_zlabel('Z')
    ax3d.set_title('3D Trajectory'); ax3d.legend(fontsize=8)
    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(data['times'],data['tracking_errors'],'b-',lw=0.8)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Error (m)')
    ax2.set_title(f"Tracking Error (RMSE={metrics.get('rmse',0):.3f}m)"); ax2.grid(True,alpha=0.3)
    ax3 = fig.add_subplot(2,2,3)
    ax3.plot(data['times'],p[:,0],label='x'); ax3.plot(data['times'],p[:,1],label='y'); ax3.plot(data['times'],p[:,2],label='z')
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Position (m)'); ax3.set_title('Position'); ax3.legend(); ax3.grid(True,alpha=0.3)
    ax4 = fig.add_subplot(2,2,4)
    ax4.plot(data['times'],data['controls'][:,0],label='Thrust')
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Thrust (N)'); ax4.set_title('Control'); ax4.legend(); ax4.grid(True,alpha=0.3)
    plt.tight_layout()
    plt.savefig('week12_capstone_demo.png',dpi=150,bbox_inches='tight')
    logger.info("Saved: week12_capstone_demo.png"); plt.close()
    return metrics


def main():
    print("="*60)
    print("Week 12: Capstone Integration Module")
    print("TC70045E - Dr. Abdul Manan Khan, UWL")
    print("="*60)
    metrics = demo_mission()
    print("\nFinal Metrics:")
    for k,v in metrics.items():
        print(f"  {k}: {v}")


if __name__ == '__main__':
    main()
