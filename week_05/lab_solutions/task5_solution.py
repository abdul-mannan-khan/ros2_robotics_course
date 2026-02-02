#!/usr/bin/env python3
"""
Week 5 - Task 5 Solution: Behavior Trees for Navigation
Complete behavior tree framework with Nav2-like navigation BT.
"""

import os
import numpy as np
from enum import Enum
import heapq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises', 'data')
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


class Status(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class BTNode:
    def __init__(self, name="BTNode"):
        self.name = name
        self.children = []

    def tick(self, blackboard):
        raise NotImplementedError

    def add_child(self, child):
        self.children.append(child)
        return self


class Sequence(BTNode):
    def tick(self, blackboard):
        for child in self.children:
            status = child.tick(blackboard)
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS


class Fallback(BTNode):
    def tick(self, blackboard):
        for child in self.children:
            status = child.tick(blackboard)
            if status != Status.FAILURE:
                return status
        return Status.FAILURE


class Action(BTNode):
    pass


class Condition(BTNode):
    pass


# --- Utility: simple A* for BT navigation ---
def _astar_simple(grid, start_rc, goal_rc):
    """Minimal A* returning path as list of (row, col)."""
    if grid[start_rc[0], start_rc[1]] == 100 or grid[goal_rc[0], goal_rc[1]] == 100:
        return []
    h, w = grid.shape
    open_set = [(0, start_rc)]
    g = {start_rc: 0}
    came = {}
    visited = set()
    SQRT2 = 1.414
    ct = 0
    while open_set:
        _, cur = heapq.heappop(open_set)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == goal_rc:
            path = []
            while cur in came:
                path.append(cur)
                cur = came[cur]
            path.append(start_rc)
            path.reverse()
            return path
        for dr, dc, cost in [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),
                              (-1,-1,SQRT2),(-1,1,SQRT2),(1,-1,SQRT2),(1,1,SQRT2)]:
            nr, nc = cur[0]+dr, cur[1]+dc
            if 0 <= nr < h and 0 <= nc < w and grid[nr, nc] != 100:
                ng = g[cur] + cost
                if ng < g.get((nr, nc), float('inf')):
                    g[(nr, nc)] = ng
                    came[(nr, nc)] = cur
                    f = ng + np.sqrt((nr-goal_rc[0])**2+(nc-goal_rc[1])**2)
                    ct += 1
                    heapq.heappush(open_set, (f, (nr, nc)))
    return []


def _line_clear(grid, r0, c0, r1, c1):
    """Check if line between two grid cells is free (Bresenham)."""
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        if grid[r, c] == 100:
            return False
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return True


class IsPathClear(Condition):
    """Returns SUCCESS if direct path to goal is clear, FAILURE otherwise."""
    def tick(self, blackboard):
        pose = blackboard['robot_pose']
        goal = blackboard['current_goal']
        grid = blackboard['grid_map']
        res = blackboard['resolution']
        r0, c0 = int(pose[1]/res), int(pose[0]/res)
        r1, c1 = int(goal[1]/res), int(goal[0]/res)
        h, w = grid.shape
        r0, c0 = np.clip(r0, 0, h-1), np.clip(c0, 0, w-1)
        r1, c1 = np.clip(r1, 0, h-1), np.clip(c1, 0, w-1)
        if _line_clear(grid, r0, c0, r1, c1):
            return Status.SUCCESS
        return Status.FAILURE


class IsPathBlocked(Condition):
    """Returns SUCCESS if path is blocked."""
    def tick(self, blackboard):
        clear = IsPathClear("check")
        result = clear.tick(blackboard)
        return Status.SUCCESS if result == Status.FAILURE else Status.FAILURE


class NavigateToPose(Action):
    """Move robot toward goal using A* path following."""
    def __init__(self, name="NavigateToPose"):
        super().__init__(name)
        self._path = None
        self._path_idx = 0

    def _find_free_cell(self, grid, r, c):
        """Find nearest free cell to (r, c)."""
        h, w = grid.shape
        r, c = np.clip(r, 1, h-2), np.clip(c, 1, w-2)
        if grid[r, c] != 100:
            return r, c
        for rad in range(1, 15):
            for dr in range(-rad, rad+1):
                for dc in range(-rad, rad+1):
                    nr, nc = r+dr, c+dc
                    if 0 < nr < h-1 and 0 < nc < w-1 and grid[nr, nc] == 0:
                        return nr, nc
        return r, c

    def tick(self, blackboard):
        pose = blackboard['robot_pose']
        goal = blackboard['current_goal']
        grid = blackboard['grid_map']
        res = blackboard['resolution']

        # Check if reached
        dist = np.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if dist < 0.15:
            self._path = None
            return Status.SUCCESS

        # Plan path if needed
        if self._path is None or len(self._path) == 0:
            sr, sc = int(pose[1]/res), int(pose[0]/res)
            gr, gc = int(goal[1]/res), int(goal[0]/res)
            sr, sc = self._find_free_cell(grid, sr, sc)
            gr, gc = self._find_free_cell(grid, gr, gc)
            path_cells = _astar_simple(grid, (sr, sc), (gr, gc))
            if not path_cells:
                return Status.FAILURE
            self._path = [(c * res, r * res) for r, c in path_cells]
            self._path_idx = 0

        # Follow path: move toward next point
        if self._path_idx >= len(self._path):
            self._path = None
            return Status.RUNNING

        target = self._path[min(self._path_idx + 3, len(self._path) - 1)]
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        dist_to_target = np.hypot(dx, dy)

        step_size = 0.05  # meters per tick
        if dist_to_target < step_size:
            self._path_idx += 3
        else:
            ratio = step_size / dist_to_target
            blackboard['robot_pose'][0] += dx * ratio
            blackboard['robot_pose'][1] += dy * ratio
            blackboard['robot_pose'][2] = np.arctan2(dy, dx)

        return Status.RUNNING


class RecoveryBehavior(Action):
    """Rotate in place to try to find a clear path."""
    def tick(self, blackboard):
        count = blackboard.get('recovery_count', 0)
        blackboard['recovery_count'] = count + 1
        grid = blackboard['grid_map']
        res = blackboard['resolution']

        # Rotate 45 degrees
        blackboard['robot_pose'][2] += np.pi / 4

        # Try small displacement, ensure we stay in free space
        for _ in range(8):
            dx = np.random.uniform(-0.1, 0.1)
            dy = np.random.uniform(-0.1, 0.1)
            nx = np.clip(blackboard['robot_pose'][0] + dx, 0.2, 4.8)
            ny = np.clip(blackboard['robot_pose'][1] + dy, 0.2, 4.8)
            gc = int(nx / res)
            gr = int(ny / res)
            h, w = grid.shape
            if 0 <= gr < h and 0 <= gc < w and grid[gr, gc] == 0:
                blackboard['robot_pose'][0] = nx
                blackboard['robot_pose'][1] = ny
                break

        if count > 16:
            return Status.FAILURE
        return Status.SUCCESS


def build_nav_bt():
    """Build Nav2-like navigation behavior tree.

    Fallback(MainBT)
      Sequence(Navigate)
        IsPathClear
        NavigateToPose
      Sequence(PlanAndNavigate)
        NavigateToPose  (will plan via A*)
      Sequence(Recovery)
        IsPathBlocked
        RecoveryBehavior
    """
    root = Fallback("MainBT")

    # Direct navigation (path is clear)
    direct = Sequence("DirectNav")
    direct.add_child(IsPathClear("IsPathClear"))
    direct.add_child(NavigateToPose("DirectNavigate"))
    root.add_child(direct)

    # Planned navigation (uses A*)
    planned = Sequence("PlannedNav")
    planned.add_child(NavigateToPose("PlannedNavigate"))
    root.add_child(planned)

    # Recovery
    recovery = Sequence("Recovery")
    recovery.add_child(IsPathBlocked("IsBlocked"))
    recovery.add_child(RecoveryBehavior("Recover"))
    root.add_child(recovery)

    return root


def print_bt(node, indent=0):
    """Print behavior tree structure."""
    prefix = "  " * indent
    print(f"{prefix}[{node.__class__.__name__}] {node.name}")
    for child in node.children:
        print_bt(child, indent + 1)


def main():
    print("=" * 60)
    print("Task 5: Behavior Trees for Navigation")
    print("=" * 60)
    print()

    grid_map = np.load(os.path.join(DATA_DIR, 'grid_map.npy'))
    metadata = np.load(os.path.join(DATA_DIR, 'map_metadata.npy'))
    waypoints = np.load(os.path.join(DATA_DIR, 'waypoints.npy'))
    resolution = metadata[0]

    bt = build_nav_bt()
    print("Behavior Tree Structure:")
    print_bt(bt)
    print()

    blackboard = {
        'robot_pose': np.array([1.0, 1.0, 0.0]),
        'grid_map': grid_map,
        'resolution': resolution,
        'recovery_count': 0,
        'trajectory': [],
        'bt_log': [],
    }

    waypoint_results = []

    for wp_idx, wp in enumerate(waypoints[:5]):
        blackboard['current_goal'] = wp
        blackboard['recovery_count'] = 0
        # Reset NavigateToPose path for new goal
        for child in bt.children:
            for c in child.children:
                if isinstance(c, NavigateToPose):
                    c._path = None

        print(f"Navigating to waypoint {wp_idx}: ({wp[0]:.1f}, {wp[1]:.1f})")
        reached = False

        for step in range(500):
            status = bt.tick(blackboard)
            blackboard['trajectory'].append(blackboard['robot_pose'][:2].copy())
            blackboard['bt_log'].append((wp_idx, step, status.name))

            if status == Status.SUCCESS:
                dist = np.hypot(blackboard['robot_pose'][0] - wp[0],
                               blackboard['robot_pose'][1] - wp[1])
                if dist < 0.2:
                    print(f"  Reached in {step} steps (dist={dist:.3f}m)")
                    reached = True
                    break
            elif status == Status.FAILURE:
                print(f"  FAILURE at step {step}")
                break

        if not reached:
            dist = np.hypot(blackboard['robot_pose'][0] - wp[0],
                           blackboard['robot_pose'][1] - wp[1])
            print(f"  Timeout. Final dist: {dist:.3f}m")
            if dist < 0.3:
                reached = True

        waypoint_results.append(reached)

    # Plot
    traj = np.array(blackboard['trajectory'])
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    ax = axes[0]
    ax.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100,
              extent=[0, 5, 0, 5])
    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=1, alpha=0.7, label='Trajectory')
    for i, wp in enumerate(waypoints[:5]):
        color = 'green' if waypoint_results[i] else 'red'
        ax.plot(wp[0], wp[1], 'o', color=color, markersize=10)
        ax.annotate(f'WP{i}', (wp[0]+0.05, wp[1]+0.05), fontsize=9)
    ax.plot(1.0, 1.0, 'gs', markersize=12, label='Start')
    ax.legend()
    ax.set_title(f'BT Navigation ({sum(waypoint_results)}/{len(waypoint_results)} reached)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    # BT execution log
    ax = axes[1]
    log = blackboard['bt_log']
    status_map = {'SUCCESS': 0, 'RUNNING': 1, 'FAILURE': 2}
    statuses = [status_map[l[2]] for l in log]
    wps = [l[0] for l in log]
    ax.scatter(range(len(statuses)), wps, c=statuses, cmap='RdYlGn_r', s=2, alpha=0.5)
    ax.set_xlabel('Tick')
    ax.set_ylabel('Waypoint Index')
    ax.set_title('BT Execution Log (green=SUCCESS, yellow=RUNNING, red=FAILURE)')
    ax.set_yticks(range(5))

    plt.suptitle('Behavior Tree Navigation', fontsize=14)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, 'task5_behavior_tree.png')
    plt.savefig(out_path, dpi=100)
    print(f"\nSaved plot to {out_path}")

    print(f"\nResults: {sum(waypoint_results)}/{len(waypoint_results)} waypoints reached")
    print(f"Total BT ticks: {len(log)}")
    print(f"Recovery invocations: {blackboard['recovery_count']}")

    print("\n--- Key Concepts ---")
    print("- Sequence: all children must succeed (AND logic)")
    print("- Fallback: tries alternatives until one succeeds (OR logic)")
    print("- The BT orchestrates planning, execution, and recovery")
    print("- Nav2 uses BehaviorTree.CPP with XML-defined trees")


if __name__ == '__main__':
    main()
