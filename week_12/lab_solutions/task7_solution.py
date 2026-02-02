#!/usr/bin/env python3
"""Task 7 Solution: Comprehensive System Evaluation"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from capstone_sim import AutonomousDroneSystem, MissionState


def _make_env(difficulty='medium'):
    if difficulty == 'easy':
        return {'bounds': ([-2,-2,-0.5],[12,12,6]),
                'obstacles': [([5,5,2],1.0)],
                'waypoints': [[3,3,2],[8,8,2]]}
    elif difficulty == 'hard':
        return {'bounds': ([-2,-2,-0.5],[20,20,10]),
                'obstacles': [([3,2,2],0.8),([5,5,3],1.0),([8,4,2.5],0.9),
                              ([11,8,3],1.1),([14,6,2],0.7),([16,12,3.5],1.0)],
                'waypoints': [[2,1,2],[5,3,3],[8,7,2.5],[11,5,3],[14,10,2.5],[18,15,2]]}
    else:
        return {'bounds': ([-2,-2,-0.5],[15,15,8]),
                'obstacles': [([5,3,2],1.0),([8,7,3],1.0),([11,5,2.5],0.8)],
                'waypoints': [[3,2,2],[6,5,3],[9,9,2.5],[12,10,2]]}


def _run(ctrl='lqr', replan=True, noise=1.0, difficulty='medium'):
    env = _make_env(difficulty)
    s = AutonomousDroneSystem(controller_type=ctrl, enable_replanning=replan, sensor_noise_scale=noise)
    s.setup_environment(env['bounds'], env['obstacles'], env['waypoints'])
    m = s.run_mission(max_time=100.0, verbose=False)
    return m, s


def run_evaluation_suite(n_missions=3):
    results = {}
    for diff in ['easy', 'medium', 'hard']:
        runs = []
        for i in range(n_missions):
            m, _ = _run(difficulty=diff)
            runs.append(m)
            print(f"  {diff} run {i+1}: RMSE={m.get('rmse',999):.3f}, success={m.get('mission_complete',False)}")
        results[diff] = runs
    return results


def compare_controllers(n_runs=3):
    results = {}
    for ctrl in ['pid', 'lqr']:
        runs = []
        for _ in range(n_runs):
            m, s = _run(ctrl=ctrl)
            data = s.perf.get_arrays()
            effort = np.sum(np.abs(data['controls'][:,0]))*s.dt if len(data['controls']) > 0 else 0
            m['control_effort'] = effort
            runs.append(m)
        results[ctrl] = runs
        avg_rmse = np.mean([r.get('rmse',999) for r in runs])
        print(f"  {ctrl.upper()}: avg RMSE={avg_rmse:.3f}m")
    return results


def compare_replanning(n_runs=3):
    results = {}
    for rp in [True, False]:
        runs = []
        for _ in range(n_runs):
            m, _ = _run(replan=rp)
            runs.append(m)
        results[rp] = runs
        sr = np.mean([1 if r.get('mission_complete',False) else 0 for r in runs])
        print(f"  Replan={rp}: success rate={sr*100:.0f}%")
    return results


def generate_evaluation_report(suite, ctrl, replan, filename='task7_evaluation.png'):
    fig, axes = plt.subplots(2, 4, figsize=(24, 10))

    # Panel 1: Success rate by difficulty
    diffs = ['easy','medium','hard']
    sr = [np.mean([1 if r.get('mission_complete',False) else 0 for r in suite.get(d,[])]) for d in diffs]
    axes[0,0].bar(diffs, [s*100 for s in sr], color=['#4CAF50','#ff9800','#f44336'])
    axes[0,0].set_title('Success Rate by Difficulty'); axes[0,0].set_ylabel('%'); axes[0,0].set_ylim(0,105)

    # Panel 2: RMSE by difficulty
    rmses = [[r.get('rmse',0) for r in suite.get(d,[])] for d in diffs]
    bp = axes[0,1].boxplot(rmses, labels=diffs, patch_artist=True)
    for patch, color in zip(bp['boxes'], ['#4CAF50','#ff9800','#f44336']):
        patch.set_facecolor(color); patch.set_alpha(0.5)
    axes[0,1].set_title('RMSE by Difficulty'); axes[0,1].set_ylabel('RMSE (m)')

    # Panel 3: Planning time
    all_pt = []
    for d in diffs:
        for r in suite.get(d,[]):
            all_pt.append(r.get('avg_planning_time',0)*1000)
    if all_pt:
        axes[0,2].hist(all_pt, bins=15, color='#2196f3', alpha=0.7)
    axes[0,2].set_title('Planning Time Distribution'); axes[0,2].set_xlabel('ms')

    # Panel 4: Controller RMSE comparison
    ctrl_names = list(ctrl.keys())
    ctrl_rmse = [np.mean([r.get('rmse',0) for r in ctrl[c]]) for c in ctrl_names]
    axes[0,3].bar([c.upper() for c in ctrl_names], ctrl_rmse, color=['#667eea','#764ba2'])
    axes[0,3].set_title('Controller RMSE'); axes[0,3].set_ylabel('RMSE (m)')

    # Panel 5: Controller effort
    ctrl_eff = [np.mean([r.get('control_effort',0) for r in ctrl[c]]) for c in ctrl_names]
    axes[1,0].bar([c.upper() for c in ctrl_names], ctrl_eff, color=['#667eea','#764ba2'])
    axes[1,0].set_title('Control Effort'); axes[1,0].set_ylabel('Total Effort (N*s)')

    # Panel 6: Replanning comparison
    rp_labels = ['With Replan', 'No Replan']
    rp_sr = [np.mean([1 if r.get('mission_complete',False) else 0 for r in replan.get(k,[])]) for k in [True, False]]
    axes[1,1].bar(rp_labels, [s*100 for s in rp_sr], color=['#4CAF50','#f44336'])
    axes[1,1].set_title('Replanning Impact'); axes[1,1].set_ylabel('Success %'); axes[1,1].set_ylim(0,105)

    # Panel 7: Best run tracking error
    best_run = None; best_rmse = 999
    for d in diffs:
        for r in suite.get(d,[]):
            if r.get('rmse',999) < best_rmse:
                best_rmse = r.get('rmse',999)
    # Run one more to get time series
    _, best_sys = _run(difficulty='easy')
    bd = best_sys.perf.get_arrays()
    if len(bd['times']) > 0:
        axes[1,2].plot(bd['times'], bd['tracking_errors'], 'b-', lw=0.8)
    axes[1,2].set_title(f'Best Run Error (RMSE={best_rmse:.3f}m)')
    axes[1,2].set_xlabel('Time (s)'); axes[1,2].set_ylabel('Error (m)'); axes[1,2].grid(True, alpha=0.3)

    # Panel 8: Summary table
    axes[1,3].axis('off')
    summary = [
        ['Metric', 'Value'],
        ['Easy Success', f"{sr[0]*100:.0f}%"],
        ['Medium Success', f"{sr[1]*100:.0f}%"],
        ['Hard Success', f"{sr[2]*100:.0f}%"],
        ['PID RMSE', f"{ctrl_rmse[0]:.3f}m" if len(ctrl_rmse)>0 else 'N/A'],
        ['LQR RMSE', f"{ctrl_rmse[1]:.3f}m" if len(ctrl_rmse)>1 else 'N/A'],
        ['Avg Plan Time', f"{np.mean(all_pt):.1f}ms" if all_pt else 'N/A'],
    ]
    table = axes[1,3].table(cellText=summary, loc='center', cellLoc='center')
    table.auto_set_font_size(False); table.set_fontsize(10); table.scale(1, 1.5)
    for i in range(len(summary)):
        for j in range(2):
            cell = table[i, j]
            if i == 0:
                cell.set_facecolor('#667eea'); cell.set_text_props(color='white', fontweight='bold')
            elif i % 2 == 0:
                cell.set_facecolor('#f0f0f0')
    axes[1,3].set_title('Summary')

    plt.suptitle('Week 12 Capstone: Comprehensive System Evaluation', fontsize=16, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Saved: {filename}")
    plt.close()


def main():
    print("Task 7 Solution: Comprehensive System Evaluation")
    print("=" * 50)
    print("\nEvaluation suite:")
    suite = run_evaluation_suite(n_missions=3)
    print("\nController comparison:")
    ctrl = compare_controllers(n_runs=2)
    print("\nReplanning comparison:")
    replan = compare_replanning(n_runs=2)
    print("\nGenerating report...")
    generate_evaluation_report(suite, ctrl, replan)
    print("\nTask 7 Solution complete.")


if __name__ == '__main__':
    main()
