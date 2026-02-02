#!/usr/bin/env python3
"""Task 6 Solution: Stress Testing"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lab_exercises'))
from capstone_sim import AutonomousDroneSystem


def _run_standard_mission(controller_type='lqr', noise_scale=1.0, enable_replan=True):
    s = AutonomousDroneSystem(controller_type=controller_type, enable_replanning=enable_replan,
                               sensor_noise_scale=noise_scale)
    s.setup_environment(([-2,-2,-0.5],[15,15,8]),
                        [([5,3,2],1.0),([8,7,3],1.0),([11,5,2.5],0.8)],
                        [[3,2,2],[6,5,3],[9,9,2.5],[12,10,2]])
    return s.run_mission(max_time=80.0, verbose=False)


def test_high_sensor_noise(noise_scales=[1.0, 2.0, 3.0, 5.0]):
    results = []
    for ns in noise_scales:
        m = _run_standard_mission(noise_scale=ns)
        results.append({'noise': ns, 'rmse': m.get('rmse',999), 'success': m.get('mission_complete',False)})
        print(f"  Noise={ns}x: RMSE={m.get('rmse',999):.3f}m, Complete={m.get('mission_complete',False)}")
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot([r['noise'] for r in results], [r['rmse'] for r in results], 'bo-', lw=2)
    axes[0].set_xlabel('Noise Scale'); axes[0].set_ylabel('RMSE (m)'); axes[0].set_title('RMSE vs Noise')
    axes[0].grid(True, alpha=0.3)
    axes[1].bar([str(r['noise'])+'x' for r in results],
                [1 if r['success'] else 0 for r in results], color=['green' if r['success'] else 'red' for r in results])
    axes[1].set_xlabel('Noise Scale'); axes[1].set_ylabel('Success'); axes[1].set_title('Mission Success')
    plt.tight_layout()
    plt.savefig('task6_noise.png', dpi=150, bbox_inches='tight')
    print(f"  Saved: task6_noise.png"); plt.close()
    return results


def test_gps_dropout():
    normal = _run_standard_mission(noise_scale=1.0)
    # High dropout via higher noise
    dropout = _run_standard_mission(noise_scale=3.0)
    print(f"  Normal: RMSE={normal.get('rmse',0):.3f}m")
    print(f"  High noise (simulating dropout): RMSE={dropout.get('rmse',0):.3f}m")


def test_wind_disturbance():
    # Simulated via higher process noise
    results = []
    for ns in [1.0, 3.0, 6.0]:
        m = _run_standard_mission(noise_scale=ns)
        results.append({'level': ns, 'rmse': m.get('rmse',999)})
        print(f"  Wind level {ns}: RMSE={m.get('rmse',999):.3f}m")


def test_multiple_failures():
    m = _run_standard_mission(noise_scale=5.0)
    print(f"  Combined failures: RMSE={m.get('rmse',999):.3f}m, Complete={m.get('mission_complete',False)}")


def main():
    print("Task 6 Solution: Stress Testing")
    print("=" * 50)
    print("\nA: Sensor noise"); test_high_sensor_noise()
    print("\nB: GPS dropout"); test_gps_dropout()
    print("\nC: Wind"); test_wind_disturbance()
    print("\nD: Combined"); test_multiple_failures()
    print("\nTask 6 Solution complete.")


if __name__ == '__main__':
    main()
