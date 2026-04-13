import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.spatial.transform import Rotation

plt.rcParams['lines.linewidth'] = 2.5
plt.rcParams['font.size'] = 20


def compute_ate(est: np.ndarray, truth: np.ndarray) -> dict:
    """Compute ATE with optional SE2 alignment (like the standard SLAM benchmark).

    Parameters
    ----------
    est   : (N, 4) array of [t, x, y, yaw]
    truth : (N, 4) array of [t, x, y, yaw]

    Returns
    -------
    dict with per-timestep errors and summary statistics
    """
    est_xy   = est[:, 1:3]
    truth_xy = truth[:, 1:3]

    # ── Align trajectories (Umeyama / Horn method, 2D) ───────────────────────
    # This removes any global offset/rotation so ATE measures estimation
    # quality rather than reference frame mismatch.
    est_mean   = est_xy.mean(axis=0)
    truth_mean = truth_xy.mean(axis=0)
    est_c   = est_xy   - est_mean
    truth_c = truth_xy - truth_mean

    W = truth_c.T @ est_c
    U, _, Vt = np.linalg.svd(W)
    R_align = U @ Vt
    if np.linalg.det(R_align) < 0:       # handle reflection
        Vt[-1, :] *= -1
        R_align = U @ Vt
    t_align = truth_mean - R_align @ est_mean
    est_aligned = (R_align @ est_c.T).T + truth_mean

    # ── Per-timestep errors ───────────────────────────────────────────────────
    trans_errors = np.linalg.norm(est_aligned - truth_xy, axis=1)
    yaw_errors   = np.abs(np.arctan2(
        np.sin(est[:, 3] - truth[:, 3]),
        np.cos(est[:, 3] - truth[:, 3])
    ))

    return {
        'times':        est[:, 0],
        'trans_errors': trans_errors,
        'yaw_errors':   yaw_errors,
        'ate_rmse':     float(np.sqrt(np.mean(trans_errors ** 2))),
        'ate_mean':     float(np.mean(trans_errors)),
        'ate_max':      float(np.max(trans_errors)),
        'yaw_rmse':     float(np.sqrt(np.mean(yaw_errors ** 2))),
        'yaw_mean':     float(np.mean(yaw_errors)),
        'est_aligned':  est_aligned,
        'truth_xy':     truth_xy,
        'est_xy':       est_xy,
    }


def plot_ate(results: dict, output_dir: Path):
    t            = results['times']
    trans_errors = results['trans_errors']
    yaw_errors   = np.degrees(results['yaw_errors'])

    fig, axes = plt.subplots(1, 2, figsize=(16, 5), constrained_layout=True)
    # fig.suptitle('Offline ATE Analysis')

    # ── Col 1: Translation error over time ───────────────────────────────────
    ax = axes[0]
    ax.plot(t, trans_errors, 'b-', linewidth=1.0, alpha=0.6, label='Per-step error')
    ax.axhline(results['ate_mean'], color='red',    linestyle='--',
               linewidth=2.0, label=f'Mean: {results["ate_mean"]:.4f} m')
    ax.axhline(results['ate_rmse'], color='orange', linestyle='-.',
               linewidth=2.0, label=f'RMSE: {results["ate_rmse"]:.4f} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Translation error (m)')
    # ax.set_title('ATE Translation over Time')
    ax.legend(fontsize=15)

    # ── Col 2: Yaw error over time ────────────────────────────────────────────
    ax = axes[1]
    ax.plot(t, yaw_errors, 'purple', linewidth=1.0, alpha=0.6, label='Per-step error')
    ax.axhline(np.degrees(results['yaw_mean']), color='red',    linestyle='--',
               linewidth=2.0, label=f'Mean: {np.degrees(results["yaw_mean"]):.4f}°')
    ax.axhline(np.degrees(results['yaw_rmse']), color='orange', linestyle='-.',
               linewidth=2.0, label=f'RMSE: {np.degrees(results["yaw_rmse"]):.4f}°')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw error (deg)')
    # ax.set_title('ATE Yaw over Time')
    ax.legend(fontsize=15)

    plt.savefig(output_dir / 'ate_analysis.png', dpi=300)
    plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, default='trajectories',
                        help='Directory containing est_trajectory.npy and truth_trajectory.npy')
    args = parser.parse_args()

    output_dir = Path(args.dir)
    est   = np.load(output_dir / 'est_trajectory.npy')
    truth = np.load(output_dir / 'truth_trajectory.npy')

    results = compute_ate(est, truth)

    print('── ATE Results ──────────────────────────────')
    print(f'  Translation RMSE : {results["ate_rmse"]:.4f} m')
    print(f'  Translation mean : {results["ate_mean"]:.4f} m')
    print(f'  Translation max  : {results["ate_max"]:.4f} m')
    print(f'  Yaw RMSE         : {np.degrees(results["yaw_rmse"]):.4f} deg')
    print(f'  Yaw mean         : {np.degrees(results["yaw_mean"]):.4f} deg')
    print('─────────────────────────────────────────────')

    plot_ate(results, output_dir)


if __name__ == '__main__':
    main()