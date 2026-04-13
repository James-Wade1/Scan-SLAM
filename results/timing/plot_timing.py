import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

plt.rcParams['lines.linewidth'] = 2.5
plt.rcParams['font.size'] = 20

def load_run(directory: str) -> dict:
    d = Path(directory)
    return {
        'frontend_times':          np.load(d / 'frontend_times.npy'),
        'frontend_iterations':     np.load(d / 'frontend_iterations.npy'),
        'loop_closure_times':      np.load(d / 'loop_closure_times.npy'),
        'loop_closure_iterations': np.load(d / 'loop_closure_iterations.npy'),
        'backend_times':           np.load(d / 'backend_times.npy'),
        'backend_iterations':      np.load(d / 'backend_iterations.npy'),
        'backend_graph_sizes':     np.load(d / 'backend_graph_sizes.npy'),
    }


def plot_bar_summary(sparse: dict):
    """Bar chart comparing mean wall time and iterations across frontend operations."""
    fig, axes = plt.subplots(1, 2, figsize=(15, 5), constrained_layout=True)
    # fig.suptitle('Runtime Summary')

    labels = [
        'Frontend\n(Sequential)',
        'Frontend\n(Loop Closure)',
        'Backend\n(Sparse)',
    ]
    means_time = [
        sparse['frontend_times'].mean(),
        sparse['loop_closure_times'].mean() if len(sparse['loop_closure_times']) > 0 else 0.0,
        sparse['backend_times'].mean(),
    ]
    std_time = [
        sparse['frontend_times'].std(),
        sparse['loop_closure_times'].std() if len(sparse['loop_closure_times']) > 0 else 0.0,
        sparse['backend_times'].std(),
    ]
    means_iter = [
        sparse['frontend_iterations'].mean(),
        sparse['loop_closure_iterations'].mean() if len(sparse['loop_closure_iterations']) > 0 else 0.0,
        sparse['backend_iterations'].mean(),
    ]
    std_iter = [
        sparse['frontend_iterations'].std(),
        sparse['loop_closure_iterations'].std() if len(sparse['loop_closure_iterations']) > 0 else 0.0,
        sparse['backend_iterations'].std(),
    ]

    colors = ['steelblue', 'seagreen', 'orange']
    x = np.arange(len(labels))
    width = 0.5

    ax = axes[0]
    bars = ax.bar(x, means_time, width, yerr=std_time,
                  color=colors, capsize=5, edgecolor='black')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('Wall time (ms)')
    # ax.set_title('Mean Wall Time per Call')
    for bar, mean in zip(bars, means_time):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                f'{mean:.1f} ms', ha='center', va='bottom', fontsize=15)

    ax = axes[1]
    bars = ax.bar(x, means_iter, width, yerr=std_iter,
                  color=colors, capsize=5, edgecolor='black')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel('Iterations')
    # ax.set_title('Mean Iteration Count per Call')
    for bar, mean in zip(bars, means_iter):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.1,
                f'{mean:.1f}', ha='center', va='bottom', fontsize=15)

    plt.savefig('runtime_summary.png', dpi=300)
    plt.show()


def plot_scaling(sparse: dict, dense: dict):
    """Scatter + trendline showing how sparse vs dense solver scales with graph size."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    # fig.suptitle('Sparse vs Dense Solver Scaling')

    for ax, key, ylabel, title in zip(
        axes,
        ['backend_times', 'backend_iterations'],
        ['Wall time (ms)', 'Iterations'],
        ['Wall Time vs Graph Size', 'Iterations vs Graph Size'],
    ):
        for data, color, label in [
            (sparse, 'steelblue', 'Sparse'),
            (dense,  'tomato',    'Dense'),
        ]:
            x = np.array(data['backend_graph_sizes'])
            y = np.array(data[key])

            ax.scatter(x, y, color=color, alpha=0.4, s=20, label=f'{label} (samples)')

            # Fit polynomial trendline — degree 2 captures quadratic growth
            if len(x) > 3:
                coeffs = np.polyfit(x, y, 2)
                x_fit  = np.linspace(x.min(), x.max(), 200)
                y_fit  = np.polyval(coeffs, x_fit)
                ax.plot(x_fit, y_fit, color=color, linewidth=2,
                        linestyle='--', label=f'{label} (trend)')

        ax.set_xlabel('Graph size (nodes)')
        ax.set_ylabel(ylabel)
        # ax.set_title(title)
        ax.legend(fontsize=15)

    plt.savefig('solver_scaling.png', dpi=300)
    plt.show()


def print_summary(sparse: dict, dense: dict):
    print('── Timing Summary ───────────────────────────────────────────────')
    print(f'  Frontend sequential  : {sparse["frontend_times"].mean():.2f} ± {sparse["frontend_times"].std():.2f} ms')
    print(f'  Frontend iterations  : {sparse["frontend_iterations"].mean():.1f} ± {sparse["frontend_iterations"].std():.1f}')
    if len(sparse['loop_closure_times']) > 0:
        print(f'  Loop closure time    : {sparse["loop_closure_times"].mean():.2f} ± {sparse["loop_closure_times"].std():.2f} ms')
        print(f'  Loop closure iters   : {sparse["loop_closure_iterations"].mean():.1f} ± {sparse["loop_closure_iterations"].std():.1f}')
    print(f'  Backend sparse time  : {sparse["backend_times"].mean():.2f} ± {sparse["backend_times"].std():.2f} ms')
    print(f'  Backend sparse iters : {sparse["backend_iterations"].mean():.1f} ± {sparse["backend_iterations"].std():.1f}')
    print(f'  Backend dense time   : {dense["backend_times"].mean():.2f} ± {dense["backend_times"].std():.2f} ms')
    print(f'  Backend dense iters  : {dense["backend_iterations"].mean():.1f} ± {dense["backend_iterations"].std():.1f}')
    print('─────────────────────────────────────────────────────────────────')


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--sparse', type=str, default='timing_results_sparse',
                        help='Directory with sparse solver timing data')
    parser.add_argument('--dense',  type=str, default='timing_results_dense',
                        help='Directory with dense solver timing data')
    args = parser.parse_args()

    sparse = load_run(args.sparse)
    dense  = load_run(args.dense)

    print_summary(sparse, dense)
    plot_bar_summary(sparse)
    plot_scaling(sparse, dense)


if __name__ == '__main__':
    main()