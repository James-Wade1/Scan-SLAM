import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.ndimage import binary_dilation, generate_binary_structure, iterate_structure

plt.rcParams['font.size'] = 20


def make_error_overlay(truth, estimate, inflation_radius: int = 0):
    """Return an RGBA image and accuracy score.

    Parameters
    ----------
    inflation_radius : int
        If > 0, a truth cell is considered correct if the estimate has a
        matching value anywhere within this many cells (Euclidean radius).

    Rules
    -----
    - Only cells where truth == 1.0 or truth == 0.0 are scored.
    - Truth unsure (0.5) cells are skipped (gray).
    - Unsure in estimate counts as wrong unless inflation finds a match nearby.

    Green  = correct (within inflation radius)
    Red    = wrong
    Gray   = truth is unsure (not scored)
    """
    h, w = truth.shape
    rgba = np.zeros((h, w, 4), dtype=np.float32)

    truth_unsure   = truth == 0.5
    truth_definite = ~truth_unsure

    if inflation_radius > 0:
        # Build a circular structuring element of the given radius
        y, x = np.ogrid[-inflation_radius:inflation_radius+1,
                        -inflation_radius:inflation_radius+1]
        struct = (x**2 + y**2) <= inflation_radius**2

        # For each definite truth value, dilate its mask in the estimate
        # and check if any neighbour within radius matches
        est_occupied = estimate == 1.0
        est_free     = estimate == 0.0

        # Dilate the estimate masks so nearby matches count
        est_occupied_dilated = binary_dilation(est_occupied, structure=struct)
        est_free_dilated     = binary_dilation(est_free,     structure=struct)

        match = (
            truth_definite &
            (
                ((truth == 1.0) & est_occupied_dilated) |
                ((truth == 0.0) & est_free_dilated)
            )
        )
    else:
        match = truth_definite & (truth == estimate)

    mismatch = truth_definite & ~match

    rgba[truth_unsure] = [0.6, 0.6, 0.6, 0.6]  # gray,  alpha=0.6
    rgba[match]        = [0.0, 0.8, 0.0, 0.7]  # green, alpha=0.7
    rgba[mismatch]     = [0.9, 0.0, 0.0, 0.9]  # red,   alpha=0.9

    n_scored = truth_definite.sum()
    accuracy = match.sum() / n_scored * 100.0 if n_scored > 0 else float('nan')

    return rgba, accuracy


def main():
    grid_truth    = np.load('src/scan_slam_results/occupancy_grid/occupancy_grid_truth.npy')
    grid_estimate = np.load('src/scan_slam_results/occupancy_grid/occupancy_grid_estimate.npy')

    grid_estimate = 1 - 1 / (1 + np.exp(grid_estimate))
    display = np.full_like(grid_estimate, 0.5)
    display[grid_estimate < 0.01] = 0.0
    display[grid_estimate > 0.6]  = 1.0
    grid_estimate = display

    overlay_truth,    accuracy_truth    = make_error_overlay(grid_truth, grid_estimate, 1)
    overlay_estimate, accuracy_estimate = make_error_overlay(grid_truth, grid_estimate, 1)

    fig, axes = plt.subplots(1, 2, figsize=(20, 10), constrained_layout=True)
    plt.subplots_adjust(bottom=0.3, wspace=0.35)

    for ax, grid, overlay, accuracy, title in zip(
        axes,
        [grid_truth, grid_estimate],
        [overlay_truth, overlay_estimate],
        [accuracy_truth, accuracy_estimate],
        ['Ground Truth', 'Estimated'],
    ):
        ax.imshow(grid, origin='lower', cmap='gray_r', vmin=0.0, vmax=1.0)
        ax.imshow(overlay, origin='lower')
        ax.set_xlabel('X (cells)')
        ax.set_ylabel('Y (cells)')
        ax.set_aspect('equal')
        if title == 'Estimated':
            title += f'\n(Accuracy: {accuracy:.1f}%)'
        ax.text(0.2, 0.87, title, transform=ax.transAxes, fontsize=20,
        ha='center', va='center')
        ax.grid()

    legend_patches = [
        mpatches.Patch(color=(0.0, 0.8, 0.0), label='Correct'),
        mpatches.Patch(color=(0.9, 0.0, 0.0), label='Wrong'),
        mpatches.Patch(color=(0.6, 0.6, 0.6), label='Unsure (not scored)'),
    ]
    fig.legend(handles=legend_patches, loc='lower center',
            ncol=3, bbox_to_anchor=(0.5, 0.01))

    fig.savefig('occupancy_grid_analysis.png', dpi=300)
    plt.show()


if __name__ == '__main__':
    main()