#!/usr/bin/env python3
"""2D robot simulator with raycasting in a rectangular room with circular obstacles.
Publishes sensor_msgs/LaserScan on /scan.

Entry points
------------
main()      – run the interactive pygame simulator (default: `python robot_sim.py`)
build_map() – generate and return/save a ground-truth occupancy grid
              (run with: `python robot_sim.py --map`)

Occupancy grid values
---------------------
  1.0  occupied  (room wall inner face, obstacle circumference ring)
  0.5  unsure    (obstacle interior, region outside/behind room walls)
  0.0  free      (open navigable space inside the room)
"""

import pygame
import math
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped

# ── Config ────────────────────────────────────────────────────────────────────
WIDTH, HEIGHT = 800, 600
ROOM_MARGIN = 40
FPS = 60
NUM_RAYS = 100
RAY_MAX_LEN = 500
ROBOT_RADIUS = 12
ROBOT_SPEED = 1
ROBOT_TURN_SPEED = 1        # degrees per frame
CMD_VEL_TIMEOUT_SEC = 0.5
WALL_THICKNESS = 3          # pixels; matches pygame.draw.rect(..., 3)
PX_PER_METRE = 100.0        # 100 px = 1 m

# Colors
BG_COLOR       = (30,  30,  30)
WALL_COLOR     = (200, 200, 200)
OBSTACLE_COLOR = (100, 60,  60)
OBSTACLE_OUTLINE = (180, 100, 100)
ROBOT_COLOR    = (60,  180, 80)
ROBOT_DIR_COLOR = (200, 255, 200)
RAY_COLOR      = (255, 255, 80, 120)
HIT_COLOR      = (255, 60,  60)

# Room bounds (pixel coordinates)
ROOM_LEFT   = ROOM_MARGIN
ROOM_RIGHT  = WIDTH  - ROOM_MARGIN
ROOM_TOP    = ROOM_MARGIN
ROOM_BOTTOM = HEIGHT - ROOM_MARGIN

# Obstacles: (cx, cy, radius) in pixels
OBSTACLES = [
    (250, 200, 40),
    (550, 150, 30),
    (400, 400, 50),
    (150, 450, 35),
    (650, 350, 45),
    (350, 250, 25),
    (600, 480, 30),
]


# ── Geometry helpers ──────────────────────────────────────────────────────────

def ray_circle_intersect(ox, oy, dx, dy, cx, cy, cr):
    """Return distance from (ox,oy) along (dx,dy) to circle (cx,cy,cr), or None."""
    fx, fy = ox - cx, oy - cy
    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - cr * cr
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    disc_sqrt = math.sqrt(disc)
    t1 = (-b - disc_sqrt) / (2 * a)
    t2 = (-b + disc_sqrt) / (2 * a)
    if t1 >= 0:
        return t1
    if t2 >= 0:
        return t2
    return None


def ray_rect_intersect(ox, oy, dx, dy, rect):
    """Return distance from (ox,oy) along (dx,dy) to the inside walls of rect."""
    tmin = float('inf')
    if dx != 0:
        for wall_x in (rect.left, rect.right):
            t = (wall_x - ox) / dx
            if t > 0 and rect.top <= oy + t * dy <= rect.bottom:
                tmin = min(tmin, t)
    if dy != 0:
        for wall_y in (rect.top, rect.bottom):
            t = (wall_y - oy) / dy
            if t > 0 and rect.left <= ox + t * dx <= rect.right:
                tmin = min(tmin, t)
    return tmin if tmin < float('inf') else None


def cast_rays(rx, ry, angle_deg, num_rays, max_len):
    """Cast rays from robot position.

    Returns
    -------
    results   : list of (end_x, end_y, hit_bool)
    distances : list of float distances in *pixels*  (nan = no hit)
    angles    : list of ray angles in radians (world frame)
    """
    fov = math.radians(360)
    start_angle = math.radians(angle_deg) - fov / 2
    step = fov / num_rays
    results, distances, angles = [], [], []
    for i in range(num_rays):
        a = start_angle + i * step
        dx, dy = math.cos(a), math.sin(a)
        closest = max_len
        t = ray_rect_intersect(rx, ry, dx, dy,
                               pygame.Rect(ROOM_LEFT, ROOM_TOP,
                                           ROOM_RIGHT - ROOM_LEFT,
                                           ROOM_BOTTOM - ROOM_TOP))
        if t is not None and t < closest:
            closest = t
        for cx, cy, cr in OBSTACLES:
            t = ray_circle_intersect(rx, ry, dx, dy, cx, cy, cr)
            if t is not None and t < closest:
                closest = t
        hit = closest < max_len
        results.append((rx + dx * closest, ry + dy * closest, hit))
        distances.append(closest if hit else float('nan'))
        angles.append(a)
    return results, distances, angles


def clamp_to_room(x, y, radius):
    x = max(ROOM_LEFT + radius,  min(ROOM_RIGHT  - radius, x))
    y = max(ROOM_TOP  + radius,  min(ROOM_BOTTOM - radius, y))
    return x, y


def collides_obstacle(x, y, robot_r):
    return any(math.hypot(x - cx, y - cy) < robot_r + cr
               for cx, cy, cr in OBSTACLES)


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class SimNode(Node):
    """Minimal ROS 2 node: publishes LaserScan on /scan and Pose on /real_pose."""

    def __init__(self):
        super().__init__('robot_sim_scan')
        self.declare_parameter('disable_publishers', False)
        self.disable_publishers = (
            self.get_parameter('disable_publishers')
                .get_parameter_value().bool_value
        )
        self.scan_pub = self.create_publisher(LaserScan,   '/scan',      10)
        self.pose_pub = self.create_publisher(PoseStamped, '/real_pose', 10)
        self.linear_x  = 0.0
        self.angular_z = 0.0
        self.last_msg_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self._on_cmd_vel, 10)
        self.get_logger().info('Publishing LaserScan on /scan')
        self.get_logger().info('Publishing Pose on /real_pose')
        self.get_logger().info('Listening on /cmd_vel for robot motion input')

    def publish_scan(self, distances_px, angles_rad):
        if not distances_px:
            return
        heading  = (angles_rad[0] + angles_rad[-1]) / 2.0
        rh_angles = [-(a - heading) for a in angles_rad]
        if rh_angles[0] > rh_angles[-1]:
            rh_angles    = list(reversed(rh_angles))
            distances_px = list(reversed(distances_px))
        num = len(rh_angles)
        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min       = float(rh_angles[0])
        msg.angle_max       = float(rh_angles[-1])
        msg.angle_increment = float((rh_angles[-1] - rh_angles[0]) / max(num - 1, 1))
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / FPS
        msg.range_min       = 0.0
        msg.range_max       = float(RAY_MAX_LEN / PX_PER_METRE)
        msg.ranges          = [float('inf') if math.isnan(d) else d / PX_PER_METRE
                               for d in distances_px]
        msg.intensities     = []
        if not self.disable_publishers:
            self.scan_pub.publish(msg)

    def publish_pose(self, x_m, y_m, theta_rad):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position   = Point(x=x_m, y=y_m, z=0.0)
        msg.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(theta_rad / 2.0),
            w=math.cos(theta_rad / 2.0),
        )
        if not self.disable_publishers:
            self.pose_pub.publish(msg)

    def _on_cmd_vel(self, msg: TwistStamped):
        self.linear_x      = float(msg.twist.linear.x)
        self.angular_z     = float(msg.twist.angular.z)
        self.last_msg_time = self.get_clock().now()

    def get_cmd_vel(self):
        age_sec = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if age_sec > CMD_VEL_TIMEOUT_SEC:
            return 0.0, 0.0
        return self.linear_x, self.angular_z


# ── Occupancy-grid generation ─────────────────────────────────────────────────

def generate_occupancy_grid(
    width: int = WIDTH,
    height: int = HEIGHT,
    resolution: float = 1.0,
    wall_thickness: int = WALL_THICKNESS,
) -> np.ndarray:
    """Return a 2-D numpy array (rows=Y, cols=X) with occupancy values.

    Parameters
    ----------
    width, height   : canvas size in pixels
    resolution      : pixels per grid cell  (1 = full res, 2 = half res, …)
    wall_thickness  : pixel thickness of room boundary walls
    """
    cols = int(np.ceil(width  / resolution))
    rows = int(np.ceil(height / resolution))
    grid = np.zeros((rows, cols), dtype=np.float32)

    xs = (np.arange(cols) + 0.5) * resolution
    ys = (np.arange(rows) + 0.5) * resolution
    xv, yv = np.meshgrid(xs, ys)

    # Outside the room → unsure (unreachable by the robot)
    outside = (xv < ROOM_LEFT) | (xv > ROOM_RIGHT) | \
              (yv < ROOM_TOP)  | (yv > ROOM_BOTTOM)
    grid[outside] = 0.5

    # Room wall inner face → occupied
    wall_inner = (
        (xv >= ROOM_LEFT)   & (xv < ROOM_LEFT   + wall_thickness) |
        (xv <= ROOM_RIGHT)  & (xv > ROOM_RIGHT  - wall_thickness) |
        (yv >= ROOM_TOP)    & (yv < ROOM_TOP    + wall_thickness) |
        (yv <= ROOM_BOTTOM) & (yv > ROOM_BOTTOM - wall_thickness)
    )
    grid[wall_inner & ~outside] = 1.0

    # Obstacles: circumference ring → occupied, interior → unsure
    effective_band = max(wall_thickness, resolution)  # always ≥ 1 cell wide
    for cx, cy, cr in OBSTACLES:
        dist2 = (xv - cx) ** 2 + (yv - cy) ** 2
        obstacle_wall     = dist2 <= cr ** 2
        obstacle_interior = dist2 <  (cr - effective_band) ** 2
        grid[obstacle_interior]              = 0.5
        grid[obstacle_wall & ~obstacle_interior] = 1.0

    return np.flipud(grid)  # flip Y axis to match simulator frame


def grid_to_metric(grid: np.ndarray, resolution: float = 1.0):
    """Return (grid, info_dict) describing the metric frame.

    Frame matches the simulator's right-hand system:
      x → right,  y → up,  origin at spawn point (WIDTH/2, HEIGHT/2).
    """
    spawn_x, spawn_y = WIDTH / 2.0, HEIGHT / 2.0
    rows, cols = grid.shape
    info = {
        "resolution_m":  resolution / PX_PER_METRE,
        "width_cells":   cols,
        "height_cells":  rows,
        "origin_x_m":   -spawn_x / PX_PER_METRE,
        "origin_y_m":   -(HEIGHT - spawn_y) / PX_PER_METRE,
    }
    return grid, info


# ── Entry points ──────────────────────────────────────────────────────────────

def build_map(
    resolution: float = 1.0,
    output: str = "occupancy_grid.npy",
    visualize: bool = False,
) -> tuple[np.ndarray, dict]:
    """Generate the ground-truth occupancy grid and optionally save / visualise it.

    Parameters
    ----------
    resolution : pixels per grid cell (1 = full res, higher = coarser)
    output     : path to save the .npy file; pass None to skip saving
    visualize  : if True, display the grid with matplotlib

    Returns
    -------
    grid : np.ndarray  shape (rows, cols), values in {0.0, 0.5, 1.0}
    info : dict        metric frame metadata
    """
    grid = generate_occupancy_grid(resolution=resolution)
    grid, info = grid_to_metric(grid, resolution=resolution)

    if output:
        np.save(output, grid)
        print(f"Saved occupancy grid  shape={grid.shape}  →  '{output}'")

    print(f"Metric info: {info}")
    print(f"  free     (0.0): {(grid == 0.0).sum()} cells")
    print(f"  unsure   (0.5): {(grid == 0.5).sum()} cells")
    print(f"  occupied (1.0): {(grid == 1.0).sum()} cells")

    if visualize:
        import matplotlib.pyplot as plt
        import matplotlib.colors as mcolors

        cmap   = mcolors.ListedColormap(["white", "lightgray", "black"])
        bounds = [-0.25, 0.25, 0.75, 1.25]
        norm   = mcolors.BoundaryNorm(bounds, cmap.N)

        plt.figure(figsize=(10, 8))
        plt.imshow(grid, cmap=cmap, norm=norm, origin="lower")
        cbar = plt.colorbar(ticks=[0, 0.5, 1])
        cbar.set_ticklabels(["Free (0.0)", "Unsure (0.5)", "Occupied (1.0)"])
        plt.title("Ground Truth Occupancy Grid")
        plt.xlabel("X (pixels)")
        plt.ylabel("Y (pixels)")
        plt.tight_layout()
        plt.show()

    return grid, info


def main():
    """Run the interactive pygame simulator."""
    rclpy.init()
    sim_node = SimNode()

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("2D Robot Raycaster  (publishing /scan)")
    clock     = pygame.time.Clock()
    ray_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    ROOM_RECT   = pygame.Rect(ROOM_LEFT, ROOM_TOP,
                              ROOM_RIGHT - ROOM_LEFT,
                              ROOM_BOTTOM - ROOM_TOP)

    spawn_x, spawn_y = float(WIDTH // 2), float(HEIGHT // 2)
    robot_x, robot_y = spawn_x, spawn_y
    robot_angle = 0.0   # degrees, 0 = right

    running = True
    while running:
        dt = clock.get_time() / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        keys           = pygame.key.get_pressed()
        turn_left      = keys[pygame.K_LEFT]  or keys[pygame.K_a]
        turn_right     = keys[pygame.K_RIGHT] or keys[pygame.K_d]
        move_forward   = keys[pygame.K_UP]    or keys[pygame.K_w]
        move_backward  = keys[pygame.K_DOWN]  or keys[pygame.K_s]
        keyboard_active = turn_left or turn_right or move_forward or move_backward

        if turn_left:  robot_angle -= ROBOT_TURN_SPEED
        if turn_right: robot_angle += ROBOT_TURN_SPEED

        if not keyboard_active:
            linear_x, angular_z = sim_node.get_cmd_vel()
            robot_angle -= math.degrees(angular_z * dt)
        else:
            linear_x, angular_z = 0.0, 0.0

        dx = math.cos(math.radians(robot_angle))
        dy = math.sin(math.radians(robot_angle))
        nx, ny = robot_x, robot_y

        if move_forward:  nx += dx * ROBOT_SPEED; ny += dy * ROBOT_SPEED
        if move_backward: nx -= dx * ROBOT_SPEED; ny -= dy * ROBOT_SPEED
        if not keyboard_active:
            nx += dx * linear_x * PX_PER_METRE * dt
            ny += dy * linear_x * PX_PER_METRE * dt

        nx, ny = clamp_to_room(nx, ny, ROBOT_RADIUS)
        if not collides_obstacle(nx, ny, ROBOT_RADIUS):
            robot_x, robot_y = nx, ny

        # ── Draw ─────────────────────────────────────────────────────────────
        screen.fill(BG_COLOR)
        pygame.draw.rect(screen, WALL_COLOR, ROOM_RECT, 3)
        for cx, cy, cr in OBSTACLES:
            pygame.draw.circle(screen, OBSTACLE_COLOR,   (cx, cy), cr)
            pygame.draw.circle(screen, OBSTACLE_OUTLINE, (cx, cy), cr, 2)

        rays, distances, angles = cast_rays(
            robot_x, robot_y, robot_angle, NUM_RAYS, RAY_MAX_LEN)
        sim_node.publish_scan(distances, angles)

        pose_x     = (robot_x - spawn_x) / PX_PER_METRE
        pose_y     = -(robot_y - spawn_y) / PX_PER_METRE
        pose_theta = math.radians(-robot_angle)
        sim_node.publish_pose(pose_x, pose_y, pose_theta)

        rclpy.spin_once(sim_node, timeout_sec=0)

        ray_surface.fill((0, 0, 0, 0))
        for ex, ey, hit in rays:
            pygame.draw.line(ray_surface, (255, 255, 80, 60),
                             (robot_x, robot_y), (ex, ey), 1)
            color = HIT_COLOR if hit else (255, 255, 80)
            pygame.draw.circle(ray_surface, (*color, 200), (int(ex), int(ey)), 3)
        screen.blit(ray_surface, (0, 0))

        pygame.draw.circle(screen, ROBOT_COLOR,
                           (int(robot_x), int(robot_y)), ROBOT_RADIUS)
        pygame.draw.line(screen, ROBOT_DIR_COLOR,
                         (robot_x, robot_y),
                         (robot_x + dx * (ROBOT_RADIUS + 8),
                          robot_y + dy * (ROBOT_RADIUS + 8)), 3)

        font = pygame.font.SysFont(None, 22)
        screen.blit(
            font.render("WASD/Arrows to move  |  ESC to quit  |  /scan published",
                        True, (160, 160, 160)),
            (ROOM_LEFT + 4, HEIGHT - ROOM_MARGIN + 8))

        x_m      = (robot_x - spawn_x) / PX_PER_METRE
        y_m      = -(robot_y - spawn_y) / PX_PER_METRE
        theta_deg = (-robot_angle) % 360
        theta_rad = math.radians(theta_deg)
        for i, line in enumerate([f"x: {x_m:.2f} m",
                                   f"y: {y_m:.2f} m",
                                   f"θ: {theta_deg:.1f}°  ({theta_rad:.2f} rad)"]):
            screen.blit(font.render(line, True, (180, 220, 255)),
                        (ROOM_LEFT + 8, ROOM_TOP + 8 + i * 20))

        pygame.display.flip()
        clock.tick(FPS)

    sim_node.destroy_node()
    rclpy.shutdown()
    pygame.quit()
    sys.exit()


# ── Script entry ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="2D robot simulator / map builder.")
    parser.add_argument("--map",        action="store_true",
                        help="Build the ground-truth occupancy grid instead of running the sim.")
    parser.add_argument("--resolution", type=float, default=5.0,
                        help="(--map) Pixels per grid cell (default 1).")
    parser.add_argument("--output",     type=str, default="occupancy_grid.npy",
                        help="(--map) Output .npy file path.")
    parser.add_argument("--visualize",  action="store_true",
                        help="(--map) Display the grid with matplotlib.")
    args = parser.parse_args()

    if args.map:
        build_map(resolution=args.resolution,
                  output=args.output,
                  visualize=args.visualize)
    else:
        main()