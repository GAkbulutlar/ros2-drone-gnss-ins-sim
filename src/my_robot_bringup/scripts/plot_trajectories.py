#!/usr/bin/env python3
"""
plot_trajectories.py

Records three odometry trajectories while the simulation runs and
automatically generates a comparison PNG whenever the launch is stopped.

Topics recorded:
  /ground_truth/odom          -> cyan  (Gazebo perfect pose)
  /diff_drive_controller/odom -> orange (raw wheel encoder dead-reckoning)
  /odometry/filtered          -> green  (EKF fused: wheels + IMU)

Output:
  ~/ros2-drone-gnss-ins-sim/trajectory_YYYYMMDD_HHMMSS.png
  (opened automatically with xdg-open if a display is available)
"""

import os
import datetime
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')          # works headless (no display required for saving)
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


# ── configuration ──────────────────────────────────────────────────────────────
TOPICS = {
    '/ground_truth/odom':          ('Ground Truth',        '#00d4ff', 2.5),
    '/diff_drive_controller/odom': ('Wheel Odom (raw)',    '#ff8c00', 1.8),
    '/odometry/filtered':          ('EKF Fused (IMU+Wheel)', '#00ff66', 1.8),
}
SAVE_DIR  = os.path.expanduser('~/ros2-drone-gnss-ins-sim')
MIN_DIST  = 0.02    # [m] minimum step before a new point is appended
# ───────────────────────────────────────────────────────────────────────────────


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self._paths: dict[str, dict] = {t: {'x': [], 'y': []} for t in TOPICS}

        for topic in TOPICS:
            self.create_subscription(
                Odometry, topic,
                lambda msg, t=topic: self._cb(msg, t),
                10,
            )

        # Register plot generation on any clean shutdown (SIGINT / SIGTERM / Ctrl-C)
        rclpy.get_default_context().on_shutdown(self._generate_plot)

        self.get_logger().info(
            'Trajectory recorder running — '
            'plot will be saved automatically on shutdown.'
        )

    # ── callback ───────────────────────────────────────────────────────────────
    def _cb(self, msg: Odometry, topic: str) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        buf = self._paths[topic]
        if buf['x']:
            dx, dy = x - buf['x'][-1], y - buf['y'][-1]
            if dx * dx + dy * dy < MIN_DIST ** 2:
                return
        buf['x'].append(x)
        buf['y'].append(y)

    # ── plot ───────────────────────────────────────────────────────────────────
    def _generate_plot(self) -> None:
        total = sum(len(v['x']) for v in self._paths.values())
        if total < 10:
            self.get_logger().warn('Too few points recorded — skipping plot.')
            return

        self.get_logger().info('Generating trajectory comparison plot …')

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_facecolor('#111111')
        fig.patch.set_facecolor('#1e1e1e')

        line_handles = []
        for topic, (label, color, lw) in TOPICS.items():
            xs = self._paths[topic]['x']
            ys = self._paths[topic]['y']
            if len(xs) < 2:
                continue

            line, = ax.plot(xs, ys, color=color, linewidth=lw,
                            alpha=0.95, zorder=3)
            line_handles.append((line, label))

            # Mark start (circle) and end (square)
            ax.plot(xs[0],  ys[0],  'o', color=color,
                    markersize=10, markeredgecolor='white',
                    markeredgewidth=0.6, zorder=6)
            ax.plot(xs[-1], ys[-1], 's', color=color,
                    markersize=10, markeredgecolor='white',
                    markeredgewidth=0.6, zorder=6)

        # ── formatting ─────────────────────────────────────────────────────
        ax.set_xlabel('X  [m]', color='#dddddd', fontsize=13, labelpad=8)
        ax.set_ylabel('Y  [m]', color='#dddddd', fontsize=13, labelpad=8)
        ax.set_title(
            'Robot Trajectory Comparison\n'
            'Ground Truth   ·   Wheel Odometry (raw)   ·   EKF Fused (Wheel + IMU)',
            color='white', fontsize=14, fontweight='bold', pad=16,
        )
        ax.tick_params(colors='#cccccc', which='both', labelsize=11)
        for spine in ax.spines.values():
            spine.set_color('#444444')

        ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
        ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
        ax.grid(True, color='#2a2a2a', linewidth=0.8, zorder=0)
        ax.set_aspect('equal', adjustable='datalim')

        # Legend (one entry per line)
        ax.legend(
            [h for h, _ in line_handles],
            [l for _, l in line_handles],
            facecolor='#222', edgecolor='#555',
            labelcolor='white', fontsize=12,
            loc='best', framealpha=0.88,
        )
        ax.annotate('●  start     ■  end',
                    xy=(0.02, 0.02), xycoords='axes fraction',
                    color='#888888', fontsize=9)

        # ── save ───────────────────────────────────────────────────────────
        os.makedirs(SAVE_DIR, exist_ok=True)
        ts      = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(SAVE_DIR, f'trajectory_{ts}.png')
        fig.savefig(out_path, dpi=150, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        plt.close(fig)

        print(f'\n[trajectory_recorder]  Plot saved  →  {out_path}\n')

        # Open the image if a display is available
        try:
            os.system(f'xdg-open "{out_path}" 2>/dev/null &')
        except Exception:
            pass


# ── entry point ────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
