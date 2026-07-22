#!/usr/bin/env python3
import os, sys, signal, datetime
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

TOPICS = {
    "/ground_truth/odom":          ("Ground Truth",          "#00d4ff", 2.5),
    "/diff_drive_controller/odom": ("Wheel Odom (raw)",       "#ff8c00", 1.8),
    "/odometry/filtered":          ("EKF Fused (IMU+Wheel)",  "#00ff66", 1.8),
}
SAVE_DIR = os.path.expanduser("~/ros2-drone-gnss-ins-sim")
MIN_DIST = 0.02


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")
        self._paths = {t: {"x": [], "y": []} for t in TOPICS}
        self._plot_done = False
        for topic in TOPICS:
            self.create_subscription(Odometry, topic,
                lambda msg, t=topic: self._cb(msg, t), 10)
        self.get_logger().info("Trajectory recorder running - plot saved on shutdown.")

    def _cb(self, msg, topic):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        buf = self._paths[topic]
        if buf["x"]:
            dx, dy = x - buf["x"][-1], y - buf["y"][-1]
            if dx*dx + dy*dy < MIN_DIST**2:
                return
        buf["x"].append(x)
        buf["y"].append(y)

    def generate_plot(self):
        if self._plot_done:
            return
        self._plot_done = True
        total = sum(len(v["x"]) for v in self._paths.values())
        if total < 10:
            print("[trajectory_recorder] Too few points - skipping plot.")
            return
        print("[trajectory_recorder] Generating plot...")
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_facecolor("#111111")
        fig.patch.set_facecolor("#1e1e1e")

        line_handles = []
        for topic, (label, color, lw) in TOPICS.items():
            xs, ys = self._paths[topic]["x"], self._paths[topic]["y"]
            if len(xs) < 2:
                continue
            n = len(xs)
            line, = ax.plot(xs, ys, color=color, linewidth=lw, alpha=0.95, zorder=3)
            line_handles.append((line, f"{label}  ({n} pts)"))
            # start marker
            ax.plot(xs[0],  ys[0],  "o", color=color, markersize=11,
                    markeredgecolor="white", markeredgewidth=0.7, zorder=6)
            # end marker
            ax.plot(xs[-1], ys[-1], "s", color=color, markersize=11,
                    markeredgecolor="white", markeredgewidth=0.7, zorder=6)

        ax.set_xlabel("X  [m]", color="#dddddd", fontsize=13, labelpad=8)
        ax.set_ylabel("Y  [m]", color="#dddddd", fontsize=13, labelpad=8)
        ax.set_title(
            "Robot Trajectory Comparison\n"
            "Sensor Fusion with robot_localization EKF  "
            "(Ground Truth  |  Wheel Odom  |  EKF Fused)",
            color="white", fontsize=14, fontweight="bold", pad=18)
        ax.tick_params(colors="#cccccc", which="both", labelsize=11)
        for spine in ax.spines.values():
            spine.set_color("#444444")
        ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
        ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
        ax.grid(True, color="#2a2a2a", linewidth=0.8, zorder=0)
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(
            [h for h, _ in line_handles],
            [l for _, l in line_handles],
            facecolor="#222", edgecolor="#555", labelcolor="white",
            fontsize=11, loc="best", framealpha=0.88)
        ax.annotate("circle = start     square = end",
                    xy=(0.02, 0.02), xycoords="axes fraction",
                    color="#777777", fontsize=9)

        os.makedirs(SAVE_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out = os.path.join(SAVE_DIR, f"trajectory_{ts}.png")
        fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)
        print(f"[trajectory_recorder] Plot saved  ->  {out}")
        try:
            os.system(f'xdg-open "{out}" 2>/dev/null &')
        except Exception:
            pass
        fig.patch.set_facecolor("#1e1e1e")
        line_handles = []
        for topic, (label, color, lw) in TOPICS.items():
            xs, ys = self._paths[topic]["x"], self._paths[topic]["y"]
            if len(xs) < 2:
                continue
            line, = ax.plot(xs, ys, color=color, linewidth=lw, alpha=0.95, zorder=3)
            line_handles.append((line, label))
            ax.plot(xs[0],  ys[0],  "o", color=color, markersize=10,
                    markeredgecolor="white", markeredgewidth=0.6, zorder=6)
            ax.plot(xs[-1], ys[-1], "s", color=color, markersize=10,
                    markeredgecolor="white", markeredgewidth=0.6, zorder=6)
        ax.set_xlabel("X  [m]", color="#dddddd", fontsize=13, labelpad=8)
        ax.set_ylabel("Y  [m]", color="#dddddd", fontsize=13, labelpad=8)
        ax.set_title(
            "Robot Trajectory Comparison\nGround Truth  .  Wheel Odom (raw)  .  EKF Fused",
            color="white", fontsize=14, fontweight="bold", pad=16)
        ax.tick_params(colors="#cccccc", which="both", labelsize=11)
        for spine in ax.spines.values():
            spine.set_color("#444444")
        ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
        ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
        ax.grid(True, color="#2a2a2a", linewidth=0.8, zorder=0)
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend([h for h, _ in line_handles], [l for _, l in line_handles],
                  facecolor="#222", edgecolor="#555", labelcolor="white",
                  fontsize=12, loc="best", framealpha=0.88)
        ax.annotate("circle = start     square = end",
                    xy=(0.02, 0.02), xycoords="axes fraction",
                    color="#888888", fontsize=9)
        os.makedirs(SAVE_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out = os.path.join(SAVE_DIR, f"trajectory_{ts}.png")
        fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)
        print(f"[trajectory_recorder] Plot saved  ->  {out}")
        try:
            os.system(f'xdg-open "{out}" 2>/dev/null &')
        except Exception:
            pass


_node_ref = None

def _signal_handler(signum, frame):
    sys.exit(0)

def main():
    global _node_ref
    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGINT,  _signal_handler)
    rclpy.init()
    _node_ref = TrajectoryRecorder()
    try:
        rclpy.spin(_node_ref)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:
        print(f"[trajectory_recorder] Error: {e}")
    finally:
        _node_ref.generate_plot()
        try:
            _node_ref.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
