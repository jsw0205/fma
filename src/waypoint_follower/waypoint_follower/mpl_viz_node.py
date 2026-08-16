import math
import threading

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker


class MplVizNode(Node):
    """Plain-matplotlib alternative to RViz: reads the same
    waypoint_follower/markers topic and redraws a 2D top-down view."""

    def __init__(self):
        super().__init__("mpl_viz_node")
        self._lock = threading.Lock()
        self.path_xy = []
        self.waypoints_xy = []
        self.target_xy = None
        self.vehicle_xy = None
        self.vehicle_yaw = None
        self.steer_text = ""

        self.create_subscription(
            MarkerArray, "waypoint_follower/markers", self.on_markers, 10
        )

    def on_markers(self, msg):
        with self._lock:
            for m in msg.markers:
                if m.id == 0:  # path LINE_STRIP
                    self.path_xy = [(p.x, p.y) for p in m.points]
                elif m.id == 1:  # waypoints SPHERE_LIST
                    self.waypoints_xy = [(p.x, p.y) for p in m.points]
                elif m.id == 2:  # target SPHERE
                    self.target_xy = (m.pose.position.x, m.pose.position.y)
                elif m.id == 3:  # vehicle ARROW or SPHERE
                    self.vehicle_xy = (m.pose.position.x, m.pose.position.y)
                    if m.type == Marker.ARROW:
                        q = m.pose.orientation
                        self.vehicle_yaw = 2.0 * math.atan2(q.z, q.w)
                    else:
                        self.vehicle_yaw = None
                elif m.id == 5:  # "steer=..." TEXT_VIEW_FACING
                    self.steer_text = m.text


def spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = MplVizNode()
    t = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    t.start()

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")

    (path_line,) = ax.plot([], [], "b-", linewidth=2, label="path")
    (waypoints_pts,) = ax.plot([], [], "yo", markersize=8, label="waypoints")
    (target_pt,) = ax.plot([], [], "go", markersize=12, label="target")
    (vehicle_pt,) = ax.plot([], [], "ro", markersize=10, label="vehicle")
    heading_arrow = ax.annotate(
        "", xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle="->", color="red", lw=2),
    )
    title = ax.set_title("")
    ax.legend(loc="upper right")

    def update(_frame):
        with node._lock:
            path_xy = list(node.path_xy)
            waypoints_xy = list(node.waypoints_xy)
            target_xy = node.target_xy
            vehicle_xy = node.vehicle_xy
            vehicle_yaw = node.vehicle_yaw
            steer_text = node.steer_text

        if path_xy:
            xs, ys = zip(*path_xy)
            path_line.set_data(xs, ys)
        if waypoints_xy:
            xs, ys = zip(*waypoints_xy)
            waypoints_pts.set_data(xs, ys)
        if target_xy:
            target_pt.set_data([target_xy[0]], [target_xy[1]])
        if vehicle_xy:
            vehicle_pt.set_data([vehicle_xy[0]], [vehicle_xy[1]])
            if vehicle_yaw is not None:
                dx = 1.5 * math.cos(vehicle_yaw)
                dy = 1.5 * math.sin(vehicle_yaw)
                heading_arrow.xy = (vehicle_xy[0] + dx, vehicle_xy[1] + dy)
                heading_arrow.set_position(vehicle_xy)
            else:
                heading_arrow.set_position((0, 0))
                heading_arrow.xy = (0, 0)

        title.set_text(steer_text)

        all_xy = path_xy + waypoints_xy
        if vehicle_xy:
            all_xy = all_xy + [vehicle_xy]
        if all_xy:
            xs, ys = zip(*all_xy)
            pad = 5.0
            ax.set_xlim(min(xs) - pad, max(xs) + pad)
            ax.set_ylim(min(ys) - pad, max(ys) + pad)

        return path_line, waypoints_pts, target_pt, vehicle_pt, heading_arrow, title

    from matplotlib.animation import FuncAnimation
    _anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
