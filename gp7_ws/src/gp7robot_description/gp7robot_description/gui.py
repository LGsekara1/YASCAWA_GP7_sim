"""Small control and monitoring GUI for the GP7 pick-and-place simulation.

Start Gazebo first (``run_demo:=false`` so the GUI controls the demo), then:

    ros2 run gp7robot_description gp7_gui

The GUI shows live joint angles and velocities, the gripper path seen from
above, and the gripper height and speed. Its buttons run the pick-and-place
demo, stop it, and reset the world (which also moves Box B to the x/y typed
in the panel, so new start positions can be tried without editing the world).
"Place Box B at random" picks a random position the planner can reach (see
``workspace.py``), moves Box B there and homes the arm; "Start task" then runs
the pick-and-place from that position.
"""

from __future__ import annotations

import collections
import queue
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

import matplotlib
import numpy as np
import rclpy
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Rectangle
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .kinematics import (
    JOINT_NAMES,
    UPRIGHT_REFERENCE_CONFIGURATION,
    forward_kinematics,
)
from .pick_place import BOX_B_SIZE, GRIPPER_LINK_OFFSET
from .workspace import random_box_b

matplotlib.rcParams.update({"font.size": 8, "axes.grid": True, "grid.alpha": 0.3})

HISTORY_SECONDS = 30.0
RECORD_PERIOD = 0.02  # store at most 50 samples per second
REFRESH_MS = 100
JOINT_COLOURS = ("#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b")
BOX_A_CENTRE = (0.50, 0.50)
BOX_A_SIZE = 0.30
DEFAULT_BOX_B = (0.35, 0.25)

# Gripper point in the Link6 frame (the mount used by the Gazebo launch file).
_UPRIGHT_ROTATION = forward_kinematics(UPRIGHT_REFERENCE_CONFIGURATION)[:3, :3]
GRIPPER_IN_LINK6 = _UPRIGHT_ROTATION.T @ np.array([0.0, 0.0, -GRIPPER_LINK_OFFSET])


def gripper_position(joints: np.ndarray) -> np.ndarray:
    link6 = forward_kinematics(joints)
    return link6[:3, 3] + link6[:3, :3] @ GRIPPER_IN_LINK6


class JointMonitor(Node):
    """Collects /joint_states into short time-series buffers."""

    def __init__(self) -> None:
        super().__init__("gp7_gui")
        self.times: collections.deque[float] = collections.deque()
        self.joints: collections.deque[np.ndarray] = collections.deque()
        self.velocities: collections.deque[np.ndarray] = collections.deque()
        self.tool: collections.deque[np.ndarray] = collections.deque()
        self.tool_speed: collections.deque[float] = collections.deque()
        self.message_count = 0
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)

    def _on_joint_state(self, message: JointState) -> None:
        self.message_count += 1
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        if self.times and stamp - self.times[-1] < RECORD_PERIOD:
            if stamp >= self.times[-1]:
                return  # downsample (and drop duplicates from extra bridges)
            self.clear()  # simulation time jumped back: Gazebo was restarted
        names = [name.replace("/", "::").split("::")[-1] for name in message.name]
        index = {name: i for i, name in enumerate(names)}
        if not all(name in index for name in JOINT_NAMES):
            return
        q = np.array([message.position[index[n]] for n in JOINT_NAMES])
        tool = gripper_position(q)
        if self.times:
            dt = stamp - self.times[-1]
            velocity = (q - self.joints[-1]) / dt
            speed = float(np.linalg.norm(tool - self.tool[-1]) / dt)
        else:
            velocity, speed = np.zeros(6), 0.0
        self.times.append(stamp)
        self.joints.append(q)
        self.velocities.append(velocity)
        self.tool.append(tool)
        self.tool_speed.append(speed)
        while self.times and stamp - self.times[0] > HISTORY_SECONDS:
            for buffer in (self.times, self.joints, self.velocities, self.tool, self.tool_speed):
                buffer.popleft()

    def clear(self) -> None:
        for buffer in (self.times, self.joints, self.velocities, self.tool, self.tool_speed):
            buffer.clear()


class ProcessRunner:
    """Runs one ros2 command at a time and streams its output to a queue."""

    def __init__(self, output: queue.Queue[str]) -> None:
        self.output = output
        self.process: subprocess.Popen[str] | None = None
        self.name = ""

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self, name: str, command: list[str]) -> bool:
        if self.running:
            self.output.put(f"[gui] {self.name} is still running; stop it first")
            return False
        self.name = name
        self.output.put(f"[gui] starting {name}")
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        threading.Thread(target=self._pump, args=(self.process, name), daemon=True).start()
        return True

    def _pump(self, process: subprocess.Popen[str], name: str) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            self.output.put(line.rstrip())
        self.output.put(f"[gui] {name} finished (exit code {process.wait()})")

    def stop(self) -> None:
        if self.running:
            self.output.put(f"[gui] stopping {self.name}")
            self.process.send_signal(signal.SIGINT)


class Gp7Gui:
    def __init__(self, root: tk.Tk, node: JointMonitor) -> None:
        self.root = root
        self.node = node
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.runner = ProcessRunner(self.log_queue)

        root.title("GP7 pick-and-place")
        root.geometry("1280x780")
        root.protocol("WM_DELETE_WINDOW", self.close)

        panel = ttk.Frame(root, padding=8)
        panel.pack(side=tk.LEFT, fill=tk.Y)
        self._build_controls(panel)
        self._build_readouts(panel)

        plots = ttk.Frame(root)
        plots.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self._build_plots(plots)

        self._refresh_job = self.root.after(REFRESH_MS, self.refresh)

    # ----------------------------------------------------------------- layout

    def _build_controls(self, panel: ttk.Frame) -> None:
        box = ttk.LabelFrame(panel, text="Box B start (m)", padding=6)
        box.pack(fill=tk.X)
        self.box_x = tk.DoubleVar(value=DEFAULT_BOX_B[0])
        self.box_y = tk.DoubleVar(value=DEFAULT_BOX_B[1])
        for row, (label, var) in enumerate((("x", self.box_x), ("y", self.box_y))):
            ttk.Label(box, text=label).grid(row=row, column=0, sticky=tk.W)
            ttk.Spinbox(
                box, textvariable=var, from_=-0.95, to=0.95, increment=0.05, width=8
            ).grid(row=row, column=1, padx=4, pady=2)

        actions = ttk.LabelFrame(panel, text="Actions", padding=6)
        actions.pack(fill=tk.X, pady=(8, 0))
        self.random_button = ttk.Button(
            actions, text="Place Box B at random", command=self.place_random_box_b
        )
        self.random_button.pack(fill=tk.X)
        ttk.Button(actions, text="Start task (pick & place)", command=self.run_demo).pack(
            fill=tk.X, pady=(4, 0)
        )
        ttk.Button(
            actions, text="Reset world (move Box B, home arm)", command=self.reset_world
        ).pack(fill=tk.X, pady=4)
        ttk.Button(actions, text="Stop", command=self.runner.stop).pack(fill=tk.X)
        ttk.Button(actions, text="Clear graphs", command=self.node.clear).pack(
            fill=tk.X, pady=(4, 0)
        )

    def _build_readouts(self, panel: ttk.Frame) -> None:
        status = ttk.LabelFrame(panel, text="Status", padding=6)
        status.pack(fill=tk.X, pady=(8, 0))
        self.status_label = ttk.Label(status, text="waiting for /joint_states ...", width=34)
        self.status_label.pack(anchor=tk.W)
        self.tool_label = ttk.Label(status, text="gripper: -", font=("TkFixedFont", 9))
        self.tool_label.pack(anchor=tk.W)

        joints = ttk.LabelFrame(panel, text="Joints", padding=6)
        joints.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(joints, text="joint").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(joints, text="angle (deg)").grid(row=0, column=1, padx=6)
        ttk.Label(joints, text="speed (deg/s)").grid(row=0, column=2)
        self.joint_labels = []
        for i, name in enumerate(JOINT_NAMES):
            tk.Label(joints, text=f"J{i + 1}", fg=JOINT_COLOURS[i]).grid(
                row=i + 1, column=0, sticky=tk.W
            )
            angle = ttk.Label(joints, text="-", width=9, anchor=tk.E, font=("TkFixedFont", 9))
            speed = ttk.Label(joints, text="-", width=9, anchor=tk.E, font=("TkFixedFont", 9))
            angle.grid(row=i + 1, column=1)
            speed.grid(row=i + 1, column=2)
            self.joint_labels.append((angle, speed))

        log = ttk.LabelFrame(panel, text="Log", padding=6)
        log.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.log_text = tk.Text(log, width=44, height=14, wrap=tk.WORD, font=("TkFixedFont", 8))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

    def _build_plots(self, frame: ttk.Frame) -> None:
        self.figure = Figure(figsize=(9, 7), layout="constrained")
        grid = self.figure.add_gridspec(2, 2)
        self.ax_angle = self.figure.add_subplot(grid[0, 0])
        self.ax_speed = self.figure.add_subplot(grid[1, 0], sharex=self.ax_angle)
        self.ax_top = self.figure.add_subplot(grid[0, 1])
        self.ax_height = self.figure.add_subplot(grid[1, 1])

        self.ax_angle.set_title("Joint angles")
        self.ax_angle.set_ylabel("deg")
        self.ax_speed.set_title("Joint velocities")
        self.ax_speed.set_ylabel("deg/s")
        self.ax_speed.set_xlabel("simulation time (s)")
        self.angle_lines = [
            self.ax_angle.plot([], [], color=c, lw=1.2, label=f"J{i + 1}")[0]
            for i, c in enumerate(JOINT_COLOURS)
        ]
        self.speed_lines = [
            self.ax_speed.plot([], [], color=c, lw=1.0)[0] for c in JOINT_COLOURS
        ]
        self.ax_angle.legend(loc="upper left", ncol=6, fontsize=7, frameon=False)

        self.ax_top.set_title("Gripper path (top view)")
        self.ax_top.set_xlabel("x (m)")
        self.ax_top.set_ylabel("y (m)")
        self.ax_top.set_aspect("equal")
        self.ax_top.set_xlim(-1.0, 1.0)
        self.ax_top.set_ylim(-1.0, 1.0)
        self.ax_top.add_patch(Circle((0.0, 0.0), 0.927, fill=False, ls="--", ec="0.6"))
        self.ax_top.plot(0, 0, marker="s", ms=10, color="0.3")
        self.ax_top.annotate("robot", (0.0, -0.09), ha="center", color="0.3")
        half = BOX_A_SIZE / 2.0
        self.ax_top.add_patch(Rectangle(
            (BOX_A_CENTRE[0] - half, BOX_A_CENTRE[1] - half), BOX_A_SIZE, BOX_A_SIZE,
            fc="#c9d6e8", ec="#4a6a96",
        ))
        self.ax_top.annotate("Box A", BOX_A_CENTRE, ha="center", va="center", color="#4a6a96")
        size = BOX_B_SIZE[0]
        self.box_b_patch = Rectangle((0, 0), size, size, fc="#f2c38b", ec="#b8732b")
        self.ax_top.add_patch(self.box_b_patch)
        (self.path_line,) = self.ax_top.plot([], [], color="#d62728", lw=1.2)
        (self.tool_dot,) = self.ax_top.plot([], [], "o", color="#d62728", ms=5)

        self.ax_height.set_title("Gripper height and speed")
        self.ax_height.set_xlabel("simulation time (s)")
        self.ax_height.set_ylabel("height z (m)", color="#1f77b4")
        (self.height_line,) = self.ax_height.plot([], [], color="#1f77b4", lw=1.2)
        self.ax_tool_speed = self.ax_height.twinx()
        self.ax_tool_speed.set_ylabel("speed (m/s)", color="#ff7f0e")
        self.ax_tool_speed.grid(False)
        (self.tool_speed_line,) = self.ax_tool_speed.plot([], [], color="#ff7f0e", lw=1.0)

        self.canvas = FigureCanvasTkAgg(self.figure, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ---------------------------------------------------------------- actions

    def _box_args(self) -> list[str] | None:
        try:
            x, y = float(self.box_x.get()), float(self.box_y.get())
        except (tk.TclError, ValueError):
            self.log("[gui] Box B x/y must be numbers")
            return None
        return ["-p", f"box_b_x:={x}", "-p", f"box_b_y:={y}"]

    def run_demo(self) -> None:
        args = self._box_args()
        if args is not None:
            self.node.clear()
            self.runner.start(
                "pick & place",
                ["ros2", "run", "gp7robot_description", "gp7_pick_place",
                 "--ros-args", "-p", "use_sim_time:=true", *args],
            )

    def place_random_box_b(self) -> None:
        if self.runner.running:
            self.log(f"[gui] {self.runner.name} is still running; stop it first")
            return
        self.random_button.state(["disabled"])
        self.log("[gui] searching for a reachable random Box B position ...")
        # IK checks take a second or two: keep them off the Tk thread.
        threading.Thread(target=self._random_worker, daemon=True).start()

    def _random_worker(self) -> None:
        try:
            result: tuple[float, float] | str = random_box_b()
        except RuntimeError as error:
            result = str(error)
        self.root.after(0, self._random_done, result)

    def _random_done(self, result: tuple[float, float] | str) -> None:
        self.random_button.state(["!disabled"])
        if isinstance(result, str):
            self.log(f"[gui] {result}")
            return
        x, y = result
        self.box_x.set(x)
        self.box_y.set(y)
        self.log(f"[gui] Box B -> ({x:.3f}, {y:.3f}) m (plan checked)")
        self.reset_world()  # teleports Box B there and homes the arm

    def reset_world(self) -> None:
        args = self._box_args()
        if args is not None:
            self.runner.start(
                "reset world",
                ["ros2", "run", "gp7robot_description", "gp7_reset_world", "--ros-args", *args],
            )

    def log(self, line: str) -> None:
        # ROS log lines look like "[INFO] [stamp] [node]: text"; keep the text.
        if "]: " in line:
            level = line[1:line.index("]")] if line.startswith("[") else ""
            line = line.split("]: ", 1)[1]
            if level in ("WARN", "ERROR"):
                line = f"{level}: {line}"
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # ---------------------------------------------------------------- refresh

    def refresh(self) -> None:
        # Process every waiting ROS message, then redraw.
        for _ in range(500):
            before = self.node.message_count
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if self.node.message_count == before:
                break
        while not self.log_queue.empty():
            self.log(self.log_queue.get_nowait())

        self._update_box_b()
        if self.node.times:
            self._update_readouts()
            self._update_plots()
        self.canvas.draw_idle()
        self._refresh_job = self.root.after(REFRESH_MS, self.refresh)

    def _update_box_b(self) -> None:
        try:
            x, y = float(self.box_x.get()), float(self.box_y.get())
        except (tk.TclError, ValueError):
            return
        half = BOX_B_SIZE[0] / 2.0
        self.box_b_patch.set_xy((x - half, y - half))

    def _update_readouts(self) -> None:
        q = np.degrees(self.node.joints[-1])
        v = np.degrees(self.node.velocities[-1])
        for (angle, speed), qi, vi in zip(self.joint_labels, q, v):
            angle.configure(text=f"{qi:8.1f}")
            speed.configure(text=f"{vi:8.1f}")
        p = self.node.tool[-1]
        self.tool_label.configure(
            text=f"gripper: x {p[0]:+.3f}  y {p[1]:+.3f}  z {p[2]:+.3f} m"
        )
        busy = f"  |  running: {self.runner.name}" if self.runner.running else ""
        self.status_label.configure(text=f"sim time {self.node.times[-1]:.1f} s{busy}")

    def _update_plots(self) -> None:
        t = np.fromiter(self.node.times, float)
        q = np.degrees(np.array(self.node.joints))
        v = np.degrees(np.array(self.node.velocities))
        tool = np.array(self.node.tool)
        for i in range(6):
            self.angle_lines[i].set_data(t, q[:, i])
            self.speed_lines[i].set_data(t, v[:, i])
        self.path_line.set_data(tool[:, 0], tool[:, 1])
        self.tool_dot.set_data([tool[-1, 0]], [tool[-1, 1]])
        self.height_line.set_data(t, tool[:, 2])
        self.tool_speed_line.set_data(t, np.fromiter(self.node.tool_speed, float))

        start = max(t[0], t[-1] - HISTORY_SECONDS)
        end = max(t[-1], start + 1.0)
        for axis in (self.ax_angle, self.ax_speed, self.ax_height, self.ax_tool_speed):
            axis.set_xlim(start, end)
            axis.relim()
            axis.autoscale_view(scalex=False)

    def close(self) -> None:
        self.runner.stop()
        self.root.after_cancel(self._refresh_job)
        self.root.quit()
        self.root.destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointMonitor()
    root = tk.Tk()
    Gp7Gui(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
