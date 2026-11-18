"""Generate the figures of the GP7 learning guide from the real package code.

Run from the workspace with the package importable, e.g.

    source install/setup.bash
    python3 src/gp7robot_description/docs/learning_guide/make_figures.py

Figures are written next to this file in ``figures/``.
"""

from __future__ import annotations

import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle  # noqa: E402

from gp7robot_description.kinematics import (  # noqa: E402
    DH_TABLE,
    UPRIGHT_REFERENCE_CONFIGURATION,
    _dh_transform,
    forward_kinematics,
)
from gp7robot_description.pick_place import (  # noqa: E402
    BLEND_START_FRACTION,
    GRIPPER_LINK_OFFSET,
    MAX_JOINT_SPEEDS,
    SAMPLE_PERIOD,
    rounded_polyline,
)
from gp7robot_description.kinematics import ACCELERATION_LIMITS  # noqa: E402
from gp7robot_description.trajectory import (  # noqa: E402
    path_speed_profile,
    scurve_segment,
    superpose,
)
from gp7robot_description.workspace import (  # noqa: E402
    DEFAULT_PARAMETERS,
    MAX_RADIUS,
    MIN_RADIUS,
    _PlanChecker,
    check_box_b,
    overlaps_box_a,
)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({
    "font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
    "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 160,
})
BLUE, ORANGE, GREEN, RED, PURPLE, BROWN = (
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b")
JC = (BLUE, ORANGE, GREEN, RED, PURPLE, BROWN)


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), bbox_inches="tight")
    plt.close(fig)
    print("wrote", name)


# ------------------------------------------------------------------ helpers

def chain_points(q):
    """Origins of every DH frame (base, after joint 1, ..., Link6)."""
    t = np.eye(4)
    pts = [t[:3, 3].copy()]
    for row, angle in zip(DH_TABLE, q):
        t = t @ _dh_transform(row, float(angle))
        pts.append(t[:3, 3].copy())
    return np.array(pts)


def gripper_point(q):
    link6 = forward_kinematics(q)
    rot = forward_kinematics(UPRIGHT_REFERENCE_CONFIGURATION)[:3, :3]
    return link6[:3, 3] + link6[:3, :3] @ (rot.T @ np.array([0, 0, -GRIPPER_LINK_OFFSET]))


def full_task(box_b=(0.35, 0.25)):
    """Re-create every trajectory of PickPlaceController.run_demo offline."""
    p = dict(DEFAULT_PARAMETERS)
    planner = _PlanChecker(p)
    scale = p["speed_scale"]

    def joint_traj(a, b):
        return scurve_segment(a, b, MAX_JOINT_SPEEDS * scale, ACCELERATION_LIMITS,
                              sample_period=SAMPLE_PERIOD)

    flange = np.array([0, 0, GRIPPER_LINK_OFFSET])
    b = np.array([box_b[0], box_b[1], p["box_b_center_z"]])
    pick = b + flange
    clear = b + np.array([0, 0, p["clearance_height"]]) + flange
    dest = np.array([p["box_a_x"], p["box_a_y"], p["box_a_top_z"] + 0.025])
    release = dest + flange + np.array([0, 0, 0.005])
    above_a = np.array([dest[0], dest[1], clear[2]])
    retreat = release + np.array([0, 0, p["retreat_height"]])

    start = np.zeros(6)
    q_clear = planner._solve(clear, UPRIGHT_REFERENCE_CONFIGURATION)
    q_lift = q_clear.copy()
    q_lift[0] = start[0]
    lift = joint_traj(start, q_lift)
    rotate = joint_traj(q_lift, q_clear)
    seg1 = superpose([lift, rotate], [0.0, BLEND_START_FRACTION * lift[-1].time_from_start],
                     sample_period=SAMPLE_PERIOD)
    seg2 = planner._cartesian_path(q_clear, [clear, pick])
    seg3 = planner._cartesian_path(seg2[-1].positions, [pick, clear, above_a, release])
    seg4 = planner._cartesian_path(seg3[-1].positions, [release, retreat])
    segments = [
        ("1 lift + rotate J1", seg1),
        ("2 lower to Box B", seg2),
        ("3 carry to Box A", seg3),
        ("4 lift off", seg4),
    ]
    pauses = [0.0, 0.75 + 0.25, 0.75 + 0.5 + 0.25]  # settle + gripper signals (approx.)
    return segments, pauses, dict(pick=pick, clear=clear, above_a=above_a,
                                   release=release, retreat=retreat, q_clear=q_clear)


# ------------------------------------------------------------------ figures

def fig_architecture():
    fig, ax = plt.subplots(figsize=(8.6, 5.0))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 67)
    ax.axis("off")

    def box(x, y, w, h, title, body, color):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.6",
                                    fc=color, ec="0.25", lw=1))
        ax.text(x + w / 2, y + h - 2.2, title, ha="center", va="top",
                fontsize=9.5, weight="bold")
        ax.text(x + w / 2, y + h - 6.2, body, ha="center", va="top", fontsize=7.6,
                linespacing=1.35)

    def arrow(a, b, text, rad=0.0, offset=(0, 1.5)):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=12, lw=1.2,
                                     color="0.2", connectionstyle=f"arc3,rad={rad}"))
        mx, my = (a[0] + b[0]) / 2 + offset[0], (a[1] + b[1]) / 2 + offset[1]
        ax.text(mx, my, text, ha="center", va="bottom", fontsize=7.2, color="#7a2e00",
                bbox=dict(fc="white", ec="none", pad=0.6))

    box(1, 40, 25, 18, "gp7_gui  (gui.py)", "Tkinter window\nbuttons + live plots\nsubscribes /joint_states\nstarts other programs", "#e8f0fb")
    box(37, 40, 26, 18, "gp7_pick_place", "(pick_place.py)\nplans the whole task\nkinematics.py + trajectory.py\npublishes trajectories", "#fdeedd")
    box(74, 40, 25, 18, "gp7_reset_world", "(reset_world.py)\nteleports Box B (gz service)\nhomes the arm", "#fdeedd")
    box(30, 18, 40, 13, "ros_gz_bridge  (parameter_bridge)", "translates ROS 2 messages <-> Gazebo messages", "#eef7ea")
    box(1, 0, 98, 12, "Gazebo Sim  (gz sim + world .sdf)", "physics 1 kHz  |  JointTrajectoryController (PID per joint)  |  DetachableJoint (gripper)  |  JointStatePublisher  |  /clock", "#f3eefa")

    arrow((26.6, 49), (36.4, 49), "subprocess\n(ros2 run ...)")
    ax.add_patch(FancyArrowPatch((14, 58.8), (86, 58.8), arrowstyle="-|>", mutation_scale=12,
                                 lw=1.2, color="0.2", connectionstyle="arc3,rad=-0.12"))
    ax.text(50, 64.5, "subprocess (ros2 run ...)", ha="center", fontsize=7.2, color="#7a2e00")
    arrow((47, 39.5), (45, 31.6), "JointTrajectory\nattach / detach", offset=(-9, -1))
    arrow((56, 31.6), (56, 39.5), "/joint_states\n/clock", offset=(8, -2))
    arrow((14, 39.5), (31, 26), "/joint_states", rad=0.15, offset=(-6, -2))
    arrow((50, 17.6), (50, 12.6), "gz topics", offset=(8, -1.6))
    save(fig, "architecture.png")


def fig_ros_graph():
    fig, ax = plt.subplots(figsize=(8.4, 2.6))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 30)
    ax.axis("off")

    def node(x, y, label):
        ax.add_patch(FancyBboxPatch((x, y), 20, 9, boxstyle="round,pad=0.5", fc="#e8f0fb", ec=BLUE))
        ax.text(x + 10, y + 4.5, label, ha="center", va="center", fontsize=8.5)

    def topic(x, y, label):
        ax.add_patch(Rectangle((x, y), 24, 6, fc="#fff6d6", ec="#b08800"))
        ax.text(x + 12, y + 3, label, ha="center", va="center", fontsize=8, family="monospace")

    node(2, 18, "publisher node\n(pick_place)")
    topic(38, 19.5, "/joint_trajectory")
    node(78, 18, "subscriber\n(bridge -> Gazebo)")
    ax.annotate("", (38, 22.5), (22.5, 22.5), arrowprops=dict(arrowstyle="-|>", lw=1.3))
    ax.annotate("", (77.5, 22.5), (62, 22.5), arrowprops=dict(arrowstyle="-|>", lw=1.3))
    node(78, 2, "publisher\n(Gazebo via bridge)")
    topic(38, 3.5, "/joint_states")
    node(2, 2, "subscribers\n(pick_place, gui)")
    ax.annotate("", (62, 6.5), (77.5, 6.5), arrowprops=dict(arrowstyle="-|>", lw=1.3))
    ax.annotate("", (22.5, 6.5), (38, 6.5), arrowprops=dict(arrowstyle="-|>", lw=1.3))
    ax.text(50, 14.5, "a topic is a named channel; nodes never call each other directly",
            ha="center", fontsize=8, style="italic", color="0.35")
    save(fig, "ros_topics.png")


def fig_frames_2d():
    """Homogeneous transform illustration: a point seen from two frames."""
    fig, ax = plt.subplots(figsize=(5.2, 4.0))
    ax.set_aspect("equal")
    ax.set_xlim(-0.3, 3.3)
    ax.set_ylim(-0.3, 2.8)

    def frame(o, ang, name, c):
        R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
        for v, lab in ((R[:, 0], "x"), (R[:, 1], "y")):
            ax.annotate("", o + 0.8 * v, o, arrowprops=dict(arrowstyle="-|>", color=c, lw=1.6))
            ax.text(*(o + 0.92 * v), lab + name[-1], color=c, fontsize=9)
        ax.text(o[0] - 0.25, o[1] - 0.22, name, color=c, fontsize=9, weight="bold")
        return R

    o_a = np.array([0.0, 0.0])
    frame(o_a, 0.0, "frame A", BLUE)
    o_b = np.array([1.6, 0.7])
    R = frame(o_b, math.radians(35), "frame B", RED)
    p_b = np.array([0.9, 0.5])
    p_a = o_b + R @ p_b
    ax.plot(*p_a, "ko")
    ax.text(p_a[0] + 0.06, p_a[1] + 0.06, "point P", fontsize=9)
    ax.annotate("", o_b, o_a, arrowprops=dict(arrowstyle="-|>", ls="--", color="0.5"))
    ax.text(0.75, 0.48, "translation t", color="0.4", fontsize=8, rotation=24)
    ax.set_title("P_A = R * P_B + t   (rotate, then shift)", fontsize=9)
    save(fig, "frames_2d.png")


def fig_two_link_ik():
    """Elbow-up / elbow-down: why IK has several answers."""
    l1, l2 = 1.0, 0.8
    target = np.array([1.2, 0.8])
    d = np.hypot(*target)
    c2 = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    fig, ax = plt.subplots(figsize=(5.0, 3.8))
    ax.set_aspect("equal")
    for sign, col, lab in ((1, BLUE, "elbow down"), (-1, ORANGE, "elbow up")):
        t2 = sign * math.acos(c2)
        t1 = math.atan2(target[1], target[0]) - math.atan2(l2 * math.sin(t2), l1 + l2 * math.cos(t2))
        e = np.array([l1 * math.cos(t1), l1 * math.sin(t1)])
        ax.plot([0, e[0], target[0]], [0, e[1], target[1]], "-o", color=col, lw=3, ms=6, label=lab)
    ax.add_patch(Circle((0, 0), l1 + l2, fill=False, ls="--", ec="0.6"))
    ax.plot(*target, "k*", ms=13)
    ax.text(target[0] + 0.05, target[1] + 0.05, "target", fontsize=9)
    ax.set_xlim(-0.4, 2.0)
    ax.set_ylim(-0.6, 1.9)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("Same target, two joint solutions (IK branches)", fontsize=9)
    save(fig, "two_link_ik.png")


def fig_arm_poses():
    fig = plt.figure(figsize=(8.6, 4.0))
    poses = [("home  q = 0", np.zeros(6)),
             ("upright reference pose", UPRIGHT_REFERENCE_CONFIGURATION)]
    for k, (title, q) in enumerate(poses):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        pts = chain_points(q)
        g = gripper_point(q)
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="0.25", lw=3, ms=4)
        ax.plot([pts[-1, 0], g[0]], [pts[-1, 1], g[1]], [pts[-1, 2], g[2]], "-", color=RED, lw=3)
        for i, p in enumerate(pts[1:]):
            ax.text(*p, f" {i + 1}", fontsize=7, color=BLUE)
        ax.scatter(*g, color=RED, s=20)
        # Boxes A and B on the floor
        for cx, cy, s, h, c in ((0.5, 0.5, 0.3, 0.2, "#4a6a96"), (0.35, 0.25, 0.05, 0.05, "#d0661c")):
            xs = [cx - s / 2, cx + s / 2, cx + s / 2, cx - s / 2, cx - s / 2]
            ys = [cy - s / 2, cy - s / 2, cy + s / 2, cy + s / 2, cy - s / 2]
            ax.plot(xs, ys, [h] * 5, color=c)
            ax.plot(xs, ys, [0] * 5, color=c, alpha=0.5)
        ax.set_xlim(-0.4, 0.8)
        ax.set_ylim(-0.4, 0.8)
        ax.set_zlim(0, 1.2)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title(title, fontsize=9)
        ax.view_init(elev=20, azim=-60)
    fig.text(0.5, 0.02, "dots = joint frame origins from the DH chain; red = 75 mm gripper below Link6",
             ha="center", fontsize=8, color="0.35")
    save(fig, "arm_poses.png")


def fig_scurve():
    seg = scurve_segment([0.0], [1.0], [1.0], [1.0], sample_period=0.002)
    t = np.array([s.time_from_start for s in seg])
    x = np.array([s.positions[0] for s in seg])
    v = np.array([s.velocities[0] for s in seg])
    a = np.gradient(v, t)
    # Trapezoid with the same duration for comparison
    T = t[-1]
    ta = T / 3
    vp = 1 / (T - ta)
    vt = np.where(t < ta, vp * t / ta, np.where(t > T - ta, vp * (T - t) / ta, vp))
    at = np.gradient(vt, t)
    fig, axs = plt.subplots(1, 3, figsize=(8.6, 2.6))
    axs[0].plot(t, x, color=BLUE)
    axs[0].set_title("position s(t)")
    axs[1].plot(t, v, color=BLUE, label="S-curve (used)")
    axs[1].plot(t, vt, "--", color="0.5", label="trapezoid")
    axs[1].set_title("velocity")
    axs[1].legend(fontsize=7)
    axs[2].plot(t, a, color=BLUE)
    axs[2].plot(t, at, "--", color="0.5")
    axs[2].set_title("acceleration")
    for ax in axs:
        ax.set_xlabel("time (s)")
        for k in (1, 2):
            ax.axvline(k * T / 3, color="0.8", lw=0.8)
    fig.tight_layout()
    save(fig, "scurve.png")


def fig_profiles_compare():
    """scurve_segment (thirds) vs path_speed_profile (ramp from accel limit)."""
    L, v, a = 1.1, 0.25, 0.40
    s1 = scurve_segment([0.0], [L], [v], [a], sample_period=0.01)
    s2 = path_speed_profile(L, v, a, sample_period=0.01)
    fig, ax = plt.subplots(figsize=(6.4, 2.5))
    ax.plot([s.time_from_start for s in s1], [s.velocities[0] for s in s1], "--", color="0.5",
            label="scurve_segment: ramps are always 1/3 of the move")
    ax.plot([s.time_from_start for s in s2], [s.velocities[0] for s in s2], color=BLUE,
            label="path_speed_profile: ramp time = 2v/a")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("tool speed (m/s)")
    ax.set_title(f"1.1 m path, v_max {v} m/s, a_max {a} m/s^2", fontsize=9)
    ax.legend(fontsize=7, loc="lower center")
    save(fig, "profiles_compare.png")


def fig_corner_blend():
    wps = [np.array([0.35, 0.25, 0.10]), np.array([0.35, 0.25, 0.65]),
           np.array([0.50, 0.50, 0.65]), np.array([0.50, 0.50, 0.305])]
    pts, _ = rounded_polyline(wps, 0.10)
    # Distance along the floor from the start, so the 3D path is shown in a plane.
    h = np.concatenate(([0], np.cumsum(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1))))
    hw = [0.0, 0.0, float(np.linalg.norm(wps[2][:2] - wps[1][:2])),
          float(np.linalg.norm(wps[2][:2] - wps[1][:2]))]
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    ax.plot(hw, [w[2] for w in wps], "--o", color="0.6", label="waypoints (sharp corners)")
    ax.plot(h, pts[:, 2], color=RED, lw=2, label="planned path (10 cm arcs)")
    ax.add_patch(Rectangle((hw[2] - 0.15, 0), 0.3, 0.2, fc="#c9d6e8", ec="#4a6a96"))
    ax.text(hw[2], 0.08, "Box A", ha="center", color="#4a6a96")
    ax.add_patch(Rectangle((-0.025, 0), 0.05, 0.05, fc="#f2c38b", ec="#b8732b"))
    ax.set_aspect("equal")
    ax.set_xlabel("horizontal distance from Box B (m)")
    ax.set_ylabel("height z (m)")
    ax.set_ylim(-0.02, 0.85)
    ax.legend(fontsize=7, loc="upper center", ncol=1)
    ax.set_title("Carry path: corners rounded, last part straight down", fontsize=9)
    save(fig, "corner_blend.png")


def fig_task_timeline():
    segments, pauses, info = full_task()
    t0 = 0.0
    T, Q, G = [], [], []
    bounds = []
    for k, (name, seg) in enumerate(segments):
        if k:
            t0 += pauses[k - 1]
        t = np.array([s.time_from_start for s in seg]) + t0
        T.append(t)
        Q.append(np.array([s.positions for s in seg]))
        G.append(np.array([gripper_point(s.positions) for s in seg[::4]]))
        bounds.append((t[0], t[-1], name))
        t0 = t[-1]
    fig, axs = plt.subplots(2, 1, figsize=(8.6, 5.4), sharex=True, gridspec_kw=dict(hspace=0.3))
    for k in range(len(T)):
        for j in range(6):
            axs[0].plot(T[k], np.degrees(Q[k][:, j]), color=JC[j], lw=1.2,
                        label=f"J{j + 1}" if k == 0 else None)
        tg = T[k][::4][:len(G[k])]
        sp = np.linalg.norm(np.gradient(G[k], tg, axis=0), axis=1) if len(tg) > 2 else np.zeros(len(tg))
        axs[1].plot(tg, G[k][:, 2], color=BLUE, lw=1.4, label="height z (m)" if k == 0 else None)
        axs[1].plot(tg, sp, color=ORANGE, lw=1.2, label="speed (m/s)" if k == 0 else None)
    for a, b, name in bounds:
        for ax in axs:
            ax.axvspan(a, b, color="0.93", zorder=-1)
        axs[0].text((a + b) / 2, axs[0].get_ylim()[1] if False else 230, name, ha="center",
                    fontsize=7, color="0.3")
    axs[0].set_ylabel("joint angle (deg)")
    axs[0].legend(ncol=6, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.04), frameon=False)
    axs[1].set_ylabel("gripper")
    axs[1].set_xlabel("time (s)  (gaps = gripper attach / release and settling)")
    axs[1].legend(fontsize=7, loc="upper right")
    save(fig, "task_timeline.png")
    return info


def fig_task_3d(info):
    segments, _, _ = full_task()
    fig = plt.figure(figsize=(6.0, 4.8))
    ax = fig.add_subplot(projection="3d")
    cols = (PURPLE, GREEN, RED, BLUE)
    for (name, seg), c in zip(segments, cols):
        g = np.array([gripper_point(s.positions) for s in seg[::5]])
        ax.plot(g[:, 0], g[:, 1], g[:, 2], color=c, lw=2, label=name)
    for cx, cy, s, h, c in ((0.5, 0.5, 0.3, 0.2, "#4a6a96"), (0.35, 0.25, 0.05, 0.05, "#d0661c")):
        xs = [cx - s / 2, cx + s / 2, cx + s / 2, cx - s / 2, cx - s / 2]
        ys = [cy - s / 2, cy - s / 2, cy + s / 2, cy + s / 2, cy - s / 2]
        for z in (0, h):
            ax.plot(xs, ys, [z] * 5, color=c)
    ax.scatter([0], [0], [0], color="k", s=30)
    ax.text(0, 0, 0.03, "robot base", fontsize=7)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.legend(fontsize=7, loc="upper left")
    ax.view_init(elev=24, azim=-125)
    ax.set_title("Gripper path for the whole task (box B at 0.35, 0.25)", fontsize=9)
    save(fig, "task_3d.png")


def fig_pid():
    """Toy 1-DOF joint with gravity: P, PD and PID following a step."""
    dt, T = 0.001, 1.5
    J, m_g = 0.05, 2.0

    def sim(kp, ki, kd):
        x, v, integ, prev = 0.0, 0.0, 0.0, 0.0
        xs = []
        for k in range(int(T / dt)):
            target = 1.0
            e = target - x
            integ = float(np.clip(integ + e * dt, -1, 1))
            u = kp * e + ki * integ + kd * (e - prev) / dt
            prev = e
            tau = u - m_g * math.cos(x)
            v += tau / J * dt
            x += v * dt
            xs.append(x)
        return np.arange(len(xs)) * dt, np.array(xs)

    fig, ax = plt.subplots(figsize=(6.0, 2.6))
    for (kp, ki, kd), lab, c in (((20, 0, 0), "P only: oscillates", "0.6"),
                                  ((20, 0, 1.5), "PD: smooth but sags (gravity)", ORANGE),
                                  ((20, 40, 1.5), "PID: integral removes the sag", BLUE)):
        t, x = sim(kp, ki, kd)
        ax.plot(t, x, color=c, label=lab)
    ax.axhline(1.0, color="k", ls=":", lw=1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("joint angle (rad)")
    ax.set_title("Toy joint following a step command", fontsize=9)
    ax.legend(fontsize=7, loc="lower right")
    save(fig, "pid.png")


def fig_reachability():
    xs = np.arange(-0.9, 0.91, 0.06)
    ok, bad, skip = [], [], []
    for x in xs:
        for y in xs:
            r = math.hypot(x, y)
            if r < MIN_RADIUS or r > MAX_RADIUS:
                continue
            if overlaps_box_a(x, y):
                skip.append((x, y))
                continue
            (ok if check_box_b(float(x), float(y)) is None else bad).append((x, y))
    fig, ax = plt.subplots(figsize=(5.2, 5.0))
    ax.set_aspect("equal")
    if ok:
        ax.scatter(*np.array(ok).T, s=14, color=GREEN, label="plan works")
    if bad:
        ax.scatter(*np.array(bad).T, s=14, color=RED, marker="x", label="plan fails (IK / limits)")
    ax.add_patch(Circle((0, 0), MIN_RADIUS, fill=False, ls="--", ec="0.5"))
    ax.add_patch(Circle((0, 0), MAX_RADIUS, fill=False, ls="--", ec="0.5"))
    ax.add_patch(Rectangle((0.35, 0.35), 0.3, 0.3, fc="#c9d6e8", ec="#4a6a96"))
    ax.text(0.5, 0.5, "A", ha="center", va="center", color="#4a6a96", weight="bold")
    ax.plot(0, 0, "ks")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(fontsize=7, loc="lower left")
    ax.set_title("Where can Box B start? (checked with workspace.py)", fontsize=9)
    save(fig, "reachability.png")
    return len(ok), len(bad)


if __name__ == "__main__":
    which = set(sys.argv[1:])
    if not which or "fast" in which:
        fig_architecture()
        fig_ros_graph()
        fig_frames_2d()
        fig_two_link_ik()
        fig_arm_poses()
        fig_scurve()
        fig_profiles_compare()
        fig_corner_blend()
        fig_pid()
        info = fig_task_timeline()
        fig_task_3d(info)
    if not which or "reach" in which:
        print("reachability counts (ok, fail):", fig_reachability())
