"""Build GP7_learning_guide.pdf from the chapters below and the real source code.

    python3 make_figures.py      # (needs the package importable) -> figures/
    python3 build_guide.py       # -> ../GP7_learning_guide.pdf

Code excerpts are pulled out of the package files at build time (by function
name), so the guide always shows the code as it is now.
"""

from __future__ import annotations

import ast
import html
import os
import re
import subprocess
import sys

from pygments import highlight
from pygments.formatters import HtmlFormatter
from pygments.lexers import PythonLexer, XmlLexer

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT_HTML = os.path.join(HERE, "guide.html")
OUT_PDF = os.path.abspath(os.path.join(HERE, "..", "GP7_learning_guide.pdf"))

SOURCES = {
    "kinematics.py": "gp7robot_description/kinematics.py",
    "trajectory.py": "gp7robot_description/trajectory.py",
    "pick_place.py": "gp7robot_description/pick_place.py",
    "workspace.py": "gp7robot_description/workspace.py",
    "reset_world.py": "gp7robot_description/reset_world.py",
    "gui.py": "gp7robot_description/gui.py",
    "manual_joint.py": "gp7robot_description/manual_joint.py",
    "gazebo_sim.launch.py": "launch/gazebo_sim.launch.py",
    "display.launch.py": "launch/display.launch.py",
    "setup.py": "setup.py",
    "world.sdf": "worlds/gp7_pick_place_reachable.sdf",
    "GP7.urdf": "urdf/GP7.urdf",
}


def _read(name: str) -> str:
    with open(os.path.join(PKG, SOURCES[name]), encoding="utf-8") as handle:
        return handle.read()


def _find(tree: ast.AST, qualname: str) -> ast.AST:
    node = tree
    for part in qualname.split("."):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.ClassDef)) and child.name == part:
                node = child
                break
        else:
            raise KeyError(f"{qualname} not found")
    return node


def code_block(name: str, spec: str) -> str:
    """spec: a qualname (function/class) or 'L10-40' (line range)."""
    text = _read(name)
    lines = text.splitlines()
    if spec.startswith("L"):
        a, b = (int(v) for v in spec[1:].split("-"))
        start, end = a, b
    else:
        node = _find(ast.parse(text), spec)
        start = min([node.lineno] + [d.lineno for d in getattr(node, "decorator_list", [])])
        end = node.end_lineno
    snippet = "\n".join(lines[start - 1:end])
    lexer = XmlLexer() if name.endswith((".sdf", ".urdf", ".xml")) else PythonLexer()
    body = highlight(snippet, lexer, HtmlFormatter(nowrap=True)).rstrip("\n")
    # One <pre> with the line number inside each line, so long listings can
    # break across pages (a table cell cannot).
    width = len(str(end))
    numbered = "\n".join(
        f'<span class="ln">{str(start + i).rjust(width)}</span>{line}'
        for i, line in enumerate(body.split("\n"))
    )
    return (
        f'<div class="code"><div class="code-title">{html.escape(SOURCES[name])}'
        f' &nbsp;·&nbsp; lines {start}–{end}</div>'
        f'<pre class="src">{numbered}</pre></div>'
    )


def figure(name: str, caption: str, width: str = "100%") -> str:
    return (
        f'<figure><img src="figures/{name}" style="width:{width}">'
        f'<figcaption>{caption}</figcaption></figure>'
    )


def expand(text: str) -> str:
    text = re.sub(r"\{\{CODE:([^:}]+):([^}]+)\}\}", lambda m: code_block(m[1], m[2]), text)
    text = re.sub(
        r"\{\{FIG:([^|}]+)\|([^|}]+)(?:\|([^}]+))?\}\}",
        lambda m: figure(m[1], m[2], m[3] or "100%"),
        text,
    )
    return text


CSS = """
@page { size: A4; margin: 18mm 17mm 20mm 17mm;
  @bottom-center { content: counter(page); font: 9pt 'DejaVu Sans', sans-serif; color: #777; } }
@page :first { @bottom-center { content: none; } }
html { font-family: 'DejaVu Serif', Georgia, serif; font-size: 10.4pt; line-height: 1.5; color: #1d1d1f; }
h1, h2, h3, h4, .callout b.t, figcaption, th, .toc, .code-title { font-family: 'DejaVu Sans', Arial, sans-serif; }
h1 { font-size: 21pt; color: #12355b; border-bottom: 3px solid #12355b; padding-bottom: 4px;
     margin-top: 0; page-break-before: always; }
h1.nobreak { page-break-before: avoid; }
h2 { font-size: 14pt; color: #1b4f8a; margin-top: 20px; border-bottom: 1px solid #c9d6e8; padding-bottom: 2px; }
h3 { font-size: 11.5pt; color: #24364a; margin-top: 15px; margin-bottom: 4px; }
h2, h3 { page-break-after: avoid; }
p { margin: 6px 0; text-align: justify; }
code { font-family: 'DejaVu Sans Mono', monospace; font-size: 8.8pt; background: #f1f3f6; padding: 0 3px; border-radius: 3px; }
pre { font-family: 'DejaVu Sans Mono', monospace; font-size: 8pt; line-height: 1.38; margin: 0; white-space: pre-wrap; }
pre.term { background: #1f2430; color: #e6e6e6; padding: 8px 10px; border-radius: 5px; margin: 8px 0; page-break-inside: avoid; }
pre.term .c { color: #8fbf8f; }
pre.math { font-family: 'DejaVu Sans Mono', monospace; font-size: 9pt; background: #f7f9fc; border-left: 3px solid #1b4f8a; padding: 7px 10px; margin: 8px 0; page-break-inside: avoid; }
.code { border: 1px solid #d6dbe3; border-radius: 5px; margin: 9px 0; }
.code-title { background: #e9eef5; font-size: 8pt; color: #36506e; padding: 3px 8px; border-bottom: 1px solid #d6dbe3; }
.code pre.src { background: #fbfcfd; padding: 4px 8px; white-space: pre-wrap; }
.code .ln { color: #9aa3ad; padding-right: 10px; user-select: none; }
.code-title { break-after: avoid; }
table { border-collapse: collapse; margin: 8px 0; font-size: 9pt; page-break-inside: avoid; }
th, td { border: 1px solid #cfd6df; padding: 3px 7px; text-align: left; vertical-align: top; }
th { background: #e9eef5; font-size: 8.8pt; }
figure { margin: 10px 0; text-align: center; page-break-inside: avoid; }
figcaption { font-size: 8.6pt; color: #555; margin-top: 3px; }
.callout { border-radius: 6px; padding: 7px 11px; margin: 10px 0; page-break-inside: avoid; }
.callout b.t { display: block; margin-bottom: 2px; font-size: 9.5pt; }
.idea { background: #eef5ff; border-left: 4px solid #1b6fd1; }
.warn { background: #fff4e5; border-left: 4px solid #e08a00; }
.try  { background: #edf8ee; border-left: 4px solid #2f9a3c; }
.code-note { background: #f5f0fb; border-left: 4px solid #7b4bb7; }
ul, ol { margin: 5px 0 5px 0; padding-left: 22px; }
li { margin: 2px 0; }
.cover { text-align: center; padding-top: 70mm; page-break-after: always; }
.cover .t1 { font-family: 'DejaVu Sans', sans-serif; font-size: 30pt; color: #12355b; font-weight: bold; line-height: 1.2; }
.cover .t2 { font-family: 'DejaVu Sans', sans-serif; font-size: 14pt; color: #1b4f8a; margin-top: 14px; }
.cover .t3 { font-size: 10.5pt; color: #555; margin-top: 50px; line-height: 1.7; }
.toc { font-size: 10pt; columns: 2; column-gap: 24px; }
.toc a { color: #1d1d1f; text-decoration: none; }
.toc div.part { font-weight: bold; color: #12355b; margin-top: 8px; break-after: avoid; }
.toc div.ch { margin-left: 10px; }
.small { font-size: 8.8pt; color: #555; }
.two { display: flex; gap: 14px; align-items: flex-start; }
.two > * { flex: 1; }
.kw { font-weight: bold; color: #12355b; }
""" + HtmlFormatter(style="friendly").get_style_defs(".code")


# =========================================================================
# Chapters. Placeholders: {{CODE:file:qualname or Lstart-end}} and
# {{FIG:file.png|caption|width}}.
# =========================================================================

CHAPTERS: list[tuple[str, str, str]] = []  # (part, id, html)


def chapter(part: str, cid: str, body: str) -> None:
    CHAPTERS.append((part, cid, body))


# ------------------------------------------------------------------ PART 0
chapter("Start here", "intro", r"""
<h1 id="intro" class="nobreak">0. How to use this guide</h1>
<p>This guide explains <b>everything</b> in the GP7 pick-and-place project, assuming you start
from zero in ROS 2, Gazebo and robot kinematics. It has three parts:</p>
<ol>
<li><b>Foundations</b> (chapters 1–7): the ideas you need: the Linux workspace, the bits of
Python the code uses, ROS 2, Gazebo, coordinate frames, forward and inverse kinematics,
trajectories and PID control. Each idea is explained with the GP7 as the example.</li>
<li><b>The project, file by file</b> (chapters 8–15): every source file, with the real code
next to a plain-language explanation. The code excerpts are copied out of your files when
this PDF is built, and the line numbers match the files.</li>
<li><b>Using it</b> (chapters 16–18): how to run and tune it, how to debug it, exercises to
practise with, and a glossary.</li>
</ol>
<p>If you only have one hour, read chapter 8 (the big picture) and chapter 12 (the task,
step by step); then come back to the foundations whenever a word is unclear. The glossary
at the end defines every technical word used.</p>

<h2>What the project does, in one paragraph</h2>
<p>A computer model of a <b>Yaskawa Motoman GP7</b> (a real 6-joint industrial arm that can lift
7&nbsp;kg) stands in a simulated room made by <b>Gazebo</b>. On the floor are two boxes: a large
blue <b>Box A</b> (30&nbsp;×&nbsp;30&nbsp;×&nbsp;20&nbsp;cm, centred at x&nbsp;=&nbsp;0.5&nbsp;m, y&nbsp;=&nbsp;0.5&nbsp;m)
and a small orange <b>Box B</b> (a 5&nbsp;cm cube). A Python program, using <b>ROS 2</b> to talk to the
simulator, works out how every joint must turn (<b>inverse kinematics</b>), plans smooth motions
(<b>trajectory generation</b>), and makes the robot pick Box B up, carry it upright, put it on
the centre of Box A, let go, and lift its arm away. A small window (the <b>GUI</b>) lets you place
Box B at a random reachable spot, start the task, and watch the joint angles live.</p>

{{FIG:task_3d.png|The gripper path the planner produces for the whole task (computed from the real code).|78%}}

<div class="callout idea"><b class="t">Reading the boxes in this guide</b>
Blue boxes explain an idea. <span style="color:#a06000">Orange</span> boxes warn about a trap.
Green boxes are things to try yourself. Purple boxes point at a detail of the code.</div>
""")

# ------------------------------------------------------------------ PART 1
chapter("Part 1 – Foundations", "linux", r"""
<h1 id="linux">1. The workspace, the terminal and building</h1>

<h2>1.1 Folder layout</h2>
<p>ROS 2 code lives in a <b>workspace</b>: a folder with a <code>src/</code> subfolder that holds one
or more <b>packages</b>. Yours is:</p>
<pre class="term">gp7_ws/                         <span class="c"># the workspace</span>
├── src/
│   └── gp7robot_description/   <span class="c"># the one package (all the code)</span>
│       ├── package.xml         <span class="c"># package name, dependencies (ROS reads this)</span>
│       ├── setup.py            <span class="c"># how Python installs it + program names</span>
│       ├── gp7robot_description/   <span class="c"># the Python modules</span>
│       │   ├── kinematics.py   <span class="c"># forward / inverse kinematics</span>
│       │   ├── trajectory.py   <span class="c"># smooth motion profiles</span>
│       │   ├── pick_place.py   <span class="c"># the task planner + ROS node</span>
│       │   ├── workspace.py    <span class="c"># "is this Box B position reachable?"</span>
│       │   ├── reset_world.py  <span class="c"># move Box B, send the arm home</span>
│       │   ├── gui.py          <span class="c"># the control window</span>
│       │   └── manual_joint.py <span class="c"># move one joint by hand</span>
│       ├── launch/             <span class="c"># start-up scripts (Gazebo, RViz)</span>
│       ├── urdf/GP7.urdf       <span class="c"># the robot's shape and joints (exported from SolidWorks)</span>
│       ├── meshes/*.STL        <span class="c"># 3D shapes of each link</span>
│       ├── worlds/*.sdf        <span class="c"># the Gazebo rooms (floor, Box A, Box B)</span>
│       └── docs/               <span class="c"># this guide, the report</span>
├── build/    <span class="c"># made by colcon - temporary build files</span>
├── install/  <span class="c"># made by colcon - the installed package (what ROS runs!)</span>
└── log/      <span class="c"># made by colcon - build logs</span></pre>

<h2>1.2 Building with colcon</h2>
<p><b>colcon</b> is the ROS 2 build tool. It reads each package in <code>src/</code> and copies or compiles
it into <code>install/</code>. For a Python package "building" mostly means copying the files and creating
small launcher programs. <b>ROS always runs the copy in <code>install/</code>, not your files in
<code>src/</code></b> (unless you build with <code>--symlink-install</code>). So after editing code you must build again.</p>
<pre class="term">cd ~/Documents/GitHub/YASCAWA_GP7_sim/gp7_ws
colcon build --packages-select gp7robot_description   <span class="c"># build just this package</span>
colcon build --symlink-install                        <span class="c"># (optional) link instead of copy:</span>
                                                      <span class="c"># Python edits then work without rebuilding</span></pre>

<h2>1.3 "Sourcing" – why <code>Package not found</code> happens</h2>
<p>A new terminal does not know about ROS or your workspace. <code>source</code> runs a script that sets
environment variables (such as <code>AMENT_PREFIX_PATH</code> and <code>PYTHONPATH</code>) so the <code>ros2</code>
command can find packages. You need two: the ROS installation (the <b>underlay</b>) and your
workspace (the <b>overlay</b>).</p>
<pre class="term">source /opt/ros/lyrical/setup.bash     <span class="c"># ROS 2 itself (often already in ~/.bashrc)</span>
source install/setup.bash              <span class="c"># your workspace - in EVERY new terminal</span></pre>
<div class="callout warn"><b class="t">The most common beginner error</b>
<code>Package 'gp7robot_description' not found</code> means the terminal was not sourced with
<code>install/setup.bash</code>. Add it to <code>~/.bashrc</code> to do it automatically.</div>

<h2>1.4 Programs ("executables") of this package</h2>
<p><code>setup.py</code> maps short program names to Python functions. <code>ros2 run &lt;package&gt; &lt;program&gt;</code>
starts one:</p>
{{CODE:setup.py:L46-53}}
<table><tr><th>Command</th><th>What it starts</th></tr>
<tr><td><code>ros2 run gp7robot_description gp7_pick_place</code></td><td>the planner – runs the task once (<code>pick_place.main</code>)</td></tr>
<tr><td><code>ros2 run gp7robot_description gp7_gui</code></td><td>the control window</td></tr>
<tr><td><code>ros2 run gp7robot_description gp7_reset_world</code></td><td>put Box B back, home the arm</td></tr>
<tr><td><code>ros2 run gp7robot_description gp7_move_joint Joint2 0.3</code></td><td>move one joint to 0.3 rad</td></tr>
<tr><td><code>ros2 launch gp7robot_description gazebo_sim.launch.py</code></td><td>Gazebo + robot + bridge (+ the task)</td></tr>
<tr><td><code>ros2 launch gp7robot_description display.launch.py</code></td><td>RViz viewer with joint sliders (no physics)</td></tr>
</table>
""")

chapter("Part 1 – Foundations", "python", r"""
<h1 id="python">2. The Python you need to read this code</h1>
<p>The project is written in Python 3. These are the language features it uses, each with a tiny example.</p>

<h3>Modules and imports</h3>
<p>Each <code>.py</code> file is a <b>module</b>. <code>from .kinematics import forward_kinematics</code> means "from the module
<code>kinematics.py</code> in this same package (the dot), take the function <code>forward_kinematics</code>".</p>

<h3>Functions, type hints and <code>from __future__ import annotations</code></h3>
<pre class="term">def horizontal_reach(position: Iterable[float]) -&gt; float:
    xyz = np.asarray(tuple(position), dtype=float)
    return float(math.hypot(float(xyz[0]), float(xyz[1])))</pre>
<p>The parts after <code>:</code> and <code>-&gt;</code> are <b>type hints</b>: notes saying what types are expected
(here: anything you can loop over that holds floats; returns a float). Python does not enforce them; they
are documentation. <code>np.ndarray | None</code> means "a NumPy array or nothing".</p>

<h3>Classes and <code>self</code></h3>
<pre class="term">class PickPlaceController(Node):          <span class="c"># a new class that extends ROS's Node class</span>
    def __init__(self) -&gt; None:            <span class="c"># runs when an object is created</span>
        super().__init__("gp7_pick_place_controller")   <span class="c"># run Node's own setup first</span>
        self.current_joint_positions = None   <span class="c"># data stored on the object</span>
    def run_demo(self) -&gt; None: ...        <span class="c"># a method; self = this object</span></pre>
<p>A <b>class</b> is a blueprint; an <b>object</b> is one thing built from it. <code>PickPlaceController(Node)</code>
<b>inherits</b> from ROS's <code>Node</code>, so it gets all of Node's abilities (publishing, subscribing,
parameters, clock) and adds its own. Methods whose names start with <code>_</code> are "private by
convention": helpers meant to be used only inside the class.</p>

<h3>Dataclasses</h3>
<pre class="term">@dataclass(frozen=True)
class TrajectorySample:
    time_from_start: float
    positions: np.ndarray
    velocities: np.ndarray</pre>
<p><code>@dataclass</code> writes the boring <code>__init__</code> for you: <code>TrajectorySample(0.5, q, v)</code> makes an
object with those three fields. <code>frozen=True</code> makes it read-only.</p>

<h3>NumPy</h3>
<p>NumPy gives fast arrays and linear algebra. Things used everywhere in this code:</p>
<table>
<tr><th>Code</th><th>Meaning</th></tr>
<tr><td><code>np.array([1.0, 2.0, 3.0])</code></td><td>a vector (1-D array)</td></tr>
<tr><td><code>np.eye(4)</code></td><td>4×4 identity matrix</td></tr>
<tr><td><code>A @ B</code></td><td>matrix multiplication</td></tr>
<tr><td><code>T[:3, :3]</code>, <code>T[:3, 3]</code></td><td>top-left 3×3 block (rotation), first 3 entries of column 3 (position)</td></tr>
<tr><td><code>M.T</code></td><td>transpose</td></tr>
<tr><td><code>np.linalg.norm(v)</code></td><td>length of a vector</td></tr>
<tr><td><code>np.linalg.solve(A, b)</code></td><td>solve A x = b</td></tr>
<tr><td><code>np.cross(a, b)</code></td><td>cross product</td></tr>
<tr><td><code>np.clip(q, lo, hi)</code></td><td>limit every value into [lo, hi]</td></tr>
<tr><td><code>np.gradient(y, t, axis=0)</code></td><td>numerical derivative dy/dt</td></tr>
<tr><td><code>np.interp(x, xp, fp)</code></td><td>linear interpolation in a table</td></tr>
<tr><td><code>np.deg2rad</code>, <code>np.rad2deg</code></td><td>angle unit conversion (1 rad ≈ 57.3°)</td></tr>
</table>

<h3>Exceptions</h3>
<p><code>raise ValueError("...")</code> stops the current function with an error message; <code>try: ... except
ValueError as error:</code> catches it. The planner raises <code>ValueError</code> whenever a plan is impossible
(target out of reach, IK fails, robot did not get there), and <code>main()</code> catches it and logs it.</p>

<h3>Threads, subprocesses and queues (used by the GUI)</h3>
<ul>
<li><code>subprocess.Popen([...])</code> starts another program (e.g. <code>ros2 run ...</code>) and keeps running.</li>
<li><code>threading.Thread(target=f).start()</code> runs <code>f</code> at the same time as the rest of the program.</li>
<li><code>queue.Queue</code> passes messages safely from a thread to the main program.</li>
</ul>
""")

chapter("Part 1 – Foundations", "ros", r"""
<h1 id="ros">3. ROS 2 from zero</h1>
<p><b>ROS 2</b> (Robot Operating System 2) is not an operating system. It is a set of libraries and tools
that let many small programs work together on a robot by sending each other messages. The version
installed here is <b>ROS 2 Lyrical</b> (it lives in <code>/opt/ros/lyrical</code>).</p>

<h2>3.1 Nodes</h2>
<p>A <b>node</b> is one program (or one object in a program) with one job. In this project:</p>
<table>
<tr><th>Node name</th><th>Made by</th><th>Job</th></tr>
<tr><td><code>gp7_pick_place_controller</code></td><td>pick_place.py</td><td>plans and commands the task</td></tr>
<tr><td><code>gp7_gui</code></td><td>gui.py</td><td>listens to joint states for the plots</td></tr>
<tr><td><code>gp7_reset_world</code></td><td>reset_world.py</td><td>homes the arm</td></tr>
<tr><td><code>gp7_manual_joint_commander</code></td><td>manual_joint.py</td><td>moves one joint</td></tr>
<tr><td><code>parameter_bridge</code></td><td>ros_gz_bridge (installed)</td><td>connects ROS and Gazebo</td></tr>
<tr><td><code>robot_state_publisher</code></td><td>installed</td><td>publishes the robot model + link positions (for RViz)</td></tr>
</table>

<h2>3.2 Topics, messages, publishers, subscribers</h2>
<p>A <b>topic</b> is a named channel, like a radio frequency. A node <b>publishes</b> messages on a topic;
any node that <b>subscribes</b> to it receives them. Publisher and subscriber do not need to know about
each other. Each topic carries one <b>message type</b>, a fixed structure of fields.</p>
{{FIG:ros_topics.png|Publish / subscribe: the two most important topics of this project.|88%}}
<table>
<tr><th>Topic</th><th>Message type</th><th>Direction</th><th>Content</th></tr>
<tr><td><code>/model/GP7Robot/joint_trajectory</code></td><td><code>trajectory_msgs/JointTrajectory</code></td><td>planner → Gazebo</td><td>a list of timed joint positions + velocities to follow</td></tr>
<tr><td><code>/joint_states</code></td><td><code>sensor_msgs/JointState</code></td><td>Gazebo → everyone</td><td>current angle and speed of each joint</td></tr>
<tr><td><code>/gp7/gripper/attach</code></td><td><code>std_msgs/Empty</code></td><td>planner → Gazebo</td><td>"glue Box B to the hand now"</td></tr>
<tr><td><code>/gp7/gripper/detach</code></td><td><code>std_msgs/Empty</code></td><td>planner → Gazebo</td><td>"let go of Box B"</td></tr>
<tr><td><code>/clock</code></td><td><code>rosgraph_msgs/Clock</code></td><td>Gazebo → everyone</td><td>the simulation time</td></tr>
</table>
<p>A <code>JointTrajectory</code> message looks like this (simplified):</p>
<pre class="term">header:      { stamp: &lt;time the trajectory starts&gt; }
joint_names: [joint1, Joint2, Joint3, Join4, Join5, Joint6]
points:
  - positions: [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]   velocities: [...]   time_from_start: 0.000 s
  - positions: [0.00, 0.00, 0.01, 0.00, 0.00, 0.00]   velocities: [...]   time_from_start: 0.005 s
  - ... one point every 5 ms ...</pre>
<p>In Python, creating a publisher and subscriber is one line each:</p>
{{CODE:pick_place.py:PickPlaceController.__init__}}
<p><code>create_subscription(type, topic, callback, 10)</code>: whenever a message arrives, ROS calls
<code>callback(message)</code>. The <code>10</code> is the <b>queue depth</b> (how many messages to keep if the
program is busy).</p>

<h2>3.3 Spinning: when do callbacks run?</h2>
<p>Callbacks do <b>not</b> run by magic. ROS only delivers messages when the program asks it to, by
<b>spinning</b>. <code>rclpy.spin(node)</code> loops forever; <code>rclpy.spin_once(node, timeout_sec=0.01)</code>
handles whatever has arrived (waiting at most 10 ms) and returns. This project uses <code>spin_once</code>
inside its own loops, so it can wait for the robot while still receiving <code>/joint_states</code>:</p>
{{CODE:pick_place.py:PickPlaceController._sleep}}

<h2>3.4 Parameters</h2>
<p>A <b>parameter</b> is a named setting of a node that can be changed when starting it, without editing
code. The node declares it with a default, then reads it:</p>
<pre class="term">self.declare_parameter("box_b_x", 0.35)                     <span class="c"># in __init__</span>
x = float(self.get_parameter("box_b_x").value)              <span class="c"># later</span>

ros2 run gp7robot_description gp7_pick_place --ros-args -p box_b_x:=0.6 -p box_b_y:=-0.3</pre>
<p>Chapter 16 lists every parameter of this project.</p>

<h2>3.5 Time: wall clock vs simulation clock</h2>
<p>Gazebo can run slower (or faster) than real time. If the planner waited "2 real seconds" while the
simulator only advanced 1 simulated second, it would give up too early. With the parameter
<code>use_sim_time:=true</code>, a node's clock follows the <code>/clock</code> topic published by Gazebo, so
<code>self.get_clock().now()</code> returns <b>simulated</b> time. The planner and the GUI use sim time;
<code>reset_world.py</code> uses the wall clock (<code>time.monotonic()</code>) for its short waits.</p>

<h2>3.6 Launch files</h2>
<p>A <b>launch file</b> is a Python script that starts many programs at once with the right settings.
<code>ros2 launch gp7robot_description gazebo_sim.launch.py run_demo:=false</code> runs the function
<code>generate_launch_description()</code> in that file, which returns the list of things to start.
<code>run_demo:=false</code> is a <b>launch argument</b>. Chapter 14 walks through it.</p>

<h2>3.7 Useful command-line tools</h2>
<pre class="term">ros2 node list                         <span class="c"># which nodes are running?</span>
ros2 topic list                        <span class="c"># which topics exist?</span>
ros2 topic echo /joint_states          <span class="c"># print the messages live (Ctrl+C to stop)</span>
ros2 topic hz /joint_states            <span class="c"># how often are they published?</span>
ros2 topic info -v /model/GP7Robot/joint_trajectory   <span class="c"># who publishes / subscribes?</span>
ros2 param list /gp7_pick_place_controller            <span class="c"># parameters of a running node</span>
ros2 interface show trajectory_msgs/msg/JointTrajectory   <span class="c"># fields of a message type</span>
rqt_graph                              <span class="c"># picture of nodes and topics</span></pre>
""")

chapter("Part 1 – Foundations", "gazebo", r"""
<h1 id="gazebo">4. Gazebo: the simulator</h1>
<p><b>Gazebo Sim</b> (command <code>gz sim</code>) is a physics simulator: it computes how bodies move under
gravity, motor torques and contacts, and draws them in 3D. It is a separate program from ROS with its own
message system ("gz topics"); the <b>bridge</b> connects the two.</p>

<h2>4.1 Worlds, models, links and joints</h2>
<ul>
<li>A <b>world</b> (an <code>.sdf</code> file) describes the whole scene: physics settings, light, ground, objects.</li>
<li>A <b>model</b> is one object (the robot, Box A, Box B). It has a <b>pose</b>: position x y z and orientation
roll pitch yaw.</li>
<li>A <b>link</b> is one rigid body of a model, with <b>visual</b> geometry (what you see), <b>collision</b> geometry
(what touches other things) and <b>inertial</b> data (mass, inertia).</li>
<li>A <b>joint</b> connects two links. A <b>revolute</b> joint rotates about one axis (all 6 GP7 joints); a
<b>fixed</b> joint glues them.</li>
<li>A <b>static</b> model never moves (the ground, Box A). Box B is not static: it falls, slides and can be carried.</li>
</ul>
{{CODE:world.sdf:L1-10}}
<p><code>max_step_size 0.001</code> means physics is computed every 1&nbsp;ms (1000 times per simulated second);
<code>real_time_factor 1.0</code> asks Gazebo to try to run at real speed. The <b>plugins</b> ("systems") add
features: physics, user commands (needed to teleport Box B), and the scene broadcaster (for the 3D view).</p>
{{CODE:world.sdf:L54-78}}

<h2>4.2 URDF vs SDF</h2>
<p>The robot comes from SolidWorks as a <b>URDF</b> (Unified Robot Description Format) file: the ROS format for
a robot's links, joints, meshes and limits. Gazebo's own format is <b>SDF</b>; Gazebo converts URDF to SDF when the
robot is spawned. A URDF joint looks like this:</p>
{{CODE:GP7.urdf:L87-102}}
<ul>
<li><code>origin xyz rpy</code>: where the joint sits relative to its parent link (metres; roll-pitch-yaw radians).</li>
<li><code>axis</code>: the rotation axis in the joint's own frame.</li>
<li><code>limit lower/upper</code>: allowed angle range in radians.</li>
</ul>
<div class="callout warn"><b class="t">The joint names have typos – and must keep them</b>
The SolidWorks export named the joints <code>joint1, Joint2, Joint3, Join4, Join5, Joint6</code>. Every message must use
exactly these names, so the code copies them as they are (<code>JOINT_NAMES</code> in kinematics.py).</div>

<h2>4.3 Plugins this project adds to the robot</h2>
<p>The launch file edits the URDF in memory before spawning it (chapter 14) and adds four Gazebo systems:</p>
<table>
<tr><th>System</th><th>What it does here</th></tr>
<tr><td><b>JointTrajectoryController</b></td><td>listens on <code>/model/GP7Robot/joint_trajectory</code>; for every joint runs a <b>PID</b>
controller that applies torque so the joint follows the trajectory points in time.</td></tr>
<tr><td><b>JointStatePublisher</b></td><td>publishes every joint's angle and speed (bridged to <code>/joint_states</code>).</td></tr>
<tr><td><b>DetachableJoint</b></td><td>the "gripper": on a message on <code>/gp7/gripper/attach</code> it creates a fixed joint between
<code>Link6</code> and Box B (keeping their current relative pose); on <code>/gp7/gripper/detach</code> it removes it.
There are no real fingers squeezing the box: this is the usual way to simulate grasping reliably.</td></tr>
<tr><td>self_collide</td><td>lets the robot's own links collide with each other.</td></tr>
</table>

<h2>4.4 The bridge</h2>
<p><code>ros_gz_bridge parameter_bridge</code> is given a list of <code>topic@ROS_type[gz_type</code> or <code>]gz_type</code>
strings. <code>[</code> means Gazebo → ROS, <code>]</code> means ROS → Gazebo:</p>
{{CODE:gazebo_sim.launch.py:L198-209}}
<p>The last line also <b>remaps</b> Gazebo's long joint-state topic name to the short <code>/joint_states</code>.</p>

<h2>4.5 Gazebo services: teleporting Box B</h2>
<p>A <b>service</b> is a request/answer call (unlike a topic, which is one-way). The world offers
<code>/world/gp7_pick_place/set_pose</code>; <code>reset_world.py</code> calls it with the <code>gz service</code> command to put Box B
anywhere instantly (chapter 13).</p>

<h2>4.6 RViz vs Gazebo</h2>
<p><b>RViz</b> only <i>displays</i> a robot from its joint angles; there is no physics, gravity or contact.
<code>display.launch.py</code> opens RViz with a slider window (<code>joint_state_publisher_gui</code>), which is a good way
to learn what each joint does. <b>Gazebo</b> <i>simulates</i>: motors, gravity, boxes falling.</p>
""")

chapter("Part 1 – Foundations", "frames", r"""
<h1 id="frames">5. Kinematics I: positions, frames and forward kinematics</h1>
<p><b>Kinematics</b> is the geometry of motion: where things are and how they move, ignoring the forces.
For a robot arm, two questions matter:</p>
<ul>
<li><b>Forward kinematics (FK)</b>: "the joints are at these angles; where is the hand?" – easy, one answer.</li>
<li><b>Inverse kinematics (IK)</b>: "I want the hand here, pointing this way; what joint angles do that?" – hard,
zero, one or many answers (chapter 6).</li>
</ul>

<h2>5.1 The world frame of this project</h2>
<p>All positions are in metres in the robot's <b>base frame</b>, which is the world frame: the origin is
at the bottom centre of the robot, <b>z</b> points up, <b>x</b> and <b>y</b> lie on the floor. Box A's centre is at
(0.5, 0.5); its top is at z = 0.20. Angles are in <b>radians</b> in the code (π rad = 180°).</p>

<h2>5.2 Joint space vs task space</h2>
<p>A robot's state can be described two ways:</p>
<ul>
<li><b>Joint space</b>: the 6 joint angles <b>q</b> = (q1 … q6). This is what motors understand.</li>
<li><b>Task (Cartesian) space</b>: the hand's position (x, y, z) and orientation. This is what the task is about.</li>
</ul>
<p>Moving in a straight line in joint space (all joints turning at proportional speeds) makes the hand
move along a curve. To move the hand in a straight line you must compute IK at many points along the line.
This project uses both: joint-space moves when nothing is held, straight lines near and with the box.</p>

<h2>5.3 Rotation matrices and homogeneous transforms</h2>
<p>A <b>coordinate frame</b> is an origin plus three perpendicular axes. Every link of the robot has one.
To describe frame B as seen from frame A you need:</p>
<ul>
<li>a <b>rotation matrix R</b> (3×3): its columns are B's x, y, z axes written in A's coordinates;</li>
<li>a <b>translation t</b> (3×1): where B's origin is, in A's coordinates.</li>
</ul>
<p>A point known in B is converted to A by rotating then shifting: <b>P<sub>A</sub> = R P<sub>B</sub> + t</b>.</p>
{{FIG:frames_2d.png|A point P described in frame B and in frame A.|55%}}
<p>Robotics packs R and t into one 4×4 <b>homogeneous transform</b>, so that chaining frames is just
multiplying matrices:</p>
<pre class="math">        ┌ R   t ┐              ┌ r11 r12 r13 tx ┐
   T =  │       │     e.g.  T = │ r21 r22 r23 ty │        T_AC = T_AB · T_BC
        └ 0   1 ┘              │ r31 r32 r33 tz │
                               └  0   0   0   1 ┘</pre>
<p>In the code: <code>T[:3, :3]</code> is R and <code>T[:3, 3]</code> is t. <code>pose(position, rotation)</code> in kinematics.py
builds such a matrix.</p>

<h2>5.4 Denavit–Hartenberg (DH) parameters</h2>
<p>A 6-joint arm is a chain: base → link 1 → link 2 → … → link 6. The DH convention describes each step
of the chain with just <b>four numbers</b>. Each joint i rotates about its own z-axis by the joint angle, and:</p>
<pre class="math">A_i = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)        with θ_i = θ_offset,i + q_i

  θ  (theta)  rotate about z      (this is where the joint angle q_i goes)
  d           slide along z
  a           slide along the new x   ("link length")
  α  (alpha)  twist about the new x   (angle between this joint axis and the next)</pre>
<p>Written out, one DH step is this matrix (exactly what <code>_dh_transform</code> returns):</p>
<pre class="math">       ┌ cosθ  -sinθ·cosα   sinθ·sinα   a·cosθ ┐
 A_i = │ sinθ   cosθ·cosα  -cosθ·sinα   a·sinθ │
       │  0       sinα        cosα        d    │
       └  0        0           0          1    ┘</pre>
<p>The GP7 table was derived from the joint axes and origins in <code>GP7.urdf</code>. The large θ offsets
(e.g. −112.65°) exist because the CAD export's "zero" pose is not the textbook DH zero; the offsets make
q = 0 mean the same pose as in the URDF.</p>
{{CODE:kinematics.py:L34-41}}
<table>
<tr><th>Joint</th><th>Yaskawa name</th><th>Moves</th><th>d (m)</th><th>a (m)</th><th>α</th></tr>
<tr><td>1</td><td>S (swing)</td><td>whole arm left/right about the vertical</td><td>0.330</td><td>0.040</td><td>−90°</td></tr>
<tr><td>2</td><td>L (lower arm)</td><td>lower arm forward/back</td><td>0</td><td>0.445</td><td>0°</td></tr>
<tr><td>3</td><td>U (upper arm)</td><td>upper arm up/down</td><td>0</td><td>0.040</td><td>−90°</td></tr>
<tr><td>4</td><td>R (wrist roll)</td><td>twists the forearm</td><td>0.440</td><td>0</td><td>+90°</td></tr>
<tr><td>5</td><td>B (wrist bend)</td><td>bends the wrist</td><td>0</td><td>0</td><td>−90°</td></tr>
<tr><td>6</td><td>T (tool turn)</td><td>spins the flange</td><td>−0.059</td><td>0</td><td>0°</td></tr>
</table>
<p>So the base column is 0.33 m tall, the lower arm 0.445 m, the forearm 0.44 m: roughly 0.93 m of
horizontal reach, which is where <code>MAX_HORIZONTAL_REACH = 0.927</code> comes from.</p>

<h2>5.5 Forward kinematics in code</h2>
<p>FK is just "multiply the six DH matrices in order", then one fixed rotation so that the result matches
the CAD <code>Link6</code> frame used by Gazebo:</p>
{{CODE:kinematics.py:_dh_transform}}
{{CODE:kinematics.py:forward_kinematics}}
{{FIG:arm_poses.png|Forward kinematics of two poses, drawn from the real DH chain. Left: all joints at 0. Right: the "upright reference" pose, flange straight down above Box B.}}

<h2>5.6 The gripper point and "upright"</h2>
<p>The gripper is mounted 75 mm (<code>GRIPPER_LINK_OFFSET</code>) below Link6, and it is mounted so that in the
<code>UPRIGHT_REFERENCE_CONFIGURATION</code> pose it points straight down. The planner keeps Link6's orientation
<b>exactly equal</b> to its orientation in that pose (<code>UPRIGHT_LINK6_ROTATION</code>) whenever the box is near
or held. That is what "keeping the box upright" means mathematically: the target of every IK solve has
the same rotation matrix, only the position changes.</p>

<h2>5.7 Joint limits</h2>
<p>Every joint has a minimum and maximum angle (<code>JOINT_LIMITS</code>, radians):</p>
<table><tr><th>Joint</th><th>1</th><th>2</th><th>3</th><th>4</th><th>5</th><th>6</th></tr>
<tr><td>min (°)</td><td>−101.2</td><td>−79.1</td><td>−4.8</td><td>−190.0</td><td>−137.7</td><td>−138.8</td></tr>
<tr><td>max (°)</td><td>238.8</td><td>105.9</td><td>255.2</td><td>190.0</td><td>132.3</td><td>221.0</td></tr></table>
<p>The comments in kinematics.py explain two corrections made to the SolidWorks export (joint 3's range was
mirrored, joint 4's was far too small).</p>
""")

chapter("Part 1 – Foundations", "ik", r"""
<h1 id="ik">6. Kinematics II: inverse kinematics and the Jacobian</h1>

<h2>6.1 Why IK is hard</h2>
<p>FK is a formula. IK means solving that formula backwards: 6 unknown angles, 6 equations (3 for position,
3 for orientation), all full of sines and cosines. There may be <b>no</b> solution (target too far), or
<b>several</b>: a 6-joint arm typically has up to 8 ways ("branches") to reach the same pose: elbow up or
down, shoulder left or right, wrist flipped or not.</p>
{{FIG:two_link_ik.png|Even a 2-joint arm has two solutions for the same target.|55%}}
<p>Some robots have closed-form IK formulas. This project uses a <b>numerical</b> method instead: start from a
guess (the <b>seed</b>), look at the error, nudge the joints to reduce it, repeat. A numerical method
returns the solution <i>nearest the seed</i>, which is useful: seeding with the current joint angles keeps
the arm on the same branch, so it does not suddenly flip.</p>

<h2>6.2 The Jacobian</h2>
<p>The <b>Jacobian</b> J(q) is a 6×6 matrix that says how the hand moves when the joints move a little:</p>
<pre class="math">   ┌ v ┐                  v = hand linear velocity (3)
   │   │ = J(q) · q̇        ω = hand angular velocity (3)
   └ ω ┘                  q̇ = joint velocities (6)</pre>
<p>For a revolute joint i with axis z<sub>i−1</sub> passing through point p<sub>i−1</sub>, column i is:</p>
<pre class="math">   J_v,i = z_(i-1) × (p_hand − p_(i-1))      (turning about an axis moves the hand sideways)
   J_ω,i = z_(i-1)                           (and rotates it about that axis)</pre>
<p>That is exactly what the code computes: walk along the chain, remember each joint's axis and origin, then
fill in the columns.</p>
{{CODE:kinematics.py:geometric_jacobian}}

<h2>6.3 Orientation error as a rotation vector</h2>
<p>Position error is easy: target position − current position. For orientation the code computes the rotation
that takes the current orientation to the target, <code>R_err = R_target · R_currentᵀ</code>, and converts it to a
<b>rotation vector</b> (axis × angle, 3 numbers) with <code>_rotation_vector</code>. The 6-number error
e = (position error, rotation vector) is what IK drives to zero.</p>

<h2>6.4 Damped least squares (the IK algorithm used)</h2>
<p>If the hand must move by e, we want the joint change Δq with J Δq ≈ e. J cannot simply be inverted: near
a <b>singularity</b> (e.g. arm fully stretched, or two wrist axes lined up) J loses rank and the inverse
explodes into huge joint jumps. <b>Damped least squares</b> (also called Levenberg–Marquardt) adds a small
"damping" λ that trades a little accuracy for stability:</p>
<pre class="math">   Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e</pre>
<p>Big λ → small, safe steps; small λ → fast, accurate steps. The code adapts λ every iteration: if a step
made the error smaller it is accepted and λ shrinks (× 0.75, not below 0.008); if not, the step is
rejected and λ grows (× 2, up to 0.4). Steps are also limited to 0.2 rad per joint and clipped to the
joint limits. It stops when position error ≤ tolerance and orientation error ≤ tolerance (the planner
asks for 0.01 mm and 0.006°), and raises <code>ValueError</code> if 250 iterations are not enough.</p>
{{CODE:kinematics.py:inverse_kinematics}}

<h2>6.5 How the planner calls IK</h2>
{{CODE:pick_place.py:PickPlaceController._solve}}
<ul>
<li>First a cheap check: is the target within 0.927 m horizontally? If not, fail with a clear message.</li>
<li>The target pose always uses <code>UPRIGHT_LINK6_ROTATION</code> (box upright).</li>
<li>Tight tolerances. The comment explains why: with loose tolerances, IK called with a seed that is already
"close enough" returns the seed unchanged, so a densely sampled straight line becomes a staircase of
stops and jumps.</li>
</ul>

<h2>6.6 Workspace and reachability</h2>
<p>The <b>workspace</b> is the set of positions the hand can reach. With a fixed orientation (straight down) it
is smaller than the full reach: near the base the arm cannot fold tightly enough, and far away the wrist cannot
point down. Behind the robot (negative x) the planner's seeded IK often lands on a branch that cannot hold the
orientation. <code>workspace.py</code> tests each candidate Box B position by running the real planning steps
(chapter 11). The map below was computed that way:</p>
{{FIG:reachability.png|Box B start positions on a 6 cm grid: green = the whole plan works, red = it fails. Circles: 0.30 m and 0.877 m.|62%}}
""")

chapter("Part 1 – Foundations", "traj", r"""
<h1 id="traj">7. Trajectories, smooth motion and PID control</h1>

<h2>7.1 Path vs trajectory</h2>
<p>A <b>path</b> is a shape in space (e.g. "straight line from A to B"). A <b>trajectory</b> is a path
<i>plus timing</i>: where to be at every instant. Good timing matters as much as a good path: a robot that
starts or stops abruptly shakes, overshoots, and can throw the box.</p>
<ul>
<li><b>position</b> x(t); <b>velocity</b> v = dx/dt; <b>acceleration</b> a = dv/dt; <b>jerk</b> j = da/dt.</li>
<li>A sudden change in velocity means infinite acceleration (impossible). A sudden change in acceleration
means infinite jerk: it makes the arm vibrate.</li>
</ul>

<h2>7.2 Trapezoid vs S-curve</h2>
<p>The simplest profile is the <b>trapezoid</b>: accelerate at a constant rate, cruise, decelerate. Its
acceleration jumps instantly between 0 and ±a<sub>max</sub> (infinite jerk at four points). An <b>S-curve</b>
ramps acceleration smoothly. This project shapes each acceleration ramp as a <b>sin² pulse</b>, so acceleration
and jerk are continuous and start and end at zero.</p>
{{FIG:scurve.png|scurve_segment: position, velocity and acceleration of a normalised move (blue) vs a trapezoid (grey). Vertical lines: the thirds.}}
<pre class="math">acceleration during a ramp of length Ta:   a(t) = A · sin²(π t / Ta)
velocity reached at the end of the ramp:   v = A · Ta / 2            ⇒  A = 2 v / Ta</pre>

<h2>7.3 <code>scurve_segment</code>: synchronised joint moves</h2>
<p>For joint-space moves, all 6 joints share one normalised progress value s(t) going 0 → 1, so they start and
stop together and the joint-space path stays a straight line. The move is split into thirds (accelerate,
cruise, decelerate). The duration T is the smallest value that respects every joint's speed and
acceleration limits:</p>
<pre class="math">T = max over joints of  max( 1.5·|Δq| / v_max ,  √(9·|Δq| / a_max) )</pre>
{{CODE:trajectory.py:scurve_segment}}

<h2>7.4 <code>path_speed_profile</code>: tool speed along a Cartesian path</h2>
<p>For straight-line moves of the tool, the thirds rule wastes time on long paths (it would spend two thirds
of the move speeding up and slowing down). <code>path_speed_profile</code> uses the same sin² ramps, but the
ramp time comes from the acceleration limit, Ta = 2v/a (0.25 m/s and 0.40 m/s² → 1.25 s), and the tool
cruises at full speed for the rest. If the path is too short to reach full speed, the peak speed is lowered to
√(a·L/2).</p>
{{FIG:profiles_compare.png|Same 1.1 m path, same limits: the old thirds profile (grey) vs path_speed_profile (blue).|80%}}

<h2>7.5 Blending: not stopping at corners</h2>
<p>When two moves are chained, stopping at the corner wastes time and looks jerky. Two techniques are used:</p>
<ul>
<li><b>Superposition</b> (<code>superpose</code>, joint space): start the next move while the previous one is
still decelerating, and add their displacements. Used for "lift the arm, then rotate joint 1": the rotation
starts when the lift is 2/3 done (<code>BLEND_START_FRACTION</code>).</li>
<li><b>Corner rounding</b> (<code>rounded_polyline</code>, Cartesian): replace each sharp corner of the path by a
circular arc tangent to both straight legs, then run <i>one</i> speed profile over the whole length. The tool
keeps a constant speed through the corner.</li>
</ul>
{{FIG:corner_blend.png|The carry path: up from Box B, across, down onto Box A. Grey: the waypoints. Red: the planned path.|52%}}

<h2>7.6 Sampling and why it is 5 ms</h2>
<p>A trajectory message is a list of points; the code samples the profile every <code>SAMPLE_PERIOD = 0.005</code> s.
The comment in pick_place.py explains why it is so dense: Gazebo's JointTrajectoryController does not
interpolate between points; its target <i>steps</i> from one point to the next. The PID's D term turns
each step into a torque kick. With a point every 5 physics steps, the steps are tiny and the motion is smooth.</p>

<h2>7.7 PID control: how Gazebo makes the joints follow</h2>
<p>A motor does not know how to "go to 30°"; it can only apply torque. A <b>PID controller</b> computes the
torque from the error e = target − actual:</p>
<pre class="math">torque = Kp·e  +  Ki·∫e dt  +  Kd·de/dt
         ─┬──     ───┬────     ───┬───
          │          │            └ D: brakes when the error closes fast  → less overshoot
          │          └ I: grows while an error persists      → removes steady sag (gravity)
          └ P: pushes harder the further away it is</pre>
{{FIG:pid.png|A toy joint with gravity following a step: P alone oscillates, PD sags under gravity, PID reaches the target.|78%}}
<p>The gains used for the GP7 are set in the launch file. Big joints (1–3) carry the whole arm and get large
gains and torque limits; the wrist gets small ones. Only joints fighting gravity (2, 3, 5) get an I term;
joint 1 rotates about a vertical axis, so gravity does not pull it and an I term would only cause overshoot.</p>
<table><tr><th>Joint</th><th>1</th><th>2</th><th>3</th><th>4</th><th>5</th><th>6</th></tr>
<tr><td>Kp</td><td>180</td><td>180</td><td>150</td><td>35</td><td>25</td><td>15</td></tr>
<tr><td>Ki</td><td>0</td><td>120</td><td>100</td><td>0</td><td>8</td><td>0</td></tr>
<tr><td>Kd</td><td>28</td><td>28</td><td>24</td><td>7</td><td>5</td><td>3</td></tr>
<tr><td>torque limit (N·m)</td><td>260</td><td>260</td><td>220</td><td>80</td><td>55</td><td>35</td></tr></table>
<p>Because a PID always lags a little, the planner does not assume the robot arrived when the trajectory
time is over: it watches <code>/joint_states</code> until every joint is within 2° of the goal for 0.25 s (chapter 12).</p>
""")

# ------------------------------------------------------------------ PART 2
chapter("Part 2 – The project, file by file", "arch", r"""
<h1 id="arch">8. The big picture</h1>
{{FIG:architecture.png|Who talks to whom. Boxes are programs; arrows are ROS topics or process starts.}}
<h2>8.1 What happens when you use the GUI</h2>
<ol>
<li><b>Terminal 1</b>: <code>ros2 launch gp7robot_description gazebo_sim.launch.py run_demo:=false</code> starts Gazebo with the
world, the bridge and robot_state_publisher; after 5 s it spawns the robot. With <code>run_demo:=false</code> it
does <i>not</i> start the planner.</li>
<li><b>Terminal 2</b>: <code>ros2 run gp7robot_description gp7_gui</code> opens the window. It subscribes to
<code>/joint_states</code> and draws plots.</li>
<li><b>Place Box B at random</b>: the GUI calls <code>random_box_b()</code> (workspace.py) in a background thread.
It tries random positions until the full plan works for one, writes it into the x/y fields, and runs
<code>gp7_reset_world</code> with that x/y: Box B is teleported, the gripper is released, and the arm goes home.</li>
<li><b>Start task</b>: the GUI runs <code>gp7_pick_place</code> with <code>box_b_x/box_b_y</code> parameters. The planner
waits for the bridge, plans each stage, publishes trajectories, watches <code>/joint_states</code> until each stage
is reached, sends attach/detach, and exits when done. Its log appears in the GUI.</li>
</ol>

<h2>8.2 Which file does what</h2>
<table>
<tr><th>File</th><th>Lines</th><th>Role</th><th>Uses ROS?</th></tr>
<tr><td>kinematics.py</td><td>~270</td><td>DH table, FK, Jacobian, IK, joint limits</td><td>no (pure maths)</td></tr>
<tr><td>trajectory.py</td><td>~220</td><td>speed profiles, blending, building the ROS message</td><td>only the message types</td></tr>
<tr><td>pick_place.py</td><td>~550</td><td>path planning + the task + executing on Gazebo</td><td>yes (a node)</td></tr>
<tr><td>workspace.py</td><td>~120</td><td>offline "is this Box B position OK?" + random placement</td><td>no</td></tr>
<tr><td>reset_world.py</td><td>~150</td><td>detach, teleport Box B, home the arm</td><td>yes</td></tr>
<tr><td>gui.py</td><td>~430</td><td>window, buttons, live plots</td><td>yes</td></tr>
<tr><td>manual_joint.py</td><td>~105</td><td>command one joint from the terminal</td><td>yes</td></tr>
<tr><td>launch/gazebo_sim.launch.py</td><td>~260</td><td>starts everything, adds gripper and plugins to the robot</td><td>launch</td></tr>
</table>
<div class="callout idea"><b class="t">A good design idea to notice</b>
The maths (kinematics.py, trajectory.py) does not depend on ROS or Gazebo. That is why workspace.py can test a
plan in a second, and why the figures in this guide could be computed without running the simulator.</div>
""")

chapter("Part 2 – The project, file by file", "kin", r"""
<h1 id="kin">9. kinematics.py</h1>
<p>Pure mathematics: no ROS. Everything about the robot's geometry lives here.</p>
<table>
<tr><th>Name</th><th>What it is</th></tr>
<tr><td><code>JOINT_NAMES</code></td><td>the 6 joint names exactly as in the URDF</td></tr>
<tr><td><code>DHRow</code>, <code>DH_TABLE</code></td><td>the DH parameters (chapter 5.4)</td></tr>
<tr><td><code>JOINT_LIMITS</code></td><td>6×2 array [min, max] in radians</td></tr>
<tr><td><code>MAX_JOINT_SPEEDS</code></td><td>datasheet maximum speeds (375, 315, 410, 550, 550, 1000 °/s)</td></tr>
<tr><td><code>ACCELERATION_LIMITS</code></td><td>conservative accelerations used for planning (rad/s²)</td></tr>
<tr><td><code>UPRIGHT_REFERENCE_CONFIGURATION</code></td><td>the pose that defines "gripper straight down"; also the IK seed</td></tr>
<tr><td><code>DH_TO_LINK6_ROTATION</code></td><td>fixed rotation from the last DH frame to the CAD Link6 frame</td></tr>
<tr><td><code>_dh_transform(row, angle)</code></td><td>one 4×4 DH matrix</td></tr>
<tr><td><code>forward_kinematics(q)</code></td><td>4×4 pose of Link6 for joint angles q</td></tr>
<tr><td><code>geometric_jacobian(q)</code></td><td>6×6 Jacobian</td></tr>
<tr><td><code>_rotation_vector(R)</code></td><td>rotation matrix → axis·angle vector (with a special case near 180°)</td></tr>
<tr><td><code>inverse_kinematics(target, seed, ...)</code></td><td>damped least squares IK</td></tr>
<tr><td><code>pose(position, rotation)</code></td><td>build a 4×4 matrix</td></tr>
<tr><td><code>horizontal_reach(p)</code></td><td>√(x² + y²)</td></tr>
</table>
{{CODE:kinematics.py:L63-84}}
<div class="callout try"><b class="t">Try it in Python</b>
<pre class="term">source install/setup.bash &amp;&amp; python3
&gt;&gt;&gt; import numpy as np
&gt;&gt;&gt; from gp7robot_description.kinematics import *
&gt;&gt;&gt; T = forward_kinematics(np.zeros(6)); print(T.round(3))     <span class="c"># where is Link6 at home?</span>
&gt;&gt;&gt; q = inverse_kinematics(T, seed=np.full(6, 0.1)); print(q.round(4))   <span class="c"># back to ~zeros</span>
&gt;&gt;&gt; forward_kinematics(UPRIGHT_REFERENCE_CONFIGURATION)[:3, 3]  <span class="c"># flange above Box B</span></pre></div>
""")

chapter("Part 2 – The project, file by file", "trajfile", r"""
<h1 id="trajfile">10. trajectory.py</h1>
<p>Turns "go from here to there" into a list of timed samples, and packs samples into a ROS message.</p>
<h3>TrajectorySample</h3>
{{CODE:trajectory.py:TrajectorySample}}
<h3>path_speed_profile (Cartesian tool speed)</h3>
<p>Returns samples whose "position" is the distance travelled along the path (metres) and whose "velocity"
is the tool speed. The <code>profile(t)</code> function has three cases (ramp up, cruise, ramp down);
ramp down is computed by mirroring ramp up: <code>length − s(duration − t)</code>.</p>
{{CODE:trajectory.py:path_speed_profile}}
<h3>superpose (blending joint moves)</h3>
<p>Each segment's displacement (relative to its own start) is added on top of the previous ones, starting
at its start time. Because every segment is a smooth rest-to-rest S-curve, their sum is smooth too.</p>
{{CODE:trajectory.py:superpose}}
<h3>joint_trajectory_message</h3>
<p>Copies each sample into a <code>JointTrajectoryPoint</code>; <code>time_from_start</code> is split into whole
seconds and nanoseconds, which is how ROS stores durations.</p>
{{CODE:trajectory.py:joint_trajectory_message}}
""")

chapter("Part 2 – The project, file by file", "pp", r"""
<h1 id="pp">11. pick_place.py, part 1: planning straight-line paths</h1>
<p>This is the heart of the project. It defines the constants, the corner-rounding geometry, and the class
<code>PickPlaceController</code>, a ROS node that plans and executes the task.</p>
<h2>11.1 Constants</h2>
{{CODE:pick_place.py:L45-78}}
<h2>11.2 rounded_polyline: the geometry of a path with rounded corners</h2>
<p>Input: a list of waypoints. Output: the path sampled every 1 mm, and the distance along the path at each
sample. For every inner corner it computes the turn angle between the incoming direction d0 and the outgoing
direction d1, and the <b>trim</b> distance r·tan(turn/2), which is how far before and after the corner the arc
starts and ends. The trim is capped at 45 % of each neighbouring leg, so the start and end of the path stay
straight (the last part of the descent onto Box A is purely vertical). The arc is drawn around a centre
point offset from the trimmed line end along the in-plane normal.</p>
{{CODE:pick_place.py:rounded_polyline}}
<h2>11.3 _cartesian_path: from geometry to joint trajectory</h2>
<ol>
<li>Build the rounded geometry.</li>
<li>Run one <code>path_speed_profile</code> over its length: distance travelled at each 5 ms step.</li>
<li>Look up the 3-D point at each distance (<code>np.interp</code> on the arc-length table).</li>
<li>Solve IK at every point, seeded with the previous solution. If any joint jumps more than 5°
(<code>MAX_IK_STEP</code>) between two samples, IK switched branch: abort, because the box would swing.</li>
<li>Joint velocities = numerical derivative of the joint angles.</li>
<li>If any joint would exceed its speed limit (datasheet speed × <code>speed_scale</code>), slow the whole profile
down by that ratio (+5 %) and try again (up to 6 times).</li>
</ol>
{{CODE:pick_place.py:PickPlaceController._cartesian_path}}
<div class="callout code-note"><b class="t">Why IK at every sample?</b>
Interpolating joint angles between two IK solutions would make the hand move along a curve and tilt the
box. Solving IK at each 5 ms point forces the hand onto the straight line with the exact upright orientation.
A 12 s path is ~2400 IK solves; each starts from the previous answer, so it converges in a few iterations.</div>
<h2>11.4 _execute: sending a trajectory and waiting for the robot</h2>
<p>Publishes the trajectory, then spins until the robot is really there. "There" means the first five joints
within 2° (joint 6 within 15°, since it only spins the upright box about the vertical) for 0.25 s of sim time.
If the deadline (trajectory duration + 3 s) passes, a small error (≤ 8°) is accepted with a warning; a larger
one raises an error, which stops the task rather than letting it continue from the wrong place.</p>
{{CODE:pick_place.py:PickPlaceController._execute}}
""")

chapter("Part 2 – The project, file by file", "task", r"""
<h1 id="task">12. pick_place.py, part 2: the task, step by step</h1>
<p><code>run_demo()</code> is the whole task. Below, every step with the numbers for the default Box B position
(0.35, 0.25). "Flange" means the Link6 origin; it is 75 mm above the box centre while the box is held.</p>
{{CODE:pick_place.py:PickPlaceController.run_demo}}
<h2>12.1 The numbers</h2>
<table>
<tr><th>Point</th><th>Formula</th><th>Flange position (m)</th></tr>
<tr><td>Box B centre</td><td>parameters</td><td>(0.35, 0.25, 0.025)</td></tr>
<tr><td>pick_pose</td><td>Box B + 0.075 up</td><td>(0.35, 0.25, 0.100)</td></tr>
<tr><td>clear_pose</td><td>Box B + 0.55 + 0.075 up</td><td>(0.35, 0.25, 0.650)</td></tr>
<tr><td>destination (Box B centre on A)</td><td>Box A top 0.20 + 0.025</td><td>(0.50, 0.50, 0.225) – box centre</td></tr>
<tr><td>above_box_a</td><td>destination x,y at clear height</td><td>(0.50, 0.50, 0.650)</td></tr>
<tr><td>release_pose</td><td>destination + 0.075 + 0.005</td><td>(0.50, 0.50, 0.305)</td></tr>
<tr><td>retreat_pose</td><td>release + retreat_height 0.15</td><td>(0.50, 0.50, 0.455)</td></tr>
</table>
<h2>12.2 The stages</h2>
<ol>
<li><b>Wait for the bridge</b> (<code>_wait_for_bridge</code>): until someone subscribes to the trajectory topic and a
complete <code>/joint_states</code> has arrived (max 15 s). Then send "detach" 3 times so nothing is attached.</li>
<li><b>Plan before moving</b>: IK for clear_pose seeded with the upright reference pose, then check the
descent is solvable. If not, the task stops before the robot moves at all.</li>
<li><b>Lift, then rotate joint 1</b> (joint space): first all joints except joint 1 move into the arm shape of
q_clear (the arm folds up, high above Box A), then joint 1 swings round to face Box B. The two moves are
superposed (the rotation starts at 2/3 of the lift).</li>
<li><b>Lower to Box B</b>: straight vertical line, clear_pose → pick_pose, gripper upright.</li>
<li><b>Grab</b>: "attach" sent 3 times, 0.25 s apart (in case one message is missed).</li>
<li><b>Carry</b>: one rounded path pick → clear → above_box_a → release: up, across, down, at a constant 0.25 m/s
through the corners. The box is released 5 mm above Box A so the arm never presses it into Box A.</li>
<li><b>Release</b>: "detach" 3 times, wait 0.5 s for the box to settle on Box A.</li>
<li><b>Lift away</b>: straight up by <code>retreat_height</code> (0.15 m). Done; the program exits.</li>
</ol>
{{FIG:task_timeline.png|The whole task computed offline: joint angles (top), gripper height and speed (bottom). Shaded = moving, gaps = gripper actions and settling (approximate).}}
<div class="callout idea"><b class="t">Why joint space first, Cartesian later?</b>
Far from the box, the shape of the path does not matter, and joint-space moves are fast, simple and cannot fail
IK halfway. Near the box and while holding it, the hand must go exactly straight down/up and stay upright, so
Cartesian lines are used there. Notice in the figure that the joint-space lift takes the gripper very high (about
1.3 m): joint 3 swings by about 205° to fold the arm over. That is the price of going from the home pose to
the "reach over" branch.</div>
""")

chapter("Part 2 – The project, file by file", "ws", r"""
<h1 id="ws">13. workspace.py and reset_world.py</h1>
<h2>13.1 workspace.py: checking a Box B position without Gazebo</h2>
<p>The GUI needs to know if a random position will work <i>before</i> moving the box there. Instead of writing
a second, slightly different planner (which would drift out of sync), workspace.py <b>borrows the planner's own
methods</b>. <code>_PlanChecker</code> is a tiny class that has the same <code>_solve</code> and <code>_cartesian_path</code>
methods as <code>PickPlaceController</code> (Python lets you copy a function into another class) and a fake
<code>get_parameter</code> that returns the defaults. No ROS node is created.</p>
{{CODE:workspace.py:L28-64}}
{{CODE:workspace.py:check_box_b}}
<p><code>random_box_b</code> picks points uniformly over the ring between 0.30 m and 0.877 m (taking the square root
of a uniform number makes the points uniform by <i>area</i>, not crowded at the centre), skips points near Box A,
and returns the first one that passes. A passing check takes about 1.5–3 s (thousands of IK solves); a failing
one usually fails in the first IK solve, in milliseconds.</p>
{{CODE:workspace.py:random_box_b}}
<h2>13.2 reset_world.py</h2>
<ol>
<li>Wait (10 s max) for the bridge and a joint state.</li>
<li>Publish "detach" so the gripper is not holding Box B.</li>
<li>Teleport Box B by calling the Gazebo service <code>/world/gp7_pick_place/set_pose</code> through the <code>gz</code>
command line tool:</li>
</ol>
{{CODE:reset_world.py:ResetWorldNode._reset_box_b}}
<ol start="4"><li>Drive all joints back to 0 with an S-curve at 20 % speed, then wait for the duration.</li></ol>
""")

chapter("Part 2 – The project, file by file", "gui", r"""
<h1 id="gui">14. The GUI (gui.py) and the launch file</h1>
<h2>14.1 How a Tk GUI and ROS share one program</h2>
<p>Tkinter (Python's built-in GUI toolkit) runs its own endless loop (<code>root.mainloop()</code>) that handles
clicks and redraws. ROS also needs spinning. The GUI solves this with a timer: every 100 ms Tk calls
<code>refresh()</code>, which spins ROS until no new messages are waiting, prints new log lines, and redraws the plots.</p>
{{CODE:gui.py:Gp7Gui.refresh}}
<h2>14.2 Collecting the data</h2>
<p><code>JointMonitor</code> is a node that keeps the last 30 s of joint angles, joint speeds, gripper position (via FK)
and gripper speed, at most 50 samples per second. If simulated time jumps backwards, Gazebo was restarted, so it
clears the buffers.</p>
{{CODE:gui.py:JointMonitor._on_joint_state}}
<h2>14.3 Running other programs from the buttons</h2>
<p><code>ProcessRunner</code> starts one <code>ros2 run ...</code> at a time with <code>subprocess.Popen</code>, reads its output line by
line in a background thread, and puts each line in a queue that <code>refresh()</code> prints. <b>Stop</b> sends
<code>SIGINT</code> (the same as pressing Ctrl+C).</p>
{{CODE:gui.py:ProcessRunner}}
<h2>14.4 The random placement button</h2>
<p>The search can take a few seconds, so it runs in a thread. Tk widgets may only be touched from the main
thread, so the thread hands the result back with <code>root.after(0, ...)</code>, which schedules a call on the
main thread.</p>
{{CODE:gui.py:Gp7Gui.place_random_box_b}}
{{CODE:gui.py:Gp7Gui._random_worker}}
{{CODE:gui.py:Gp7Gui._random_done}}
<h2>14.5 gazebo_sim.launch.py</h2>
<p>Besides starting the programs, the launch file <b>edits the robot description in memory</b>:</p>
<ul>
<li>removes the <code>ros2_control</code> and <code>gazebo</code> tags from the SolidWorks URDF (not compatible);</li>
<li>adds a <code>world</code> link and a fixed joint, bolting the robot base to the ground;</li>
<li>adds the gripper link (mounted 75 mm below Link6, rotated so it points down in the upright pose) and two
finger links (for looks);</li>
<li>adds the Gazebo systems with PID gains (chapter 7.7) and the DetachableJoint gripper.</li>
</ul>
{{CODE:gazebo_sim.launch.py:generate_launch_description}}
<p>Start order: Gazebo, bridge and robot_state_publisher immediately; the robot is spawned after 5 s (Gazebo
must be ready); the planner after 10 s, only if <code>run_demo</code> is true.</p>
<h2>14.6 manual_joint.py</h2>
<p>Sends a trajectory with a single point: "be at this angle at time <code>duration</code>"; the other joints get their
current angle so they hold still. It refuses moves larger than <code>--max-step</code> (0.5 rad) unless you raise it.</p>
<pre class="term">ros2 run gp7robot_description gp7_move_joint Joint2 0.3 --duration 2</pre>
""")

chapter("Part 2 – The project, file by file", "sequence", r"""
<h1 id="sequence">15. One message's journey</h1>
<p>To tie everything together, follow one command from Python to a moving joint and back:</p>
<ol>
<li><code>_cartesian_path</code> returns ~2400 <code>TrajectorySample</code>s (12 s at 5 ms).</li>
<li><code>joint_trajectory_message</code> packs them into one <code>JointTrajectory</code> with the header stamp = now
(sim time) and the 6 joint names.</li>
<li><code>trajectory_publisher.publish(message)</code> sends it on <code>/model/GP7Robot/joint_trajectory</code> (ROS 2's
transport is DDS).</li>
<li>The bridge receives it and republishes it as a <code>gz.msgs.JointTrajectory</code> on the Gazebo topic of the same name.</li>
<li>Gazebo's JointTrajectoryController replaces any trajectory it was following with this one. Every 1 ms physics
step it picks the current point (by time), computes PID torque for each joint, and clamps it to the torque limit.</li>
<li>The physics engine integrates the motion: joints turn, the attached box is carried.</li>
<li>JointStatePublisher publishes the new joint angles; the bridge converts them to <code>sensor_msgs/JointState</code> on
<code>/joint_states</code>.</li>
<li><code>_joint_state_callback</code> in the planner stores them; <code>_execute</code> compares them with the goal. The GUI's
<code>JointMonitor</code> stores them too and plots them.</li>
</ol>
<div class="callout warn"><b class="t">A trajectory replaces the previous one</b>
Publishing a new trajectory while one is running makes the robot abandon the old one. That is why the planner
always waits for each stage to finish, and why the GUI allows only one program at a time.</div>
""")

# ------------------------------------------------------------------ PART 3
chapter("Part 3 – Using it", "run", r"""
<h1 id="run">16. Running, parameters and tuning</h1>
<h2>16.1 Every time</h2>
<pre class="term"><span class="c"># terminal 1</span>
cd ~/Documents/GitHub/YASCAWA_GP7_sim/gp7_ws
source install/setup.bash
ros2 launch gp7robot_description gazebo_sim.launch.py run_demo:=false

<span class="c"># terminal 2</span>
cd ~/Documents/GitHub/YASCAWA_GP7_sim/gp7_ws
source install/setup.bash
ros2 run gp7robot_description gp7_gui</pre>
<p>After editing code: <code>colcon build --packages-select gp7robot_description</code>, then restart the GUI (programs the
GUI starts with <b>Start task</b> pick up the new build automatically; the GUI's own code and the random-placement
checker need a GUI restart).</p>
<h2>16.2 Launch arguments</h2>
<table><tr><th>Argument</th><th>Default</th><th>Meaning</th></tr>
<tr><td><code>run_demo</code></td><td>true</td><td>start the task automatically 10 s after launch</td></tr>
<tr><td><code>headless</code></td><td>false</td><td>run Gazebo without its window (server only)</td></tr>
<tr><td><code>world</code></td><td>gp7_pick_place_reachable.sdf</td><td>which world file to load</td></tr></table>
<h2>16.3 Planner parameters (<code>gp7_pick_place --ros-args -p name:=value</code>)</h2>
<table><tr><th>Parameter</th><th>Default</th><th>Meaning</th></tr>
<tr><td>box_a_x, box_a_y</td><td>0.50, 0.50</td><td>Box A centre (m)</td></tr>
<tr><td>box_a_top_z</td><td>0.20</td><td>height of Box A's top (m)</td></tr>
<tr><td>box_b_x, box_b_y</td><td>0.35, 0.25</td><td>Box B start (m) – the GUI sets these</td></tr>
<tr><td>box_b_center_z</td><td>0.025</td><td>height of Box B's centre</td></tr>
<tr><td>clearance_height</td><td>0.55</td><td>how high Box B is lifted before moving across</td></tr>
<tr><td>retreat_height</td><td>0.15</td><td>how far the arm lifts after releasing</td></tr>
<tr><td>speed_scale</td><td>0.30</td><td>fraction of datasheet joint speeds allowed</td></tr>
<tr><td>cartesian_speed</td><td>0.25</td><td>tool speed on straight lines (m/s)</td></tr>
<tr><td>cartesian_acceleration</td><td>0.40</td><td>tool acceleration (m/s²)</td></tr>
<tr><td>blend_radius</td><td>0.10</td><td>corner arc radius (m)</td></tr>
<tr><td>use_sim_time</td><td>(set true)</td><td>follow Gazebo's clock</td></tr></table>
<h2>16.4 Tuning advice</h2>
<ul>
<li><b>Faster</b>: raise <code>cartesian_speed</code>/<code>cartesian_acceleration</code> and <code>speed_scale</code> together. Watch
the GUI: if joint errors grow (warnings "finished X degrees from the target"), the PID cannot keep up.</li>
<li><b>Smoother</b>: lower <code>cartesian_acceleration</code> (longer, gentler ramps) or raise <code>blend_radius</code>.</li>
<li><b>Box B hits Box A while moving across</b>: raise <code>clearance_height</code>.</li>
<li>If you change defaults in pick_place.py, workspace.py takes its defaults from the same constants for speed;
other values (clearance, retreat) are listed in <code>DEFAULT_PARAMETERS</code> and must match.</li>
</ul>
""")

chapter("Part 3 – Using it", "debug", r"""
<h1 id="debug">17. When things go wrong</h1>
<table>
<tr><th>Symptom / message</th><th>Cause</th><th>Fix</th></tr>
<tr><td><code>Package 'gp7robot_description' not found</code></td><td>terminal not sourced</td><td><code>source install/setup.bash</code></td></tr>
<tr><td>My code change does nothing</td><td>ROS runs <code>install/</code>, not <code>src/</code></td><td>rebuild with colcon; restart the GUI</td></tr>
<tr><td><code>Gazebo bridge did not subscribe to the joint trajectory topic</code></td><td>Gazebo or the bridge is not running, or the robot is not spawned yet</td><td>start the launch file, wait ~10 s</td></tr>
<tr><td><code>No usable /joint_states ...</code></td><td>robot not spawned, or joint names differ</td><td><code>ros2 topic echo /joint_states</code> to see names</td></tr>
<tr><td><code>Target ... GP7 maximum horizontal reach is 0.927 m</code></td><td>Box B too far away</td><td>move it closer</td></tr>
<tr><td><code>No IK solution within tolerance</code></td><td>pose not reachable with the gripper pointing down from that seed</td><td>other position; see the reachability map</td></tr>
<tr><td><code>IK branch switch on the upright Cartesian path</code></td><td>the straight line passes near a singularity/limit</td><td>other position, or lower clearance_height</td></tr>
<tr><td><code>Gazebo did not reach the ... waypoint</code></td><td>PID could not follow (too fast, collision, torque limit)</td><td>lower speeds; check for collisions in Gazebo</td></tr>
<tr><td>Box B falls / slides off</td><td>attach missed, or arm moved before attach</td><td>check the "Gripper attach" log line; DetachableJoint needs Box B's model name <code>BoxB</code>, link <code>link</code></td></tr>
<tr><td>GUI: "... is still running; stop it first"</td><td>one program at a time</td><td>wait, or press Stop</td></tr>
<tr><td>Gazebo very slow (real-time factor &lt; 1)</td><td>weak GPU / CPU</td><td>fine: everything uses sim time; or run headless</td></tr>
</table>
<h2>Debugging method</h2>
<ol>
<li>Read the <b>first</b> error message, not the last.</li>
<li>Check the plumbing: <code>ros2 node list</code>, <code>ros2 topic list</code>, <code>ros2 topic info -v &lt;topic&gt;</code>
(is there a publisher <i>and</i> a subscriber?).</li>
<li>Check the data: <code>ros2 topic echo /joint_states --once</code>.</li>
<li>Test the maths alone in Python (no ROS needed), e.g. <code>check_box_b(0.6, -0.3)</code> returns <code>None</code> if the
plan works, or the reason it does not.</li>
</ol>
""")

chapter("Part 3 – Using it", "learn", r"""
<h1 id="learn">18. Exercises, learning path and glossary</h1>
<h2>18.1 Exercises (in increasing difficulty)</h2>
<ol>
<li>Open RViz with <code>display.launch.py</code>. Move each slider and write down what each joint does.</li>
<li>With Gazebo running: <code>ros2 topic echo /joint_states</code> while the task runs. Which joint moves most?</li>
<li>In Python, compute FK for q = 0 and for the upright pose. Check the flange of the upright pose is at
(0.35, 0.25, 0.42).</li>
<li>Run the task with <code>-p cartesian_speed:=0.1</code> and with <code>0.4</code>. Compare the GUI speed plot.</li>
<li>Call <code>check_box_b</code> for (−0.5, 0.4). Read the error. Why does it fail? (Hint: chapter 6.6.)</li>
<li>Change <code>retreat_height</code> so the arm goes back to the clearance height after placing.</li>
<li>Make the planner go home (all joints 0) at the end, using <code>_move_joint</code>.</li>
<li>Add a <code>box_a_x</code>/<code>box_a_y</code> field to the GUI (and teleport Box A with set_pose, like Box B).</li>
<li>Advanced: replace the lift-and-rotate joint move with a smoother path so the gripper does not rise to 1.3 m.</li>
</ol>
<h2>18.2 Where to learn more</h2>
<ul>
<li>ROS 2 documentation: docs.ros.org – the "Beginner: CLI tools" and "Beginner: Client libraries" tutorials (Python).</li>
<li>Gazebo Sim documentation: gazebosim.org/docs – "Building your own robot", "Moving the robot", "ROS 2 integration".</li>
<li>Kevin Lynch &amp; Frank Park, <i>Modern Robotics</i> (free PDF and videos): frames, FK, Jacobian, IK, trajectories.</li>
<li>Craig, <i>Introduction to Robotics: Mechanics and Control</i>: the classic DH treatment.</li>
<li>Peter Corke, <i>Robotics, Vision and Control</i> and his Robotics Toolbox for Python: try FK/IK interactively.</li>
</ul>
<h2>18.3 Glossary</h2>
<table>
<tr><th>Word</th><th>Meaning</th></tr>
<tr><td>Bridge</td><td>program that converts messages between ROS 2 and Gazebo (<code>ros_gz_bridge</code>)</td></tr>
<tr><td>Callback</td><td>function ROS calls when a message arrives</td></tr>
<tr><td>Cartesian / task space</td><td>describing the hand by x, y, z and orientation</td></tr>
<tr><td>colcon</td><td>ROS 2 build tool; fills <code>install/</code></td></tr>
<tr><td>DH parameters</td><td>4 numbers per joint (θ, d, a, α) describing an arm's geometry</td></tr>
<tr><td>Damped least squares</td><td>numerical IK method that stays stable near singularities</td></tr>
<tr><td>DetachableJoint</td><td>Gazebo system that glues/unglues two models on command – the simulated gripper</td></tr>
<tr><td>Flange</td><td>the mounting face at the end of the arm (Link6)</td></tr>
<tr><td>Forward kinematics</td><td>joint angles → hand pose</td></tr>
<tr><td>Homogeneous transform</td><td>4×4 matrix holding a rotation and a translation</td></tr>
<tr><td>IK branch</td><td>one of the several joint solutions for the same hand pose</td></tr>
<tr><td>Inverse kinematics</td><td>hand pose → joint angles</td></tr>
<tr><td>Jacobian</td><td>matrix linking joint velocities to hand velocity</td></tr>
<tr><td>Jerk</td><td>rate of change of acceleration</td></tr>
<tr><td>Joint space</td><td>describing the robot by its joint angles</td></tr>
<tr><td>Launch file</td><td>Python script that starts several ROS programs</td></tr>
<tr><td>Link</td><td>a rigid part of the robot</td></tr>
<tr><td>Message</td><td>a typed data structure sent on a topic</td></tr>
<tr><td>Node</td><td>a ROS program (or object) with one job</td></tr>
<tr><td>Package</td><td>unit of ROS code with a package.xml</td></tr>
<tr><td>Parameter</td><td>a named setting of a node, changeable at start-up</td></tr>
<tr><td>PID</td><td>controller computing torque from error, its integral and derivative</td></tr>
<tr><td>Pose</td><td>position + orientation</td></tr>
<tr><td>Publisher / subscriber</td><td>sender / receiver on a topic</td></tr>
<tr><td>Radian</td><td>angle unit; π rad = 180°</td></tr>
<tr><td>RViz</td><td>ROS 3D viewer (no physics)</td></tr>
<tr><td>S-curve</td><td>motion profile with smoothly changing acceleration</td></tr>
<tr><td>SDF</td><td>Gazebo's file format for worlds and models</td></tr>
<tr><td>Seed</td><td>starting guess for numerical IK</td></tr>
<tr><td>Service</td><td>request/response call (vs one-way topic)</td></tr>
<tr><td>Sim time</td><td>the simulator's clock, published on <code>/clock</code></td></tr>
<tr><td>Singularity</td><td>pose where the arm loses a direction of motion; J is not invertible</td></tr>
<tr><td>Sourcing</td><td>loading a setup script so the terminal can find ROS packages</td></tr>
<tr><td>Spin</td><td>let ROS process incoming messages and call callbacks</td></tr>
<tr><td>Topic</td><td>named message channel</td></tr>
<tr><td>Trajectory</td><td>path + timing</td></tr>
<tr><td>URDF</td><td>ROS XML format describing a robot's links and joints</td></tr>
<tr><td>Workspace (robot)</td><td>the set of poses the hand can reach</td></tr>
<tr><td>Workspace (ROS)</td><td>a folder with src/, build/, install/</td></tr>
</table>
""")


def build() -> None:
    toc, parts, current = [], [], None
    for part, cid, body in CHAPTERS:
        title = re.search(r"<h1[^>]*>(.*?)</h1>", body).group(1)
        if part != current:
            toc.append(f'<div class="part">{html.escape(part)}</div>')
            current = part
        toc.append(f'<div class="ch"><a href="#{cid}">{title}</a></div>')
        parts.append(expand(body))
    cover = """
<div class="cover">
  <div class="t1">The GP7 Pick-and-Place Project</div>
  <div class="t2">A complete learning guide to the code, ROS 2, Gazebo,<br>kinematics and motion planning</div>
  <div class="t3">Yaskawa Motoman GP7 · ROS 2 Lyrical · Gazebo Sim · Python<br>
  package <code>gp7robot_description</code><br><br>No prior knowledge assumed</div>
</div>
<h1 class="nobreak" style="page-break-before:avoid">Contents</h1>
<div class="toc">""" + "\n".join(toc) + "</div>"
    document = (
        "<!DOCTYPE html><html><head><meta charset='utf-8'><title>GP7 learning guide</title>"
        f"<style>{CSS}</style></head><body>{cover}{''.join(parts)}</body></html>"
    )
    with open(OUT_HTML, "w", encoding="utf-8") as handle:
        handle.write(document)
    chrome = next(
        (c for c in ("google-chrome", "chromium", "chromium-browser") if subprocess.run(
            ["which", c], capture_output=True).returncode == 0), None)
    if chrome is None:
        sys.exit(f"wrote {OUT_HTML}; no Chrome/Chromium found to print the PDF")
    subprocess.run(
        [chrome, "--headless", "--disable-gpu", "--no-pdf-header-footer",
         "--generate-pdf-document-outline", f"--print-to-pdf={OUT_PDF}",
         "file://" + OUT_HTML],
        check=True, capture_output=True,
    )
    print("wrote", OUT_PDF)


if __name__ == "__main__":
    build()
