"""
2-DOF Solar Tracker - 3D Kinematics Simulation
==============================================
Client Release Version.
Applies mathematical translations and rotations for exact assembly.
"""

import os
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import art3d
from typing import Dict, List, Tuple

# ==============================================================================
# 1. FILE PATHS & ASSEMBLY OFFSETS
# ==============================================================================
FILES: Dict[str, str] = {
    'base':   'STL_File/base.stl',
    'pan':    'STL_File/pan.stl',
    'mount':  'STL_File/mount.stl',
    'shaft':   'STL_File/truc.stl',
    'sensor': 'STL_File/ThanhChiaQuangTro.stl'
}

# Calibrated assembly coordinates [tx, ty, tz] and rotations [rx, ry, rz] in degrees
LOG_ASSEMBLY: Dict[str, Dict[str, List[float]]] = {
    'base':   {'pos': [68.0, -129.3, 0.0],    'rot': [0, 0, 270]},
    'pan':    {'pos': [5.3, -129.3, 62.7],    'rot': [0, 0, 0]},
    'mount':  {'pos': [-21.3, -130.7, 126.7], 'rot': [0, 0, 90]},
    'shaft':  {'pos': [5.3, -129.3, 128.0],   'rot': [270, 180, 90]},
    'sensor': {'pos': [-124.0, -130.7, 213.3],'rot': [90, 0, 0]}
}

# ==============================================================================
# 2. CORE FUNCTIONS
# ==============================================================================
def read_stl(path: str) -> np.ndarray:
    """
    Reads an STL file and extracts its triangular faces.

    Args:
        path (str): The file path to the STL file.

    Returns:
        np.ndarray: A NumPy array containing the vertices of the triangles.
    """
    with open(path, 'rb') as f:
        f.read(80)  # Skip header
        n = struct.unpack('<I', f.read(4))[0]
        tris = []
        for _ in range(n):
            f.read(12)  # Skip normal vector
            tri = [struct.unpack('<fff', f.read(12)) for _ in range(3)]
            f.read(2)  # Skip attribute byte count
            tris.append(tri)
    return np.array(tris, dtype=np.float64)

def get_R(rx: float, ry: float, rz: float) -> np.ndarray:
    """
    Generates a 3D rotation matrix from Euler angles (X-Y-Z).

    Args:
        rx (float): Rotation angle around X-axis in degrees.
        ry (float): Rotation angle around Y-axis in degrees.
        rz (float): Rotation angle around Z-axis in degrees.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    rx, ry, rz = np.radians(rx), np.radians(ry), np.radians(rz)
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def transform_mesh(v_raw: np.ndarray, name: str) -> np.ndarray:
    """
    Processes the raw mesh: fixes coordinates, centers it, and applies assembly offsets.

    Args:
        v_raw (np.ndarray): Raw mesh vertices.
        name (str): Component identifier.

    Returns:
        np.ndarray: Transformed mesh vertices ready for assembly.
    """
    pts = v_raw.copy().reshape(-1, 3)

    # 1. Convert coordinate system (Inventor to Python Matplotlib)
    pts[:, 1], pts[:, 2] = v_raw.reshape(-1, 3)[:, 2].copy(), v_raw.reshape(-1, 3)[:, 1].copy()

    # 2. Center the part at origin (0,0,0) to prevent offset drifts during rotation
    center = (pts.min(0) + pts.max(0)) / 2.0
    pts -= center

    # 3. Apply static rotations
    R = get_R(*LOG_ASSEMBLY[name]['rot'])
    pts = pts @ R.T

    # 4. Apply static translations
    pts += np.array(LOG_ASSEMBLY[name]['pos'])

    return pts.reshape(-1, 3, 3)

# ==============================================================================
# 3. INITIALIZATION & MESH LOADING
# ==============================================================================
print("Initializing 2-DOF Solar Tracker 3D Simulation...")

meshes: Dict[str, np.ndarray] = {}
for name, path in FILES.items():
    if not os.path.exists(path):
        meshes[name] = None
        continue
    raw = read_stl(path)
    meshes[name] = transform_mesh(raw, name)

orig: Dict[str, np.ndarray] = {k: v.copy() for k, v in meshes.items() if v is not None}

# ==============================================================================
# 4. KINEMATICS CONFIGURATION
# ==============================================================================
CYL_X: float = 5.3
CYL_Y: float = -129.3
Z_TILT: float = 128.0
SHAFT_Y: float = -125.3

# Absolute pivot points for the joints
PAN_PIVOT: np.ndarray = np.array([CYL_X, CYL_Y, 0.0])
TILT_ORIGIN: np.ndarray = np.array([CYL_X, SHAFT_Y, Z_TILT])

def Rz(deg: float) -> np.ndarray:
    """Returns Z-axis rotation matrix."""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def Ry(deg: float) -> np.ndarray:
    """Returns Y-axis rotation matrix."""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rot3d(pts: np.ndarray, origin: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Applies a 3D rotation matrix relative to a specific origin point."""
    return (pts - origin) @ R.T + origin

def update(val: float) -> None:
    """Callback function to update the mesh positions based on slider values."""
    Rpan = Rz(s_pan.val)
    Rtilt = Ry(s_tilt.val)

    if 'base' in polys:
        polys['base'].set_verts(orig['base'])

    if 'pan' in polys:
        v = rot3d(orig['pan'].reshape(-1, 3), PAN_PIVOT, Rpan)
        polys['pan'].set_verts(v.reshape(-1, 3, 3))

    for p in ['mount', 'shaft', 'sensor']:
        if p not in polys:
            continue
        # Apply tilt rotation first, then inherit pan rotation
        v = rot3d(orig[p].reshape(-1, 3), TILT_ORIGIN, Rtilt)
        v = rot3d(v, PAN_PIVOT, Rpan)
        polys[p].set_verts(v.reshape(-1, 3, 3))

    fig.canvas.draw_idle()

# ==============================================================================
# 5. UI & RENDERING
# ==============================================================================
COLORS: Dict[str, str] = {
    'base': '#606060', 'pan': '#d8d8d8', 'mount': '#4682b4',
    'shaft': '#8899aa', 'sensor': '#daa520'
}
ALPHA: Dict[str, float] = {
    'base': 0.85, 'pan': 0.78, 'mount': 0.80,
    'shaft': 0.95, 'sensor': 0.95
}

fig = plt.figure(figsize=(14, 9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.14, bottom=0.22, right=0.87)
ax.set_proj_type('ortho')  # Use orthographic projection for CAD-like viewing

polys: Dict[str, art3d.Poly3DCollection] = {}
for name, v in meshes.items():
    if v is None:
        continue
    poly = art3d.Poly3DCollection(
        v,
        facecolors=COLORS.get(name, '#aaa'),
        alpha=ALPHA.get(name, 0.8),
        edgecolors='k',
        linewidths=0.15,
        shade=True
    )
    ax.add_collection3d(poly)
    polys[name] = poly

# Viewport limits setup
R: float = 230
Zmax: float = 280
ax.set_xlim([PAN_PIVOT[0] - R, PAN_PIVOT[0] + R])
ax.set_ylim([PAN_PIVOT[1] - R, PAN_PIVOT[1] + R])
ax.set_zlim([0, Zmax])
ax.axis('off')  # Clean UI without coordinate axes

# Dynamic Joint Markers
ax.scatter(*PAN_PIVOT, c='lime', s=70, zorder=6)
ax.scatter(*TILT_ORIGIN, c='yellow', s=70, zorder=6)
ax.plot(
    [TILT_ORIGIN[0] - 80, TILT_ORIGIN[0] + 80],
    [TILT_ORIGIN[1]] * 2,
    [TILT_ORIGIN[2]] * 2,
    'y--', lw=1.2, alpha=0.7
)

# Interactive Sliders
ax_pan  = plt.axes([0.14, 0.10, 0.70, 0.03], facecolor='#fffacd')
ax_tilt = plt.axes([0.14, 0.05, 0.70, 0.03], facecolor='#fffacd')
s_pan   = Slider(ax_pan,  'Pan Angle (°)', -180, 180, valinit=0, color='#2ca02c')
s_tilt  = Slider(ax_tilt, 'Tilt Angle (°)', -45, 45, valinit=0, color='#1f77b4')
s_pan.on_changed(update)
s_tilt.on_changed(update)

# Viewport Control Buttons
def make_btn(rect: List[float], label: str, elev: float, azim: float) -> Button:
    """Helper to generate viewport angle transition buttons."""
    a = plt.axes(rect)
    b = Button(a, label, hovercolor='#add8e6')
    b.on_clicked(lambda e: (ax.view_init(elev, azim), fig.canvas.draw_idle()))
    return b

btn_iso   = make_btn([0.01, 0.82, 0.09, 0.05], 'ISO', 40, 225)
btn_top   = make_btn([0.01, 0.76, 0.09, 0.05], 'Top', 90, -90)
btn_front = make_btn([0.01, 0.70, 0.09, 0.05], 'Front', 0, -90)
btn_side  = make_btn([0.01, 0.64, 0.09, 0.05], 'Side', 0, 0)

# ==============================================================================
# 6. AUTO SEARCH MODE
# ==============================================================================
rng = np.random.default_rng()
AUTO_ACTIVE = False
AUTO_TARGET = {'pan': 0.0, 'tilt': 0.0}

def wrap_pan(angle: float) -> float:
    """Wrap angle to the range [-180, 180]."""
    return float(((angle + 180.0) % 360.0) - 180.0)

def shortest_angle_delta(current: float, target: float) -> float:
    """Compute shortest signed angular difference for pan movement."""
    return float(((target - current + 180.0) % 360.0) - 180.0)

def move_towards(current: float, target: float, step: float) -> float:
    """Move current value toward target by a limited step."""
    delta = target - current
    if abs(delta) <= step:
        return target
    return current + step * np.sign(delta)

def new_auto_target() -> None:
    """Generate a new random target for the tracker."""
    AUTO_TARGET['pan'] = float(rng.uniform(-180.0, 180.0))
    AUTO_TARGET['tilt'] = float(rng.uniform(-45.0, 45.0))

def auto_tick() -> None:
    """Timer callback for automatic solar-search motion."""
    if not AUTO_ACTIVE:
        return

    pan_now = float(s_pan.val)
    tilt_now = float(s_tilt.val)

    # Pick a new target when close enough or occasionally to keep motion irregular
    pan_close = abs(shortest_angle_delta(pan_now, AUTO_TARGET['pan'])) < 3.0
    tilt_close = abs(tilt_now - AUTO_TARGET['tilt']) < 1.0
    if pan_close and tilt_close or rng.random() < 0.04:
        new_auto_target()

    pan_step = 2.5
    tilt_step = 0.8

    pan_next = pan_now + np.clip(shortest_angle_delta(pan_now, AUTO_TARGET['pan']), -pan_step, pan_step)
    tilt_next = move_towards(tilt_now, AUTO_TARGET['tilt'], tilt_step)

    # Slider updates will trigger the main update() callback
    s_pan.set_val(wrap_pan(pan_next))
    s_tilt.set_val(float(np.clip(tilt_next, -45.0, 45.0)))

def toggle_auto(event) -> None:
    """Toggle automatic random search mode."""
    global AUTO_ACTIVE
    AUTO_ACTIVE = not AUTO_ACTIVE
    btn_auto.label.set_text('AUTO: ON' if AUTO_ACTIVE else 'AUTO: OFF')
    if AUTO_ACTIVE:
        new_auto_target()
    fig.canvas.draw_idle()

# Auto mode button
btn_auto_ax = plt.axes([0.01, 0.58, 0.09, 0.05])
btn_auto = Button(btn_auto_ax, 'AUTO: OFF', hovercolor='#ffcccb')
btn_auto.on_clicked(toggle_auto)

# Automatic motion timer
auto_timer = fig.canvas.new_timer(interval=80)
auto_timer.add_callback(auto_tick)
auto_timer.start()

# Apply default start view
ax.view_init(elev=40, azim=225)
plt.show()
