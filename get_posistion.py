import numpy as np
import os
from stl import mesh
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from mpl_toolkits.mplot3d import art3d

# ══════════════════════════════════════════════════════════════
# 1. FILE PATHS & THÔNG TIN
# ══════════════════════════════════════════════════════════════
FILES = {
    'base':   'STL_File/base.stl',
    'pan':    'STL_File/panelMount - Sao chép.stl',
    'mount':  'STL_File/PanelMount.stl',
    'shaft':  'STL_File/truc.stl',
    'sensor': 'STL_File/ThanhChiaQuangTro.stl',
    'phe1':   'STL_File/DIN 6799 - 12 13.stl',  # Phe gài 1
    'phe2':   'STL_File/DIN 6799 - 12 13.stl'   # Phe gài 2
}

COLORS = {
    'base': '#808080', 'pan': '#c0c0c0', 'mount': '#4682B4',
    'shaft': '#A9A9A9', 'sensor': '#DAA520', 'phe1': '#B22222', 'phe2': '#B22222'
}

# ══════════════════════════════════════════════════════════════
# 2. KHỞI TẠO VỊ TRÍ BAN ĐẦU GẦN ĐÚNG NHẤT (PRE-FILLED STATE)
# ══════════════════════════════════════════════════════════════
# tx, ty, tz: Tịnh tiến | rx, ry, rz: Góc xoay (độ)
state = {
    'base':   {'tx': 0.0,  'ty': -65.0, 'tz': 0.0,   'rx': 0, 'ry': 0,  'rz': 0},
    'pan':    {'tx': 0.0,  'ty': 10.0,  'tz': 134.0, 'rx': 0, 'ry': 0,  'rz': 0},
    'mount':  {'tx': 0.0,  'ty': 27.5,  'tz': 120.0, 'rx': 0, 'ry': 0,  'rz': 0},
    'shaft':  {'tx': 0.0,  'ty': 0.0,   'tz': 120.0, 'rx': 0, 'ry': 0,  'rz': 0},
    'sensor': {'tx': 0.0,  'ty': 0.0,   'tz': 212.0, 'rx': 0, 'ry': 0,  'rz': 0},
    'phe1':   {'tx': 62.0, 'ty': 0.0,   'tz': 120.0, 'rx': 0, 'ry': 90, 'rz': 0},
    'phe2':   {'tx':-62.0, 'ty': 0.0,   'tz': 120.0, 'rx': 0, 'ry': 90, 'rz': 0}
}

# ══════════════════════════════════════════════════════════════
# 3. TẢI FILE, GIẢM LƯỚI & ÉP TÂM TUYỆT ĐỐI
# ══════════════════════════════════════════════════════════════
print("Đang nạp công cụ lắp ráp...")
orig_vectors = {}
STEP = 3 # Giảm số lượng tam giác để giao diện mượt, không đơ lag

for name, path in FILES.items():
    target = path if os.path.exists(path) else path.replace("STL_File/", "")
    if not os.path.exists(target):
        print(f"⚠️ Không tìm thấy file: {target}")
        orig_vectors[name] = None
        continue
    
    m = mesh.Mesh.from_file(target)
    m.y, m.z = m.z.copy(), m.y.copy() # Lật trục Inventor -> Python
    
    v = m.vectors[::STEP].copy()
    
    # Ép tâm hình học về chuẩn (0,0,0) ĐỂ KHI XOAY KHÔNG BỊ VĂNG
    center = (v.reshape(-1, 3).min(axis=0) + v.reshape(-1, 3).max(axis=0)) / 2.0
    v -= center
    orig_vectors[name] = v

# ══════════════════════════════════════════════════════════════
# 4. HÀM TÍNH TOÁN & CẬP NHẬT 3D
# ══════════════════════════════════════════════════════════════
def get_R(rx, ry, rz):
    rx, ry, rz = np.radians(rx), np.radians(ry), np.radians(rz)
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx # Xoay theo thứ tự X -> Y -> Z

def update_plot():
    for name in FILES.keys():
        if orig_vectors[name] is not None:
            s = state[name]
            # B1: Xoay tại chỗ
            R = get_R(s['rx'], s['ry'], s['rz'])
            v_rot = orig_vectors[name].reshape(-1, 3) @ R.T
            # B2: Tịnh tiến trong không gian
            v_final = v_rot + np.array([s['tx'], s['ty'], s['tz']])
            polys[name].set_verts(v_final.reshape(-1, 3, 3))
    fig.canvas.draw_idle()

# ══════════════════════════════════════════════════════════════
# 5. KHỞI TẠO GIAO DIỆN CHÍNH
# ══════════════════════════════════════════════════════════════
fig = plt.figure(figsize=(15, 9))
ax = fig.add_axes([0.02, 0.05, 0.65, 0.9], projection='3d')
ax.set_proj_type('ortho') # Chế độ Orthographic chuẩn CAD

polys = {}
for name in FILES.keys():
    if orig_vectors[name] is not None:
        poly = art3d.Poly3DCollection(orig_vectors[name], facecolors=COLORS[name], 
                                      edgecolors='#555555', linewidths=0.2, alpha=0.9)
        ax.add_collection3d(poly)
        polys[name] = poly

L = 200
ax.set_xlim([-L, L]); ax.set_ylim([-L, L]); ax.set_zlim([-50, 300])
ax.set_xlabel('Trục X'); ax.set_ylabel('Trục Y'); ax.set_zlabel('Trục Z')
ax.set_title("CÔNG CỤ LẮP RÁP THỦ CÔNG", fontweight='bold')

# ══════════════════════════════════════════════════════════════
# 6. BẢNG ĐIỀU KHIỂN (BÊN PHẢI)
# ══════════════════════════════════════════════════════════════
ui_lock = False # Khóa UI khi đổi chi tiết để tránh lỗi lặp vòng

# A. Radio Buttons (Chọn chi tiết)
fig.text(0.72, 0.90, "1. CHỌN CHI TIẾT:", fontweight='bold')
ax_radio = plt.axes([0.72, 0.65, 0.20, 0.23], facecolor='#e0ffff')
radio = RadioButtons(ax_radio, list(FILES.keys()))

# B. Buttons (Xoay 90 độ)
fig.text(0.72, 0.61, "2. XOAY 90 ĐỘ TẠI CHỖ:", fontweight='bold')
ax_rx = plt.axes([0.72, 0.55, 0.08, 0.05]); btn_rx = Button(ax_rx, 'Xoay X', color='#ffb6c1')
ax_ry = plt.axes([0.81, 0.55, 0.08, 0.05]); btn_ry = Button(ax_ry, 'Xoay Y', color='#ffb6c1')
ax_rz = plt.axes([0.90, 0.55, 0.08, 0.05]); btn_rz = Button(ax_rz, 'Xoay Z', color='#ffb6c1')

def rot(axis):
    active = radio.value_selected
    state[active][f'r{axis}'] = (state[active][f'r{axis}'] + 90) % 360
    update_plot()

btn_rx.on_clicked(lambda e: rot('x'))
btn_ry.on_clicked(lambda e: rot('y'))
btn_rz.on_clicked(lambda e: rot('z'))

# C. Sliders (Tịnh tiến)
fig.text(0.72, 0.49, "3. TỊNH TIẾN (KÉO CHUỘT):", fontweight='bold')
ax_tx = plt.axes([0.75, 0.42, 0.20, 0.03], facecolor='#ffe4b5')
ax_ty = plt.axes([0.75, 0.35, 0.20, 0.03], facecolor='#ffe4b5')
ax_tz = plt.axes([0.75, 0.28, 0.20, 0.03], facecolor='#ffe4b5')

s_tx = Slider(ax_tx, 'Trục X', -200, 200, valinit=state['base']['tx'])
s_ty = Slider(ax_ty, 'Trục Y', -200, 200, valinit=state['base']['ty'])
s_tz = Slider(ax_tz, 'Trục Z', -100, 300, valinit=state['base']['tz'])

def slider_changed(val):
    if ui_lock: return
    active = radio.value_selected
    state[active]['tx'] = s_tx.val
    state[active]['ty'] = s_ty.val
    state[active]['tz'] = s_tz.val
    update_plot()

s_tx.on_changed(slider_changed)
s_ty.on_changed(slider_changed)
s_tz.on_changed(slider_changed)

# Xử lý khi đổi chi tiết: Làm sáng chi tiết đang chọn & Cập nhật thanh trượt
def part_selected(label):
    global ui_lock
    ui_lock = True 
    
    # 1. Hiệu ứng làm sáng/mờ
    for name, poly in polys.items():
        if name == label:
            poly.set_alpha(0.95); poly.set_linewidths(0.3)
        else:
            poly.set_alpha(0.25); poly.set_linewidths(0.0)
            
    # 2. Update giá trị lên thanh trượt theo state hiện hành của chi tiết đó
    s_tx.set_val(state[label]['tx'])
    s_ty.set_val(state[label]['ty'])
    s_tz.set_val(state[label]['tz'])
    
    ui_lock = False
    fig.canvas.draw_idle()

radio.on_clicked(part_selected)
part_selected('base') 

# D. Nút XUẤT LOG
ax_export = plt.axes([0.72, 0.10, 0.25, 0.08])
btn_export = Button(ax_export, '4. XUẤT LOG LÊN TERMINAL', color='#90ee90', hovercolor='#32cd32')

def export_log(event):
    print("\n" + "═"*60)
    print("📋 ĐÃ LƯU TỌA ĐỘ! HÃY COPY ĐOẠN DƯỚI ĐÂY GỬI LẠI CHO TÔI:")
    print("═"*60)
    print("LOG_ASSEMBLY = {")
    for name, s in state.items():
        print(f"    '{name}': {{'pos': [{s['tx']:.1f}, {s['ty']:.1f}, {s['tz']:.1f}], 'rot': [{s['rx']}, {s['ry']}, {s['rz']}]}},")
    print("}")
    print("═"*60 + "\n")

btn_export.on_clicked(export_log)

update_plot()
plt.show()
