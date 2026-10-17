import re
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Set font for proper rendering
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# Parse reg_read.vcd
with open("d:/FPGA/SoftWare-Project/lha6958h-pro/reg_read.vcd", "r") as f:
    lines = f.readlines()

# Variable symbol mapping
var_map = {}
for line in lines:
    m = re.match(r'\$var\s+(?:wire|reg)\s+(\d+)\s+(\S+)\s+(\S+)', line)
    if m:
        width, sym, name = m.groups()
        var_map[sym] = name

# Target signals to plot
# clk ("), rst_n (#), start_i ($), busy_o (.), reg_cs_n (1), reg_wr_n (3), reg_rd_n (2), reg_db_oe (D), done_o (/), err_o (0)
# Multi-bit: db_bus (E..T), data_o (&..-)

time_ps = 0
changes = [] # (time_ns, sym, val)

# We are interested in Test 1: from 150ns to 1350ns (150_000ps to 1_350_000ps)
db_bus_bits = {sym: i for i, sym in enumerate(['T','S','R','Q','P','O','N','M','L','K','J','I','H','G','F','E'])} # 0..15
data_o_bits = {sym: i for i, sym in enumerate(['-','', '+', '*', ')', '(', '\'', '&'])}

# Track signals over time
time_events = []
current_vals = {
    'clk': 0, 'rst_n': 0, 'start_i': 0, 'busy_o': 0,
    'reg_cs_n': 1, 'reg_wr_n': 1, 'reg_rd_n': 1, 'reg_db_oe': 0,
    'done_o': 0, 'err_o': 0, 'db_bus': 0, 'data_o': 0
}
db_bus_val = ['z']*16
data_o_val = ['0']*8

history = []

for line in lines:
    line = line.strip()
    if not line:
        continue
    if line.startswith('#'):
        time_ps = int(line[1:])
        time_ns = time_ps / 1000.0
        # snapshot
        # evaluate db_bus
        if any(c in 'zxZX' for c in db_bus_val):
            db_hex = 'Hi-Z'
        else:
            db_int = int(''.join(reversed(db_bus_val)), 2)
            db_hex = f"16'h{db_int:04X}"
        
        data_int = int(''.join(reversed([c if c in '01' else '0' for c in data_o_val])), 2)
        data_hex = f"8'h{data_int:02X}"

        snap = dict(current_vals)
        snap['time'] = time_ns
        snap['db_bus'] = db_hex
        snap['data_o'] = data_hex
        history.append(snap)
    elif len(line) >= 2 and line[0] in '01xz':
        val = line[0]
        sym = line[1:]
        if sym == '"': current_vals['clk'] = int(val) if val in '01' else 0
        elif sym == '#': current_vals['rst_n'] = int(val) if val in '01' else 0
        elif sym == '$': current_vals['start_i'] = int(val) if val in '01' else 0
        elif sym == '.': current_vals['busy_o'] = int(val) if val in '01' else 0
        elif sym == '1': current_vals['reg_cs_n'] = int(val) if val in '01' else 1
        elif sym == '3': current_vals['reg_wr_n'] = int(val) if val in '01' else 1
        elif sym == '2': current_vals['reg_rd_n'] = int(val) if val in '01' else 1
        elif sym == 'D': current_vals['reg_db_oe'] = int(val) if val in '01' else 0
        elif sym == '/': current_vals['done_o'] = int(val) if val in '01' else 0
        elif sym == '0': current_vals['err_o'] = int(val) if val in '01' else 0
        elif sym in db_bus_bits:
            db_bus_val[db_bus_bits[sym]] = val
        elif sym in data_o_bits:
            data_o_val[data_o_bits[sym]] = val

# Filter window for Test 1: 180ns to 1320ns
t_start = 180.0
t_end   = 1320.0
sub_hist = [h for h in history if t_start <= h['time'] <= t_end]

# Plot with Matplotlib in ModelSim Dark Theme style
fig, ax = plt.subplots(figsize=(16, 10), facecolor='#1e1e1e')
ax.set_facecolor('#141414')

signals = [
    ('clk', '1-bit', '#00ffcc'),
    ('start_i', '1-bit', '#ffff00'),
    ('busy_o', '1-bit', '#ffff00'),
    ('reg_cs_n', '1-bit', '#00e5ff'),
    ('reg_wr_n', '1-bit', '#ff3366'),
    ('reg_rd_n', '1-bit', '#ffaa00'),
    ('reg_db_oe', '1-bit', '#33ccff'),
    ('db_bus', 'bus', '#00ff66'),
    ('data_o', 'bus', '#ff66ff'),
    ('done_o', '1-bit', '#ffff00'),
]

num_sigs = len(signals)
y_spacing = 1.6

# Process discrete events into step segments
for idx, (sig_name, sig_type, color) in enumerate(signals):
    y_base = (num_sigs - 1 - idx) * y_spacing
    
    # Label
    ax.text(t_start - 35, y_base + 0.35, sig_name, color=color, fontsize=11, fontweight='bold', ha='right', va='center')
    
    # Draw horizontal baseline
    ax.axhline(y_base, color='#2a2a2a', linestyle='--', linewidth=0.5)

    times = [h['time'] for h in sub_hist]
    
    if sig_type == '1-bit':
        vals = [h[sig_name] for h in sub_hist]
        # Step plot
        t_steps = []
        v_steps = []
        for i in range(len(times)):
            t_steps.append(times[i])
            v_steps.append(y_base + vals[i] * 0.8)
        
        ax.step(t_steps, v_steps, where='post', color=color, linewidth=1.8)
    
    elif sig_type == 'bus':
        # Draw bus segments
        # Find transitions
        seg_start = times[0]
        cur_val = sub_hist[0][sig_name]
        
        bus_segments = []
        for i in range(1, len(sub_hist)):
            val = sub_hist[i][sig_name]
            if val != cur_val:
                bus_segments.append((seg_start, times[i], cur_val))
                seg_start = times[i]
                cur_val = val
        bus_segments.append((seg_start, times[-1], cur_val))
        
        for t0, t1, bval in bus_segments:
            if t1 - t0 <= 0: continue
            if bval == 'Hi-Z':
                # Center dotted line for Hi-Z
                ax.plot([t0, t1], [y_base + 0.4, y_base + 0.4], color='#888888', linestyle=':', linewidth=1.5)
                if t1 - t0 > 25:
                    ax.text((t0 + t1)/2, y_base + 0.4, 'Hi-Z', color='#888888', fontsize=8, ha='center', va='center')
            else:
                # Bus hexagon box
                poly_x = [t0, t0+3, t1-3, t1, t1-3, t0+3, t0]
                poly_y = [y_base+0.4, y_base+0.8, y_base+0.8, y_base+0.4, y_base, y_base, y_base+0.4]
                ax.fill(poly_x, poly_y, color='#003311', alpha=0.5)
                ax.plot(poly_x, poly_y, color=color, linewidth=1.2)
                if t1 - t0 > 30:
                    ax.text((t0 + t1)/2, y_base + 0.4, bval, color='#ffffff', fontsize=8.5, fontweight='bold', ha='center', va='center')

# Annotate stages
ax.axvspan(200, 580, color='#00ffcc', alpha=0.06)
ax.text(390, num_sigs * y_spacing + 0.3, "【第一帧：写读取地址帧 16'h8200】", color='#00ffcc', fontsize=11, ha='center', fontweight='bold')

ax.axvspan(580, 860, color='#ffaa00', alpha=0.06)
ax.text(720, num_sigs * y_spacing + 0.3, "【第二帧：数据回读帧 16'h0208】", color='#ffaa00', fontsize=11, ha='center', fontweight='bold')

ax.axvspan(860, 1260, color='#ff3366', alpha=0.06)
ax.text(1060, num_sigs * y_spacing + 0.3, "【退出帧：写 16'h0000 恢复 ADC 模式】", color='#ff3366', fontsize=11, ha='center', fontweight='bold')

# Vertical guidelines for WR_N rising edge, RD_N sample, etc.
ax.axvline(320, color='#00ffcc', linestyle='--', linewidth=1.0, alpha=0.7)
ax.text(320, -0.5, "WR↑ (320ns)\n地址锁存", color='#00ffcc', fontsize=8.5, ha='center')

ax.axvline(700, color='#ffaa00', linestyle='--', linewidth=1.0, alpha=0.7)
ax.text(700, -0.5, "RD采样 (700ns)\n读出08", color='#ffaa00', fontsize=8.5, ha='center')

ax.axvline(1000, color='#ff3366', linestyle='--', linewidth=1.0, alpha=0.7)
ax.text(1000, -0.5, "WR↑ (1000ns)\n退出寄存器模式", color='#ff3366', fontsize=8.5, ha='center')

ax.axvline(1260, color='#ffff00', linestyle='--', linewidth=1.0, alpha=0.7)
ax.text(1260, -0.5, "done_o (1260ns)\n完成输出", color='#ffff00', fontsize=8.5, ha='center')

ax.set_xlim(t_start - 40, t_end + 30)
ax.set_ylim(-1.0, num_sigs * y_spacing + 1.0)
ax.set_xlabel("Time (ns)", color='#cccccc', fontsize=12, labelpad=10)
ax.set_title("ModelSim Actual Simulation Waveform: LHA6958H Register Read (Read Addr 0x02 -> Data 0x08)",
             color='#ffffff', fontsize=14, fontweight='bold', pad=25)

# Style tick labels
ax.tick_params(colors='#888888', which='both')
ax.set_yticks([])
for spine in ax.spines.values():
    spine.set_color('#333333')

plt.tight_layout()

out_png1 = "d:/FPGA/SoftWare-Project/lha6958h-pro/sim/wave_reg_read.png"
out_png2 = "C:/Users/LiJinTao/.gemini/antigravity/brain/d714e978-3b30-4d5a-8af9-fcaf020234ae/wave_reg_read.png"

plt.savefig(out_png1, dpi=200, bbox_inches='tight')
plt.savefig(out_png2, dpi=200, bbox_inches='tight')
print("Saved waveform plots successfully to:", out_png1, "and", out_png2)
