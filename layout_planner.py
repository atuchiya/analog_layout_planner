#!/usr/bin/env python3
"""Analog Layout Planner — IC placement & routing tool"""

import tkinter as tk
from tkinter import ttk, filedialog, simpledialog, colorchooser, messagebox
import re
from collections import defaultdict
import heapq

# ネット名でパワーネットを判定（大文字小文字不問）
POWER_NET_RE = re.compile(
    r'^(VDD|VCC|VPWR|AVDD|DVDD|VSS|GND|VGND|AVSS|DVSS|AGND|DGND).*$',
    re.IGNORECASE)

def is_power_net_name(name):
    return bool(POWER_NET_RE.match(name))

# ─── Data Model ──────────────────────────────────────────────────────────────

class PinDef:
    def __init__(self, name, side, layer):
        self.name = name    # net name this pin connects to
        self.side = side    # 'N'|'S'|'E'|'W'
        self.layer = layer  # 'M1','M2',…


class Component:
    def __init__(self, inst_name, subckt, net_names, pin_defs, width, height):
        self.inst_name = inst_name   # 'X1'
        self.subckt    = subckt      # 'NAND2'
        self.net_names = net_names   # ordered list of net names
        self.pin_defs  = pin_defs    # dict net_name→PinDef
        self.width     = width       # grid units
        self.height    = height      # grid units
        self.gx = 0
        self.gy = 0
        self.rect_id  = None
        self.pin_ids  = {}           # net_name→canvas_id

    def pin_gpos(self, net_name):
        """(col, row) grid position of pin, None if unknown."""
        pd = self.pin_defs.get(net_name)
        if pd is None:
            return None
        same = [n for n in self.net_names
                if n in self.pin_defs and self.pin_defs[n].side == pd.side]
        if net_name not in same:
            return None
        idx = same.index(net_name)
        n   = len(same)
        t   = (idx + 1) / (n + 1)
        if pd.side == 'N':
            return (self.gx + round(self.width * t),  self.gy)
        if pd.side == 'S':
            return (self.gx + round(self.width * t),  self.gy + self.height)
        if pd.side == 'W':
            return (self.gx,              self.gy + round(self.height * t))
        # E
        return     (self.gx + self.width, self.gy + round(self.height * t))

    def interior_cells(self):
        """Strictly interior grid points (blocked for routing)."""
        s = set()
        for c in range(self.gx + 1, self.gx + self.width):
            for r in range(self.gy + 1, self.gy + self.height):
                s.add((c, r))
        return s


class Net:
    def __init__(self, name):
        self.name          = name
        self.connections   = []   # [(Component, net_name), …]
        self.priority      = None # int or None
        self.forced_layer  = None # int layer index (0=M1) or None (auto)
        self.is_power_ring = False
        self.segments      = []   # [RouteSegment, …]


class RouteSegment:
    def __init__(self, x1, y1, x2, y2, layer, net_name):
        if x1 > x2 or (x1 == x2 and y1 > y2):
            x1, y1, x2, y2 = x2, y2, x1, y1
        self.x1, self.y1 = x1, y1
        self.x2, self.y2 = x2, y2
        self.layer    = layer
        self.net_name = net_name

    def length(self):
        return (self.x2 - self.x1) + (self.y2 - self.y1)

    def grid_points(self):
        if self.x1 == self.x2:
            return [(self.x1, r) for r in range(self.y1, self.y2 + 1)]
        return [(c, self.y1) for c in range(self.x1, self.x2 + 1)]


# ─── Parser ───────────────────────────────────────────────────────────────────

def parse_netlist(text):
    components = []
    nets = {}

    for raw in text.splitlines():
        line = raw.strip()
        if not line or line[0] in ('*', '.', '$'):
            continue

        comment = ''
        if '*' in line:
            i = line.index('*')
            comment = line[i + 1:]
            line    = line[:i].strip()

        parts = line.split()
        if not parts or parts[0][0].upper() != 'X':
            continue

        tokens = parts[1:]
        param_start = len(tokens)
        for i, t in enumerate(tokens):
            if '=' in t and not t.startswith('='):
                param_start = i
                break

        if param_start < 1:
            continue

        subckt    = tokens[param_start - 1]
        net_names = tokens[:param_start - 1]
        if not net_names:
            continue

        width  = 8
        height = 4
        m = re.search(r'\bW=(\d+(?:\.\d+)?)', comment)
        if m: width  = max(2, int(float(m.group(1))))
        m = re.search(r'\bH=(\d+(?:\.\d+)?)', comment)
        if m: height = max(2, int(float(m.group(1))))

        pin_defs = {}
        for tok in comment.split():
            m = re.match(r'(\S+):(N|S|E|W):(M\d+)$', tok)
            if m:
                pn, side, layer = m.groups()
                pin_defs[pn] = PinDef(pn, side, layer)

        sides_cycle = ['N', 'S', 'E', 'W']
        for i, nn in enumerate(net_names):
            if nn not in pin_defs:
                pin_defs[nn] = PinDef(nn, sides_cycle[i % 4], 'M1')

        comp = Component(parts[0], subckt, net_names, pin_defs, width, height)
        components.append(comp)
        for nn in net_names:
            if nn not in nets:
                nets[nn] = Net(nn)
            nets[nn].connections.append((comp, nn))

    return components, nets


# ─── Placement ───────────────────────────────────────────────────────────────

def initial_placement(components, gap=3, cols=4):
    x = gap; y = gap; row_h = 0; col = 0
    for comp in components:
        comp.gx = x; comp.gy = y
        row_h = max(row_h, comp.height)
        x += comp.width + gap
        col += 1
        if col >= cols:
            col = 0; x = gap; y += row_h + gap; row_h = 0


# ─── A* Router ───────────────────────────────────────────────────────────────

def astar_route(src, src_layer, dst, dst_layer, blocked3d, num_layers, max_c, max_r,
                forced_layer=None):
    """
    Route on layered grid.  blocked3d = set of (col,row,layer_idx).
    If forced_layer is set (int), routing is confined to that single layer (no vias).
    Returns path [(col,row,layer), …] or None.
    """
    sc, sr = src; dc, dr = dst

    # When a layer is forced, override src/dst layers
    if forced_layer is not None:
        src_layer = dst_layer = forced_layer

    def h(c, r): return abs(c - dc) + abs(r - dr)

    start = (sc, sr, src_layer)
    open_h = [(h(sc, sr), 0, start)]
    g_sc   = {start: 0}
    prev   = {start: None}
    vis    = set()
    goal   = (dc, dr, dst_layer)

    MAX_ITER = 100_000
    it = 0
    while open_h and it < MAX_ITER:
        it += 1
        f, g, cur = heapq.heappop(open_h)
        if cur in vis: continue
        vis.add(cur)
        c, r, lyr = cur
        if c == dc and r == dr and lyr == dst_layer:
            path = []
            nd = cur
            while nd is not None:
                path.append(nd); nd = prev[nd]
            path.reverse(); return path

        for dc2, dr2 in ((1,0),(-1,0),(0,1),(0,-1)):
            nc, nr = c+dc2, r+dr2
            if not (0 <= nc <= max_c and 0 <= nr <= max_r): continue
            nb = (nc, nr, lyr)
            if nb in blocked3d and nb != goal: continue
            ng = g + 1
            if ng < g_sc.get(nb, 10**9):
                g_sc[nb] = ng; prev[nb] = cur
                heapq.heappush(open_h, (ng + h(nc, nr), ng, nb))

        # Via: layer change — skip if forced to single layer
        if forced_layer is None:
            for dl in (-1, 1):
                nl = lyr + dl
                if not (0 <= nl < num_layers): continue
                nb = (c, r, nl)
                if nb in blocked3d and nb != goal: continue
                ng = g + 1
                if ng < g_sc.get(nb, 10**9):
                    g_sc[nb] = ng; prev[nb] = cur
                    heapq.heappush(open_h, (ng + h(c, r), ng, nb))
    return None


def path_to_segments(path, net_name):
    """Convert A* path to RouteSegments (unit horizontal/vertical steps only)."""
    segs = []
    for i in range(len(path) - 1):
        c1, r1, l1 = path[i]; c2, r2, l2 = path[i+1]
        if l1 == l2:
            segs.append(RouteSegment(c1, r1, c2, r2, f'M{l1+1}', net_name))
    return segs


def merge_segments(segs):
    """Merge collinear adjacent segments on the same layer/net into longer ones."""
    if not segs: return segs
    by_key = defaultdict(list)
    for s in segs:
        if s.x1 == s.x2:  # vertical
            by_key[('V', s.x1, s.layer, s.net_name)].append((s.y1, s.y2))
        else:              # horizontal
            by_key[('H', s.y1, s.layer, s.net_name)].append((s.x1, s.x2))

    result = []
    for (orient, fixed, layer, net_name), intervals in by_key.items():
        # merge overlapping/adjacent intervals
        intervals.sort()
        merged = [list(intervals[0])]
        for a, b in intervals[1:]:
            if a <= merged[-1][1] + 1:
                merged[-1][1] = max(merged[-1][1], b)
            else:
                merged.append([a, b])
        for a, b in merged:
            if orient == 'V':
                result.append(RouteSegment(fixed, a, fixed, b, layer, net_name))
            else:
                result.append(RouteSegment(a, fixed, b, fixed, layer, net_name))
    return result


# ─── Main Application ─────────────────────────────────────────────────────────

DEMO_NETLIST = """\
* Simple demo netlist
X1 VDD GND A B INV_X1 * W=6 H=4 VDD:N:M1 GND:S:M1 A:W:M1 B:E:M1
X2 VDD GND B C INV_X1 * W=6 H=4 VDD:N:M1 GND:S:M1 B:W:M1 C:E:M1
X3 VDD GND A C D NAND2_X1 * W=8 H=6 VDD:N:M2 GND:S:M1 A:W:M1 C:W:M2 D:E:M1
X4 VDD GND D E BUF_X1 * W=6 H=4 VDD:N:M1 GND:S:M1 D:W:M1 E:E:M1
"""

DEFAULT_LAYER_COLORS = [
    '#e74c3c',  # M1 red
    '#3498db',  # M2 blue
    '#2ecc71',  # M3 green
    '#f39c12',  # M4 orange
    '#9b59b6',  # M5 purple
    '#1abc9c',  # M6 teal
]


class LayoutApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Analog Layout Planner")
        self.root.geometry("1300x820")

        self.grid_px    = 28          # pixels per grid unit
        self.num_layers = 3
        self.layer_colors = list(DEFAULT_LAYER_COLORS)
        self.ring_margin  = 2         # grid units: gap between components and power ring

        self.components = []
        self.nets       = {}

        self._drag_comp   = None
        self._drag_off    = (0, 0)
        self._drag_moved  = False

        self._build_ui()
        self._load_text(DEMO_NETLIST)

    def _build_ui(self):
        menubar = tk.Menu(self.root)

        fm = tk.Menu(menubar, tearoff=0)
        fm.add_command(label="Open Netlist…",  command=self._open_file)
        fm.add_command(label="Paste Netlist…", command=self._paste_dialog)
        fm.add_separator()
        fm.add_command(label="Exit", command=self.root.quit)
        menubar.add_cascade(label="File", menu=fm)

        rm = tk.Menu(menubar, tearoff=0)
        rm.add_command(label="Re-route All", command=self._reroute_and_redraw)
        menubar.add_cascade(label="Route", menu=rm)

        sm = tk.Menu(menubar, tearoff=0)
        sm.add_command(label="Settings…", command=self._open_settings)
        menubar.add_cascade(label="Settings", menu=sm)

        self.root.config(menu=menubar)

        # ── left panel ──
        left = ttk.Frame(self.root, width=200)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)
        left.pack_propagate(False)

        ttk.Label(left, text="Nets", font=('Arial', 10, 'bold')).pack(anchor='w')
        self._net_list = tk.Listbox(left, width=22, font=('Courier', 9))
        self._net_list.pack(fill=tk.BOTH, expand=True)
        self._net_list.bind('<<ListboxSelect>>', self._on_net_select)

        ttk.Separator(left, orient='horizontal').pack(fill='x', pady=4)
        ttk.Label(left, text="Grid (px):").pack(anchor='w')
        self._gs_var = tk.IntVar(value=self.grid_px)
        gs_spin = ttk.Spinbox(left, from_=8, to=80, textvariable=self._gs_var,
                               width=6, command=self._on_grid_change)
        gs_spin.pack(anchor='w')
        gs_spin.bind('<Return>', lambda e: self._on_grid_change())

        ttk.Label(left, text="Layers:").pack(anchor='w', pady=(6, 0))
        self._nl_var = tk.IntVar(value=self.num_layers)
        nl_spin = ttk.Spinbox(left, from_=1, to=6, textvariable=self._nl_var,
                               width=6, command=self._on_layers_change)
        nl_spin.pack(anchor='w')

        self._color_frame = ttk.Frame(left)
        self._color_frame.pack(fill='x', pady=4)
        self._build_color_buttons()

        # ── canvas ──
        cf = ttk.Frame(self.root)
        cf.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(cf, bg='#12122a', cursor='crosshair')
        hbar = ttk.Scrollbar(cf, orient=tk.HORIZONTAL, command=self.canvas.xview)
        vbar = ttk.Scrollbar(cf, orient=tk.VERTICAL,   command=self.canvas.yview)
        self.canvas.config(xscrollcommand=hbar.set, yscrollcommand=vbar.set)
        hbar.pack(side=tk.BOTTOM, fill=tk.X)
        vbar.pack(side=tk.RIGHT,  fill=tk.Y)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind('<Button-1>',        self._on_click)
        self.canvas.bind('<B1-Motion>',       self._on_drag)
        self.canvas.bind('<ButtonRelease-1>', self._on_release)
        self.canvas.bind('<Button-3>',        self._on_right_click)
        self.canvas.bind('<MouseWheel>',      self._on_wheel)

        # status bar
        self._status = tk.StringVar(value="Ready. Load a netlist or use the demo.")
        ttk.Label(self.root, textvariable=self._status,
                  relief=tk.SUNKEN, anchor='w').pack(
            side=tk.BOTTOM, fill=tk.X)

    def _build_color_buttons(self):
        for w in self._color_frame.winfo_children():
            w.destroy()
        ttk.Label(self._color_frame, text="Layer colors:").pack(anchor='w')
        for i in range(self.num_layers):
            color = self.layer_colors[i] if i < len(self.layer_colors) else '#ffffff'
            b = tk.Button(self._color_frame, text=f'  M{i+1}  ',
                          bg=color, fg='white',
                          relief=tk.FLAT, bd=2,
                          command=lambda idx=i: self._pick_color(idx))
            b.pack(fill='x', pady=1)

    def _pick_color(self, idx):
        cur = self.layer_colors[idx] if idx < len(self.layer_colors) else '#ffffff'
        c = colorchooser.askcolor(color=cur, title=f'M{idx+1} color')[1]
        if c:
            while len(self.layer_colors) <= idx:
                self.layer_colors.append('#ffffff')
            self.layer_colors[idx] = c
            self._build_color_buttons()
            self._draw_wires()

    def _on_grid_change(self):
        self.grid_px = max(8, min(80, self._gs_var.get()))
        self._gs_var.set(self.grid_px)
        self.redraw()

    def _on_layers_change(self):
        self.num_layers = max(1, min(6, self._nl_var.get()))
        self._nl_var.set(self.num_layers)
        self._build_color_buttons()
        self._reroute_and_redraw()

    # ── file / text loading ──────────────────────────────────────────────────

    def _open_file(self):
        path = filedialog.askopenfilename(
            filetypes=[('SPICE', '*.sp *.cir *.net *.spice *.spi'),
                       ('All', '*.*')])
        if not path:
            return
        with open(path, encoding='utf-8', errors='replace') as f:
            self._load_text(f.read())

    def _paste_dialog(self):
        dlg = tk.Toplevel(self.root)
        dlg.title("Paste Netlist")
        dlg.geometry("600x400")
        t = tk.Text(dlg, font=('Courier', 10))
        t.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        t.insert('1.0', DEMO_NETLIST)
        def ok():
            self._load_text(t.get('1.0', tk.END))
            dlg.destroy()
        ttk.Button(dlg, text="Load", command=ok).pack(pady=4)

    def _load_text(self, text):
        self.components, self.nets = parse_netlist(text)
        if not self.components:
            messagebox.showinfo("Info", "No X-elements found in netlist.")
            return
        # パワーネットの自動検出
        for name, net in self.nets.items():
            if is_power_net_name(name):
                net.is_power_ring = True
        initial_placement(self.components)
        self._rebuild_net_list()
        self._reroute_and_redraw()
        power_count = sum(1 for n in self.nets.values() if n.is_power_ring)
        self._status.set(
            f"{len(self.components)} components, {len(self.nets)} nets "
            f"({power_count} power rings) loaded.")

    def _rebuild_net_list(self):
        self._net_list.delete(0, tk.END)
        for name, net in sorted(self.nets.items()):
            prio  = f"P{net.priority}" if net.priority is not None else ""
            layer = f"M{net.forced_layer+1}" if net.forced_layer is not None else ""
            ring  = "Ring" if net.is_power_ring else ""
            badge = " ".join(filter(None, [ring, prio, layer]))
            label = f"{name}" + (f" [{badge}]" if badge else "")
            self._net_list.insert(tk.END, label)

    def _on_net_select(self, _):
        sel = self._net_list.curselection()
        if not sel:
            return
        txt = self._net_list.get(sel[0])
        net_name = txt.split()[0]
        self._highlight_net(net_name)

    def _highlight_net(self, name):
        self.canvas.itemconfig('wire', width=2)
        self.canvas.itemconfig(f'wire_net_{name}', width=5)

    # ── routing ─────────────────────────────────────────────────────────────

    def _comp_blocked(self):
        """Set of (col,row) strictly inside any component."""
        b = set()
        for comp in self.components:
            b |= comp.interior_cells()
        return b

    def _bounds(self):
        if not self.components:
            return 60, 60
        mc = max(c.gx + c.width  for c in self.components) + 15
        mr = max(c.gy + c.height for c in self.components) + 15
        return mc, mr

    def _build_pin_occupation(self):
        """
        各ピンの grid 位置 (col, row) について占有レイヤーを記録する。

        - 通常ピン: M1 ～ 指定レイヤー Mn（layer_idx = 0..n-1）を占有
        - パワーリングピン: M1 ～ リングまでのビアスタックが存在するため
          全レイヤー（0 .. num_layers-1）を占有とする

        戻り値: dict[(col, row, layer_idx)] -> set[net_name]
        """
        occ = defaultdict(set)
        for comp in self.components:
            for nn in comp.net_names:
                pos = comp.pin_gpos(nn)
                if pos is None:
                    continue
                col, row = pos
                pd = comp.pin_defs.get(nn)
                if pd is None:
                    continue
                m = re.match(r'M(\d+)', pd.layer)
                if not m:
                    continue
                top = int(m.group(1)) - 1  # 0-origin

                net = self.nets.get(nn)
                if net and net.is_power_ring:
                    # ビアスタック（ピン→リング）が全層を貫通するため全層ブロック
                    layers = range(self.num_layers)
                else:
                    # 通常ピン: M1 ～ 指定レイヤーのみ
                    layers = range(top + 1)

                for l in layers:
                    occ[(col, row, l)].add(nn)
        return dict(occ)

    def _blocked_for_net(self, net_name, blocked3d):
        """
        blocked3d に「他ネットのピン占有セル」を加えた集合を返す。
        自ネットのピンセルは追加しない（到達可能なまま）。
        """
        extra = {cell for cell, owners in self._pin_occ.items()
                 if net_name not in owners}
        return blocked3d | extra

    def _reroute_and_redraw(self):
        self._route_all()
        self.redraw()
        self._rebuild_net_list()

    def _route_all(self):
        for net in self.nets.values():
            net.segments = []

        comp_int = self._comp_blocked()
        max_c, max_r = self._bounds()
        nl = self.num_layers

        # ピン占有マップを構築（ルーティング中は参照のみ）
        self._pin_occ = self._build_pin_occupation()

        # blocked3d: cells occupied by routed wires of already-routed nets
        blocked3d = set()
        for (c, r) in comp_int:
            for l in range(nl):
                blocked3d.add((c, r, l))

        def _commit(net, segs):
            net.segments = segs
            for seg in segs:
                li = int(seg.layer[1:]) - 1
                for (c, r) in seg.grid_points():
                    blocked3d.add((c, r, li))

        # ── 1) パワーリングネットを先にルーティング ──────────────────────────
        power_nets = sorted(
            [n for n in self.nets.values() if n.is_power_ring],
            key=lambda n: n.name)

        for ring_idx, net in enumerate(power_nets):
            # リング間隔: 1つ目は ring_margin、以降は2グリッドずつ外側
            margin     = self.ring_margin + ring_idx * 2
            layer_idx  = self._power_ring_layer(net, ring_idx, len(power_nets))
            segs = self._route_power_ring(net, margin, layer_idx,
                                          blocked3d, max_c, max_r)
            segs = merge_segments(segs)
            _commit(net, segs)

        # ── 2) 信号線を優先度順にルーティング ───────────────────────────────
        def net_sort_key(net):
            # 配線順:
            #   tier 0: 明示的な優先度あり（数値が小さいほど先）
            #   tier 1: forced_layer のみ指定（レイヤーを先取りするため早めに配線）
            #   tier 2: 制約なし
            if net.priority is not None:
                return (0, net.priority,
                        0 if net.forced_layer is not None else 1,
                        net.name)
            elif net.forced_layer is not None:
                return (1, 0, 0, net.name)
            else:
                return (2, 0, 0, net.name)

        for net in sorted(self.nets.values(), key=net_sort_key):
            if net.is_power_ring or len(net.connections) < 2:
                continue
            pins = self._pin_positions(net)
            if not pins:
                continue
            segs = self._route_astar(net, pins, blocked3d, nl, max_c, max_r)
            segs = merge_segments(segs)
            _commit(net, segs)

    def _pin_positions(self, net):
        """[(col, row, layer_idx), …]  — respects net.forced_layer."""
        result = []
        for comp, nn in net.connections:
            pos = comp.pin_gpos(nn)
            if pos is None:
                continue
            if net.forced_layer is not None:
                l = net.forced_layer
            else:
                pd = comp.pin_defs.get(nn)
                l = 0
                if pd:
                    m = re.match(r'M(\d+)', pd.layer)
                    if m: l = int(m.group(1)) - 1
            result.append((pos[0], pos[1], l))
        return result

    def _route_astar(self, net, pins, blocked3d, nl, max_c, max_r):
        """
        A* routing connecting all pins to pin[0], respecting net.forced_layer.
        forced_layer で失敗した場合はレイヤー制約を外して再試行する。
        それでも失敗した場合はそのセグメントを未配線のままにする（短絡を生まない）。
        """
        segs = []
        fl   = net.forced_layer
        x0, y0, l0 = pins[0]
        eff = self._blocked_for_net(net.name, blocked3d)

        for x1, y1, l1 in pins[1:]:
            # ── 第1試行: forced_layer を守って A* ──────────────────────────
            path = astar_route((x0, y0), l0, (x1, y1), l1,
                               eff, nl, max_c, max_r,
                               forced_layer=fl)

            if path is None and fl is not None:
                # ── 第2試行: forced_layer 制約を外して A* ──────────────────
                # （指定レイヤーが混雑している場合の救済）
                path = astar_route((x0, y0), l0, (x1, y1), l1,
                                   eff, nl, max_c, max_r,
                                   forced_layer=None)

            if path is not None:
                segs.extend(path_to_segments(path, net.name))
            # path が None のままなら短絡を避けるため未配線のまま
        return segs

    def _power_ring_layer(self, net, ring_idx, total):
        """パワーリングのレイヤーを決定する（上位レイヤーから割当）。"""
        if net.forced_layer is not None:
            return net.forced_layer
        # 最上位レイヤーから順に割り当て（0 origin）
        return max(0, self.num_layers - 1 - ring_idx)

    def _route_power_ring(self, net, margin, layer_idx, blocked3d, max_c, max_r):
        """
        レイアウト全体を囲むリングを生成し、各ピンを最短経路でリングに接続する。
        connectorルーティング時点では blocked3d にリングセルを含めないため、
        A* がリング上の任意の点に到達できる。
        """
        if not self.components:
            return []

        rx1 = max(0, min(c.gx             for c in self.components) - margin)
        ry1 = max(0, min(c.gy             for c in self.components) - margin)
        rx2 =        max(c.gx + c.width   for c in self.components) + margin
        ry2 =        max(c.gy + c.height  for c in self.components) + margin

        layer = f'M{layer_idx + 1}'
        ring_segs = [
            RouteSegment(rx1, ry1, rx2, ry1, layer, net.name),  # 上辺
            RouteSegment(rx1, ry2, rx2, ry2, layer, net.name),  # 下辺
            RouteSegment(rx1, ry1, rx1, ry2, layer, net.name),  # 左辺
            RouteSegment(rx2, ry1, rx2, ry2, layer, net.name),  # 右辺
        ]

        # リングセル集合（コネクタ routing 時は blocked3d から除外して通行可）
        ring_cells_3d = set()
        for seg in ring_segs:
            for (c, r) in seg.grid_points():
                ring_cells_3d.add((c, r, layer_idx))

        # コネクタ A* 用ブロック:
        #   - リングセルは自ネットなので除外（到達可能）
        #   - 他ネットのピン占有セルは追加（通過禁止）
        passable_blocked = self._blocked_for_net(net.name, blocked3d) - ring_cells_3d

        connector_segs = []
        for comp, nn in net.connections:
            pos = comp.pin_gpos(nn)
            if pos is None:
                continue
            px, py = pos

            # リング周上の最近傍点
            cx, cy = self._closest_ring_point(px, py, rx1, ry1, rx2, ry2)

            # A* でコネクタを引く（レイヤー固定）
            path = astar_route(
                (px, py), layer_idx, (cx, cy), layer_idx,
                passable_blocked, self.num_layers, max_c, max_r,
                forced_layer=layer_idx)

            if path:
                connector_segs.extend(path_to_segments(path, net.name))
            else:
                # フォールバック: L字配線
                if px != cx:
                    connector_segs.append(
                        RouteSegment(px, py, cx, py, layer, net.name))
                if py != cy:
                    connector_segs.append(
                        RouteSegment(cx, py, cx, cy, layer, net.name))

        return ring_segs + connector_segs

    def _closest_ring_point(self, px, py, rx1, ry1, rx2, ry2):
        """矩形リング周上でマンハッタン距離が最小の点を返す。"""
        candidates = [
            (max(rx1, min(rx2, px)), ry1),         # 上辺
            (max(rx1, min(rx2, px)), ry2),         # 下辺
            (rx1, max(ry1, min(ry2, py))),         # 左辺
            (rx2, max(ry1, min(ry2, py))),         # 右辺
        ]
        return min(candidates, key=lambda p: abs(p[0] - px) + abs(p[1] - py))

    # ── drawing ──────────────────────────────────────────────────────────────

    def _g2p(self, gc, gr):
        gs = self.grid_px
        return gc * gs, gr * gs

    def _p2g(self, px, py):
        gs = self.grid_px
        return round(px / gs), round(py / gs)

    def _layer_color(self, layer_name):
        m = re.match(r'M(\d+)', layer_name)
        idx = (int(m.group(1)) - 1) if m else 0
        while len(self.layer_colors) <= idx:
            self.layer_colors.append('#ffffff')
        return self.layer_colors[idx]

    def redraw(self):
        self.canvas.delete('all')
        self._draw_grid()
        self._draw_components()   # 素子を先に描く（最下層）
        self._draw_wires()        # 配線を後から描く（素子の上）
        self.canvas.config(scrollregion=self.canvas.bbox('all') or (0, 0, 800, 600))

    def _draw_grid(self):
        gs = self.grid_px
        w  = max(self.canvas.winfo_width(),  1200)
        h  = max(self.canvas.winfo_height(), 800)
        extent_c, extent_r = self._bounds()
        W = max(w, extent_c * gs + gs * 5)
        H = max(h, extent_r * gs + gs * 5)
        for x in range(0, W + gs, gs):
            self.canvas.create_line(x, 0, x, H, fill='#1e1e3a', tags='grid')
        for y in range(0, H + gs, gs):
            self.canvas.create_line(0, y, W, y, fill='#1e1e3a', tags='grid')

    def _draw_wires(self):
        self.canvas.delete('wire')
        self.canvas.delete('wire_label')
        gs = self.grid_px

        # レイヤーごとに昇順で描画（M1が最下層、Mnが最上層）
        # まず全セグメントをレイヤー別に分類
        by_layer = defaultdict(list)   # layer_idx -> [(net, seg), …]
        for net in self.nets.values():
            for seg in net.segments:
                m = re.match(r'M(\d+)', seg.layer)
                layer_idx = int(m.group(1)) - 1 if m else 0
                by_layer[layer_idx].append((net, seg))

        for layer_idx in sorted(by_layer.keys()):
            for net, seg in by_layer[layer_idx]:
                color     = self._layer_color(seg.layer)
                is_ring   = net.is_power_ring
                linewidth = 5 if is_ring else 3
                x1, y1 = self._g2p(seg.x1, seg.y1)
                x2, y2 = self._g2p(seg.x2, seg.y2)
                self.canvas.create_line(
                    x1, y1, x2, y2, fill=color, width=linewidth,
                    capstyle=tk.ROUND,
                    tags=('wire', f'wire_net_{net.name}', f'layer_{seg.layer}'))
                lbl = str(seg.length())
                mx, my = (x1 + x2) / 2, (y1 + y2) / 2
                self.canvas.create_text(mx, my - 7, text=lbl,
                    fill=color, font=('Arial', 6), tags='wire_label')

        # ネット名ラベルは全レイヤーの配線が描き終わった後に最前面へ
        for net in self.nets.values():
            if not net.segments:
                continue
            total = sum(s.length() for s in net.segments)
            s0 = net.segments[0]
            tx = (s0.x1 + s0.x2) / 2 * gs
            ty = (s0.y1 + s0.y2) / 2 * gs - 16
            prio_str = f"[{net.priority}] " if net.priority is not None else ""
            color = self._layer_color(net.segments[0].layer)
            self.canvas.create_text(tx, ty,
                text=f"{prio_str}{net.name}({total})",
                fill=color, font=('Arial', 7, 'bold'), tags='wire_label')

    def _draw_components(self):
        self.canvas.delete('comp')
        gs = self.grid_px
        PIN = max(3, gs // 7)
        for comp in self.components:
            px, py = self._g2p(comp.gx, comp.gy)
            pw, ph = comp.width * gs, comp.height * gs

            comp.rect_id = self.canvas.create_rectangle(
                px, py, px+pw, py+ph,
                fill='#1a1a40', outline='#ccccdd', width=2,
                tags=('comp', f'comp_{comp.inst_name}'))
            self.canvas.create_text(px+pw/2, py+ph/2 - 7,
                text=comp.inst_name, fill='#ffffff',
                font=('Arial', 9, 'bold'), tags='comp')
            self.canvas.create_text(px+pw/2, py+ph/2 + 9,
                text=comp.subckt, fill='#aaaaff',
                font=('Arial', 8), tags='comp')

            comp.pin_ids = {}
            for nn in comp.net_names:
                gpos = comp.pin_gpos(nn)
                if gpos is None: continue
                ppx, ppy = self._g2p(*gpos)
                pid = self.canvas.create_rectangle(
                    ppx-PIN, ppy-PIN, ppx+PIN, ppy+PIN,
                    fill='#e74c3c', outline='#ffffff', width=1,
                    tags=('comp', 'pin', f'comp_{comp.inst_name}'))
                comp.pin_ids[nn] = pid
                if gs >= 20:
                    side = comp.pin_defs[nn].side if nn in comp.pin_defs else 'N'
                    offsets = {'N': (0,-PIN-6), 'S': (0,PIN+6),
                               'W': (-PIN-6,0), 'E': (PIN+6,0)}
                    ox, oy = offsets.get(side, (0, -PIN-6))
                    self.canvas.create_text(ppx+ox, ppy+oy, text=nn,
                        fill='#ffbbbb', font=('Arial', 6), tags='comp')

    # ── interaction ──────────────────────────────────────────────────────────

    def _canvas_xy(self, event):
        return self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)

    def _comp_at(self, cx, cy):
        items = self.canvas.find_overlapping(cx-1, cy-1, cx+1, cy+1)
        for item in reversed(items):
            tags = self.canvas.gettags(item)
            for tag in tags:
                if tag.startswith('comp_'):
                    name = tag[5:]
                    for c in self.components:
                        if c.inst_name == name:
                            return c
        return None

    def _on_click(self, event):
        cx, cy = self._canvas_xy(event)
        comp = self._comp_at(cx, cy)
        if comp:
            gs = self.grid_px
            self._drag_comp  = comp
            self._drag_off   = (cx - comp.gx * gs, cy - comp.gy * gs)
            self._drag_moved = False

    def _on_drag(self, event):
        if self._drag_comp is None:
            return
        cx, cy = self._canvas_xy(event)
        gs = self.grid_px
        new_gx = max(0, round((cx - self._drag_off[0]) / gs))
        new_gy = max(0, round((cy - self._drag_off[1]) / gs))
        if new_gx != self._drag_comp.gx or new_gy != self._drag_comp.gy:
            self._drag_comp.gx = new_gx
            self._drag_comp.gy = new_gy
            self._drag_moved = True
            self._route_all()
            self.redraw()

    def _on_release(self, event):
        self._drag_comp = None

    def _on_right_click(self, event):
        cx, cy = self._canvas_xy(event)
        items = self.canvas.find_overlapping(cx-4, cy-4, cx+4, cy+4)
        for item in items:
            tags = self.canvas.gettags(item)
            if 'wire' in tags:
                net_name = next((t[9:] for t in tags if t.startswith('wire_net_')), None)
                if net_name and net_name in self.nets:
                    self._wire_menu(event, net_name)
                    return

    def _wire_menu(self, event, net_name):
        net = self.nets[net_name]
        menu = tk.Menu(self.root, tearoff=0)

        prio_str  = str(net.priority) if net.priority is not None else "なし"
        layer_str = f"M{net.forced_layer+1}" if net.forced_layer is not None else "自動"
        menu.add_command(
            label=f"Net: {net_name}  優先度:{prio_str}  レイヤー:{layer_str}",
            state=tk.DISABLED)
        menu.add_separator()

        # Priority
        menu.add_command(label="優先度を設定…",
                         command=lambda: self._set_priority(net_name))
        menu.add_command(label="優先度をクリア",
                         command=lambda: self._clear_priority(net_name))
        menu.add_separator()

        # Layer sub-menu
        lm = tk.Menu(menu, tearoff=0)
        lm.add_command(label="自動（制約なし）",
                       command=lambda: self._set_net_layer(net_name, None))
        for i in range(self.num_layers):
            cur_mark = " ✓" if net.forced_layer == i else ""
            lm.add_command(label=f"M{i+1}{cur_mark}",
                           command=lambda l=i: self._set_net_layer(net_name, l))
        menu.add_cascade(label="配線レイヤーを変更…", menu=lm)

        menu.add_separator()
        ring_label = ("パワーリングを無効化" if net.is_power_ring
                      else "パワーリングとして配線")
        menu.add_command(label=ring_label,
                         command=lambda: self._toggle_power_ring(net_name))

        menu.tk_popup(event.x_root, event.y_root)

    def _set_priority(self, net_name):
        v = simpledialog.askinteger("優先度設定",
            f"ネット '{net_name}' の優先度を入力\n（小さい数字ほど優先度が高い）:",
            minvalue=1, parent=self.root)
        if v is not None:
            self.nets[net_name].priority = v
            self._reroute_and_redraw()
            self._status.set(f"Net '{net_name}' 優先度 → {v}")

    def _clear_priority(self, net_name):
        self.nets[net_name].priority = None
        self._reroute_and_redraw()

    def _set_net_layer(self, net_name, layer_idx):
        net = self.nets[net_name]
        old_layer = net.forced_layer
        net.forced_layer = layer_idx

        lbl = f"M{layer_idx+1}" if layer_idx is not None else "自動"

        # レイヤー変更後に他ネットとの短絡が生じないよう全再配線
        self._status.set(f"Net '{net_name}' レイヤー固定 → {lbl}  [再配線中…]")
        self.root.update_idletasks()   # ステータスを即時描画
        self._reroute_and_redraw()

        # 短絡（同層・同セルの重複）の有無を確認してステータスに反映
        conflicts = self._check_layer_conflicts()
        if conflicts:
            detail = "、".join(f"{a}&{b} on {lyr}" for a, b, lyr in conflicts[:3])
            self._status.set(
                f"Net '{net_name}' → {lbl}  ⚠ 短絡の可能性: {detail}")
        else:
            self._status.set(f"Net '{net_name}' レイヤー固定 → {lbl}  [短絡なし]")

    def _toggle_power_ring(self, net_name):
        net = self.nets[net_name]
        net.is_power_ring = not net.is_power_ring
        self._reroute_and_redraw()
        state = "有効（パワーリング）" if net.is_power_ring else "無効（通常配線）"
        self._status.set(f"Net '{net_name}' → {state}")

    def _check_layer_conflicts(self):
        """
        異なるネットの配線セルが同一 (col, row, layer) を共有していないか確認する。
        戻り値: [(net_name_a, net_name_b, layer_str), …]  最大 5 件
        """
        cell_owner: dict[tuple, str] = {}
        conflicts = []
        for net in self.nets.values():
            for seg in net.segments:
                m = re.match(r'M(\d+)', seg.layer)
                l = int(m.group(1)) - 1 if m else 0
                for (c, r) in seg.grid_points():
                    key = (c, r, l)
                    existing = cell_owner.get(key)
                    if existing is None:
                        cell_owner[key] = net.name
                    elif existing != net.name:
                        entry = tuple(sorted([existing, net.name])) + (seg.layer,)
                        if entry not in conflicts:
                            conflicts.append(entry)
                        if len(conflicts) >= 5:
                            return conflicts
        return conflicts

    def _on_wheel(self, event):
        if event.state & 0x4:  # Ctrl+wheel → zoom
            gs = self.grid_px + (2 if event.delta > 0 else -2)
            gs = max(8, min(80, gs))
            self.grid_px = gs
            self._gs_var.set(gs)
            self.redraw()
        else:
            self.canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    def _open_settings(self):
        SettingsDialog(self.root, self)


# ─── Settings Dialog ──────────────────────────────────────────────────────────

class SettingsDialog(tk.Toplevel):
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.title("Settings")
        self.resizable(False, False)
        self.grab_set()

        row = 0
        ttk.Label(self, text="Grid size (px):").grid(row=row, column=0,
            padx=10, pady=6, sticky='w')
        self._gs = tk.IntVar(value=app.grid_px)
        ttk.Spinbox(self, from_=8, to=80, textvariable=self._gs,
                    width=8).grid(row=row, column=1, padx=6)

        row += 1
        ttk.Label(self, text="Number of layers:").grid(row=row, column=0,
            padx=10, pady=6, sticky='w')
        self._nl = tk.IntVar(value=app.num_layers)
        ttk.Spinbox(self, from_=1, to=6, textvariable=self._nl,
                    width=8).grid(row=row, column=1, padx=6)

        row += 1
        ttk.Label(self, text="Layer colors:").grid(row=row, column=0,
            columnspan=2, padx=10, pady=(8,2), sticky='w')

        self._color_btns = []
        for i in range(6):
            row += 1
            color = app.layer_colors[i] if i < len(app.layer_colors) else '#aaaaaa'
            b = tk.Button(self, text=f'  M{i+1}  ', bg=color, fg='white',
                          relief=tk.GROOVE,
                          command=lambda idx=i: self._pick(idx))
            b.grid(row=row, column=0, columnspan=2, padx=20, pady=1, sticky='ew')
            self._color_btns.append(b)

        row += 1
        bf = ttk.Frame(self); bf.grid(row=row, column=0, columnspan=2, pady=10)
        ttk.Button(bf, text="適用", command=self._apply).pack(side=tk.LEFT, padx=6)
        ttk.Button(bf, text="閉じる", command=self.destroy).pack(side=tk.LEFT, padx=6)

    def _pick(self, idx):
        cur = self.app.layer_colors[idx] if idx < len(self.app.layer_colors) else '#aaa'
        c = colorchooser.askcolor(color=cur, title=f'M{idx+1} color', parent=self)[1]
        if c:
            while len(self.app.layer_colors) <= idx:
                self.app.layer_colors.append('#ffffff')
            self.app.layer_colors[idx] = c
            self._color_btns[idx].config(bg=c)

    def _apply(self):
        self.app.grid_px    = max(8, min(80, self._gs.get()))
        self.app.num_layers = max(1, min(6, self._nl.get()))
        self.app._gs_var.set(self.app.grid_px)
        self.app._nl_var.set(self.app.num_layers)
        self.app._build_color_buttons()
        self.app._reroute_and_redraw()
        self.destroy()


# ─── Entry Point ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    root = tk.Tk()
    app  = LayoutApp(root)
    root.mainloop()
