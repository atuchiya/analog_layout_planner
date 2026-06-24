# Analog Layout Planner

## プロジェクト概要

SPICE ネットリストを入力として，集積回路のアナログレイアウト配置配線を行うグラフィカルツールです。
Python 標準ライブラリの `tkinter` のみで動作します。外部依存パッケージはありません。

## ファイル構成

```
layout_planner.py   # ツール本体（単一ファイル，約1350行）
CLAUDE.md           # このファイル
```

## 実行方法

```bash
python layout_planner.py
```

Python 3.8 以上，tkinter が必要です（標準ライブラリに含まれます）。

## 入力ファイル形式

SPICE ネットリストの素子行にレイアウト情報をコメントで付記した形式です。

```spice
Xname net1 net2 ... subckt_name [key=val ...] * W=<幅> H=<高さ> net1:<側面>:<レイヤー> ...
```

| フィールド | 内容 |
|-----------|------|
| `Xname` | インスタンス名（`X` で始まる） |
| `net1 net2 ...` | 接続するネット名（SPICE ノード名） |
| `subckt_name` | サブサーキット種別名 |
| `W=N H=N` | BoundingBox のグリッド単位サイズ |
| `net:<N\|S\|E\|W>:<M1\|M2\|...>` | ピンの配置辺とレイヤー |

### サンプルネットリスト

```spice
* Simple demo
X1 VDD GND A B INV_X1  * W=6 H=4 VDD:N:M1 GND:S:M1 A:W:M1 B:E:M1
X2 VDD GND B C INV_X1  * W=6 H=4 VDD:N:M1 GND:S:M1 B:W:M1 C:E:M1
X3 VDD GND A C D NAND2_X1 * W=8 H=6 VDD:N:M2 GND:S:M1 A:W:M1 C:W:M2 D:E:M1
X4 VDD GND D E BUF_X1  * W=6 H=4 VDD:N:M1 GND:S:M1 D:W:M1 E:E:M1
```

## アーキテクチャ

### データモデル（`layout_planner.py` 上部）

| クラス | 役割 |
|--------|------|
| `PinDef` | ピン1個の情報（名前・配置辺・レイヤー） |
| `Component` | 素子インスタンス（位置・サイズ・ピン群） |
| `Net` | ネット（接続リスト・優先度・強制レイヤー・パワーリングフラグ） |
| `RouteSegment` | 配線セグメント1本（始点・終点・レイヤー・ネット名） |
| `ComponentGroup` | 素子グループ（`gid` 連番・`members` リスト） |

### 主要関数・メソッド

| 関数/メソッド | 役割 |
|--------------|------|
| `parse_netlist(text)` | SPICE テキストを解析して `components`, `nets` を返す |
| `initial_placement(components)` | 素子を左→右・上→下の格子状に初期配置 |
| `astar_route(...)` | 3D グリッド（col × row × layer）上の A* 最短経路探索 |
| `path_to_segments(path, net_name)` | A* パスを `RouteSegment` リストに変換 |
| `merge_segments(segs)` | 同層・同方向の隣接セグメントを1本にマージ |
| `LayoutApp._build_pin_occupation()` | ピン位置の層ブロックマップを構築 |
| `LayoutApp._route_all()` | 全ネットを優先度順に配線（パワーリング優先） |
| `LayoutApp._route_power_ring(...)` | パワーリング（矩形リング＋各ピンからのコネクタ）を生成 |
| `LayoutApp._route_astar(...)` | 信号線の A* 配線（forced_layer 対応・フォールバック再試行） |
| `LayoutApp._check_layer_conflicts()` | 短絡（同層・同セル重複）を検出 |
| `LayoutApp._comp_group(comp)` | 素子が属するグループを返す（なければ `None`） |
| `LayoutApp._create_group()` | `_multi_sel` の素子から新規グループを作成 |
| `LayoutApp._disband_group(grp)` | グループを解除 |
| `LayoutApp._transform_group(grp, op)` | グループ全体を回転・反転（位置の幾何変換＋ピン side 変換） |
| `LayoutApp._apply_side_map(comp, mapping)` | 素子のピン配置辺を一括変換 |

### 配線アルゴリズム

```
_route_all() の実行順序:
  1. パワーリングネット（VDD/GND 等）を外側のリングとして配線
       └─ 複数のパワーネット: ring_idx × 2 グリッドずつ外側に配置
       └─ レイヤー: 上位レイヤーから順に自動割当
  2. 信号線を以下の優先度順に A* で配線
       └─ tier 0: 明示的優先度（priority 属性）
       └─ tier 1: forced_layer のみ指定（レイヤーを先取り）
       └─ tier 2: 制約なし
  3. 各ネット配線後に blocked3d へ追加（後続ネットは通過禁止）
```

### ピンのレイヤー占有ルール

`_build_pin_occupation()` で管理します。

- **通常ピン（M_n 指定）**: M1 〜 M_n の全層を占有
  → 他ネットの配線は M1 〜 M_n でそのピン位置を通過できない
- **パワーリングピン**: 全レイヤーを占有
  → ピン位置にビアスタック（M1 → リングレイヤー）が存在するため

### 回転・反転の座標変換

グループ変換 `_transform_group(grp, op)` は以下の座標変換を適用します。
`dx = comp.gx - rx1`，`dy = comp.gy - ry1`，`GW = rx2 - rx1`，`GH = ry2 - ry1`

| op | gx の新値 | gy の新値 | サイズ | ピン side 変換 |
|----|-----------|-----------|--------|----------------|
| `rotate_left` (CCW) | `rx1 + dy` | `ry1 + (GW - dx - w)` | w↔h 交換 | N→W→S→E→N |
| `rotate_right` (CW) | `rx1 + (GH - dy - h)` | `ry1 + dx` | w↔h 交換 | N→E→S→W→N |
| `flip_h` | `rx1 + (GW - dx - w)` | 不変 | 不変 | E↔W |
| `flip_v` | 不変 | `ry1 + (GH - dy - h)` | 不変 | N↔S |

単体素子の回転・反転も同じ side 変換 + w/h スワップを適用します。

### 描画の Z オーダー

```
グリッド（最下層）→ 素子 BoundingBox → グループ破線枠 → M1 配線 → M2 配線 → … → ネット名ラベル（最上層）
```

### 選択状態と表示色

| 状態 | 素子アウトライン |
|------|-----------------|
| 通常 | `#ccccdd`（細線） |
| 単体選択（`_selected_comp`） | `#ffff00` 黄色（太線） |
| Shift 複数選択中（`_multi_sel`） | `#00ffff` シアン（太線） |
| 選択グループのメンバー（`_selected_group`） | `#ffaa00` オレンジ（中線） |

グループの破線枠: 非選択 `#7788cc`，選択中 `#ffaa00`

## ユーザー操作

| 操作 | 動作 |
|------|------|
| 左クリック（素子上） | 素子を選択（グループ所属の場合はグループ全体を選択） |
| Shift + 左クリック | 複数の素子をシアンでハイライト選択（グループ化の準備） |
| ドラッグ（素子上） | 素子を移動 → 自動再配線（グループ所属の場合は全メンバーを一緒に移動） |
| 右クリック（素子上） | 素子コンテキストメニュー表示 |
| &nbsp;├ N素子をグループ化 | Shift 選択中の素子をグループ化（2素子以上必要） |
| &nbsp;└ グループ解除 | 選択グループを解除 |
| 右クリック（配線上） | 配線コンテキストメニュー表示 |
| &nbsp;├ 優先度を設定 | 数値が小さいほど高優先度で最短配線 |
| &nbsp;├ 配線レイヤーを変更 | レイヤーを固定 → 再配線 → 短絡チェック |
| &nbsp;└ パワーリングとして配線 | パワーリングモードのトグル |
| Ctrl + ホイール | ズームイン／アウト |
| ホイール | スクロール |
| ネット一覧クリック | 選択ネットをハイライト |

### 回転・反転ボタン（左パネル）

素子またはグループを選択した状態でボタンを押すと変換を適用します。

| ボタン | 動作 |
|--------|------|
| ↺ 左90° | 選択素子／グループを画面上で反時計回りに90°回転 |
| ↻ 右90° | 選択素子／グループを画面上で時計回りに90°回転 |
| ⇔ 左右反転 | 選択素子／グループを左右ミラー |
| ↕ 上下反転 | 選択素子／グループを上下ミラー |

## 設定項目

左パネルで随時変更可能。「設定」メニューからも開けます。

| 設定 | 説明 |
|------|------|
| Grid (px) | グリッド間隔（ピクセル，8〜80） |
| Layers | 配線層数（1〜6） |
| Layer colors | 各層の表示色（クリックでカラーピッカー） |

`LayoutApp` の属性として保持されます（`grid_px`, `num_layers`, `layer_colors`, `ring_margin`）。

## パワーリングの自動検出

ネット名が以下のパターンに一致すると自動的にパワーリングとして扱われます。

```
VDD, VCC, VPWR, AVDD, DVDD, VSS, GND, VGND, AVSS, DVSS, AGND, DGND（前方一致，大文字小文字不問）
```

`is_power_net_name(name)` 関数（`POWER_NET_RE` 正規表現）で判定。

## 開発上の注意点

### A* ルーターの制限

- 探索上限: `MAX_ITER = 100,000` イテレーション（タイムアウト防止）
- 上限に達した場合は `None` を返す
  - `_route_astar`: forced_layer なしで再試行 → それでも失敗なら未配線
  - `_route_power_ring`: L字配線フォールバック

### blocked3d の管理

```python
# (col, row, layer_idx) の set
# 以下を順次追加:
#   1. 全素子の内部セル（全層）
#   2. 各ネットの配線後にそのセルを追加（次のネットが回避）
```

`_blocked_for_net(net_name, blocked3d)` は「他ネットのピン占有セル」を追加した
ネット固有のブロックセットを返します（大きな set のコピーが発生するため，
大規模ネットリストでは性能のボトルネックになり得ます）。

### 短絡チェック

`_check_layer_conflicts()` は全配線セグメントを走査して同層・同セルの重複を検出します。
レイヤー変更時（`_set_net_layer`）に自動実行され，結果をステータスバーに表示します。

### グループの制約

- 既存グループに属する素子は別グループに重複登録できません（`_create_group` 内でフィルタ）
- グループはネットリスト再読み込み時（`_load_text`）にリセットされます
- グループ情報はファイルに保存されません（セッション内のみ有効）

## 今後の改善候補

- 大規模ネットリスト向けの配線高速化（blocked3d のコピーを避ける差分管理）
- Steiner ツリーによるマルチピンネットの最適化
- DRC（デザインルールチェック）の拡充
- レイアウト結果の保存・読み込み（グループ情報も含む）
- 素子の重なり検出と自動回避
- グループの入れ子（階層グループ）
