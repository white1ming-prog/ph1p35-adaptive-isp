# -*- coding: utf-8 -*-
"""生成 旧版 vs 新版 边缘检测对比图 (左: 原图 中: 旧算法 右: 新算法)"""
import ref_edge_model as m

W, H = m.W, m.H
GAP = 6

stream = m.make_stream(2)
img = [[m.pattern(x, y) for x in range(W)] for y in range(H)]
old = m.sim_old(stream)
new = m.sim_new(stream)

def panel(y, x):
    if x < W:
        return img[y][x]
    x -= W + GAP
    if x < W:
        return old[y][x]
    x -= W + GAP
    if 0 <= x < W:
        return new[y][x]
    return 255 if (x == -1 or x == W) else 220   # 分隔线

CW = W * 3 + GAP * 2
rows = []
for y in range(H):
    rows.append([panel(y, x) for x in range(CW)])

m.write_png('edge_compare.png', rows, CW, H)
print('edge_compare.png  宽 %d 高 %d  (左=原图 中=旧算法 右=新算法)' % (CW, H))
