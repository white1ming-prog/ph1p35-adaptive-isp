# -*- coding: utf-8 -*-
"""用参考模型跑一遍 tb_vis_edge 的场景 (64x16, 每帧一条竖阶跃),
   得出新版 vis_edge_96 的正确期望, 用来更新 tb_vis_edge.v 的检查器。
   ModelSim 没有 license 跑不了 vsim, 所以期望值由这个模型给出。"""
import ref_edge_model as m

m.W, m.H, m.GRP = 64, 16, 16
TH = 60

def run(f):
    """一帧: 每一行都在第 f 列有一个 10 -> 220 的阶跃"""
    stream = []
    for fr in range(2):
        for y in range(m.H):
            for g in range(m.GRP):
                px = [10 if (g * 4 + p) < f else 220 for p in range(4)]
                stream.append((y, g, px, fr))
    return m.sim_new(stream, th=TH)

for f in range(16):
    out = run(f)
    # 取第二帧的中间行
    masks = set()
    for y in range(m.H):
        row = tuple(x for x in range(m.W) if out[y][x])
        masks.add(row)
    print('edge@%2d -> 第二帧各行白像素列: %s' % (f, sorted(masks)[:3]))
