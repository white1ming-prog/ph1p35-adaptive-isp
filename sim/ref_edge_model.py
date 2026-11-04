# -*- coding: utf-8 -*-
"""
vis_edge_96 周期精确参考模型 (Python), 用于在 ModelSim 许可证不可用时验证
新写的 Verilog 流水线: 对齐、阈值、滞后判定是否如预期。

同时跑一遍"当前在板上的旧版"算法做对照。

输出: sim_edge_cmp/input.png / old.png / new.png  + 终端指标
"""
import zlib, struct, random

W, H = 640, 360
GRP = W // 4
TH = 300

# ---------------------------------------------------------------- 合成图
def pattern(x, y):
    v = 40
    if (x % 40) < 3:
        v = 200                                   # 竖条 (强)
    if (y % 30) < 2:
        v = 180                                   # 横条 (强)
    if (y * 2 - 2) < x < (y * 2 + 2):
        v = 150                                   # 斜线
    dx = abs(x - 160); dy = abs(y - 90)
    d2 = dx * dx + dy * dy
    if 58 * 58 < d2 < 62 * 62:
        v = 160                                   # 圆环
    if 200 < x < 280 and 20 < y < 60:
        v = 54                                    # 弱边缘方块: 只有 14 级跳变
    return v

rnd = random.Random(20261031)

def make_stream(frames=2):
    """返回 [(y, g, [p0,p1,p2,p3]), ...], 每拍 4 个像素, 带 -3..+4 噪点"""
    s = []
    for f in range(frames):
        for y in range(H):
            for g in range(GRP):
                px = []
                for p in range(4):
                    v = pattern(g * 4 + p, y) + rnd.randint(-3, 4)
                    v = 0 if v < 0 else (255 if v > 255 else v)
                    px.append(v)
                s.append((y, g, px, f))
    return s

# ---------------------------------------------------------------- 工具
def byte(w, i):
    return (w >> (8 * i)) & 0xFF

def wsum(a, b, c):
    return 3 * a + 10 * b + 3 * c

def s13(x):                       # 13bit 有符号环绕
    x &= 0x1FFF
    return x - 0x2000 if x & 0x1000 else x

def abs13(x):
    return -x if (x & 0x1000) else x

# ---------------------------------------------------------------- 新版
def sim_new(stream, th=TH):
    line1 = [0] * GRP
    line2 = [0] * GRP
    flagm = [0] * GRP

    top_r = mid_r = cur_r = 0
    tprev_r = mprev_r = cprev_r = 0
    v1 = u1 = l1 = 0; x1 = y1 = 0

    ht = [0] * 4; hc = [0] * 4
    vm1_r = v0_r = v1r_r = v2_r = v3_r = vp4_r = 0
    v2a = u2a = l2a = 0; x2 = y2 = 0

    mag = [0] * 4
    v3 = u3 = l3 = 0; x3 = y3 = 0

    flag_d1 = flag_d2 = flag_d3 = 0
    pflag3 = 0

    x_grp = 0; y_pos = 0

    out = [[0] * W for _ in range(H)]

    for (y, g, px, f) in stream:
        tvalid = 1
        tuser = 1 if (y == 0 and g == 0) else 0
        tlast = 1 if (g == GRP - 1) else 0

        gray = [((v >> 2) + (v >> 1) + (v >> 2)) & 0xFF for v in px]

        top_word = line2[x_grp]
        mid_word = line1[x_grp]
        flag_q = flagm[x1]

        t_nxt = byte(top_word, 0)
        m_nxt = byte(mid_word, 0)
        c_nxt = gray[0]

        # 第二级 A: 加权和
        ht_c = [wsum(tprev_r, byte(top_r, 0), byte(top_r, 1)),
                wsum(byte(top_r, 0), byte(top_r, 1), byte(top_r, 2)),
                wsum(byte(top_r, 1), byte(top_r, 2), byte(top_r, 3)),
                wsum(byte(top_r, 2), byte(top_r, 3), t_nxt)]
        hc_c = [wsum(cprev_r, byte(cur_r, 0), byte(cur_r, 1)),
                wsum(byte(cur_r, 0), byte(cur_r, 1), byte(cur_r, 2)),
                wsum(byte(cur_r, 1), byte(cur_r, 2), byte(cur_r, 3)),
                wsum(byte(cur_r, 2), byte(cur_r, 3), c_nxt)]
        v_m1 = wsum(tprev_r, mprev_r, cprev_r)
        v_c0 = wsum(byte(top_r, 0), byte(mid_r, 0), byte(cur_r, 0))
        v_c1 = wsum(byte(top_r, 1), byte(mid_r, 1), byte(cur_r, 1))
        v_c2 = wsum(byte(top_r, 2), byte(mid_r, 2), byte(cur_r, 2))
        v_c3 = wsum(byte(top_r, 3), byte(mid_r, 3), byte(cur_r, 3))
        v_p4 = wsum(t_nxt, m_nxt, c_nxt)

        # 第二级 B: 梯度与幅值
        gx = [s13(hc[i] - ht[i]) for i in range(4)]
        gsrc = [vm1_r, v0_r, v1r_r, v2_r, v3_r, vp4_r]
        gy = [s13(gsrc[2] - gsrc[0]), s13(gsrc[3] - gsrc[1]),
              s13(gsrc[4] - gsrc[2]), s13(gsrc[5] - gsrc[3])]
        mag_c = [abs13(gx[i]) + abs13(gy[i]) for i in range(4)]

        # 第三级: 双阈值滞后
        th_hi = th
        th_lo = th >> 1
        st = [mag[i] >= th_hi for i in range(4)]
        wk = [mag[i] >= th_lo for i in range(4)]
        row_ok = (y3 >= 2) and (not u3)
        ok = [row_ok and x3 != 0, row_ok, row_ok, row_ok and x3 != GRP - 1]
        up_en = y3 >= 3
        st_nxt0 = mag_c[0] >= th_hi

        b = lambda v, i: (v >> i) & 1
        sup0 = (up_en and (b(flag_d3, 3) or b(flag_d2, 0) or b(flag_d2, 1))) or pflag3 or st[1]
        sup1 = (up_en and (b(flag_d2, 0) or b(flag_d2, 1) or b(flag_d2, 2))) or st[2]
        sup2 = (up_en and (b(flag_d2, 1) or b(flag_d2, 2) or b(flag_d2, 3))) or st[3]
        sup3 = (up_en and (b(flag_d2, 2) or b(flag_d2, 3) or b(flag_d1, 0))) or st_nxt0
        e = [0] * 4
        e[0] = ok[0] and (st[0] or (wk[0] and sup0))
        e[1] = ok[1] and (st[1] or (wk[1] and (sup1 or e[0])))
        e[2] = ok[2] and (st[2] or (wk[2] and (sup2 or e[1])))
        e[3] = ok[3] and (st[3] or (wk[3] and (sup3 or e[2])))

        if v3:
            row = out[y3]
            for j in range(4):
                row[x3 * 4 + j] = 255 if e[j] else 0

        # ---- 寄存器更新 (非阻塞: 全部用本拍的旧值, 先算新值再一起写回) ----
        new_top_r, new_mid_r = top_word, mid_word
        new_cur_r = (gray[3] << 24) | (gray[2] << 16) | (gray[1] << 8) | gray[0]
        new_tprev, new_mprev, new_cprev = byte(top_r, 3), byte(mid_r, 3), byte(cur_r, 3)
        new_v1, new_u1, new_l1 = tvalid, tuser, tlast
        new_x1, new_y1 = x_grp, y_pos

        new_ht, new_hc = ht_c, hc_c
        new_vm1, new_v0, new_vc1 = v_m1, v_c0, v_c1
        new_vc2, new_vc3, new_vp4 = v_c2, v_c3, v_p4
        new_v2a, new_u2a, new_l2a = v1, u1, l1
        new_x2, new_y2 = x1, y1

        new_mag = mag_c
        new_v3, new_u3, new_l3 = v2a, u2a, l2a
        new_x3, new_y3 = x2, y2

        new_flag_d1, new_flag_d2, new_flag_d3 = flag_q, flag_d1, flag_d2
        new_pflag3 = e[3]

        top_r, mid_r, cur_r = new_top_r, new_mid_r, new_cur_r
        tprev_r, mprev_r, cprev_r = new_tprev, new_mprev, new_cprev
        v1, u1, l1 = new_v1, new_u1, new_l1
        x1, y1 = new_x1, new_y1
        ht, hc = new_ht, new_hc
        vm1_r, v0_r, v1r_r = new_vm1, new_v0, new_vc1
        v2_r, v3_r, vp4_r = new_vc2, new_vc3, new_vp4
        v2a, u2a, l2a = new_v2a, new_u2a, new_l2a
        x2, y2 = new_x2, new_y2
        mag = new_mag
        v3, u3, l3 = new_v3, new_u3, new_l3
        x3, y3 = new_x3, new_y3
        flag_d1, flag_d2, flag_d3 = new_flag_d1, new_flag_d2, new_flag_d3
        pflag3 = new_pflag3

        if v3:
            flagm[x3] = (e[3] << 3) | (e[2] << 2) | (e[1] << 1) | e[0]

        line2[x_grp] = mid_word
        line1[x_grp] = (gray[3] << 24) | (gray[2] << 16) | (gray[1] << 8) | gray[0]

        if tuser:
            x_grp, y_pos = 1, 0
        elif x_grp == GRP - 1:
            x_grp = 0
            y_pos = 0 if y_pos == H - 1 else y_pos + 1
        else:
            x_grp += 1

    return out

# ---------------------------------------------------------------- 旧版 (当前烧在板上的算法)
def sim_old(stream, th=TH):
    """照 isp_top.v 里内嵌的旧 vis_edge_96 行为建模:
       4 像素步长窗口 / mag = gx*gx+gy*gy 截断到 15bit / 每拍只输出 2 个像素"""
    lb0 = [0] * GRP          # 16bit
    lb1 = [0] * GRP
    p00 = p01 = p02 = 0
    p10 = p11 = p12 = 0
    p20 = p21 = p22 = 0
    g0_r = g1_r = 0
    v1_r = tlast_r = tuser_r = 0
    cnt_x = 0; cnt_y = 0
    out = [[0] * W for _ in range(H)]
    oline = 0; obeats = 0

    for (y, g, px, f) in stream:
        gray0 = ((px[0] >> 2) + (px[0] >> 1) + (px[0] >> 3)) & 0xFF
        gray1 = ((px[1] >> 2) + (px[1] >> 1) + (px[1] >> 3)) & 0xFF
        row2_out = lb1[cnt_x]
        row1_out = lb0[cnt_x]

        gx = (-3 * p00 - 10 * p01 - 3 * p02) + (3 * p20 + 10 * p21 + 3 * p22)
        gy = (-3 * p00 - 10 * p10 - 3 * p20) + (3 * p02 + 10 * p12 + 3 * p22)
        mag = (gx * gx + gy * gy) & 0x7FFF          # 15bit 截断 <- 噪点来源
        edge = mag >= th
        pix_out = 255 if edge else 0

        # 输出 (延迟一拍), 每拍只有前 2 个像素有值
        if v1_r:
            if obeats < GRP and oline < H:
                out[oline][obeats * 4 + 0] = pix_out
                out[oline][obeats * 4 + 1] = pix_out
            obeats += 1
            if tlast_r:
                oline += 1
                obeats = 0

        # 寄存器
        lb1[cnt_x] = ((row1_out & 0xFF) << 8) | g0_r
        lb0[cnt_x] = (p11 << 8) | p01
        p00, p01, p02 = p01, p02, g0_r
        p10, p11, p12 = p11, p12, ((row1_out >> 8) & 0xFF)
        p20, p21, p22 = p21, p22, ((row2_out >> 8) & 0xFF)
        g0_r, g1_r = gray0, gray1
        v1_r, tlast_r, tuser_r = 1, 1 if (g == GRP - 1) else 0, 1 if (y == 0 and g == 0) else 0
        if cnt_x == GRP - 1:
            cnt_x = 0
            if tlast_r:
                cnt_y += 1
        else:
            cnt_x += 1

    return out

# ---------------------------------------------------------------- PNG
def write_png(path, img, w, h):
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        for x in range(w):
            v = img[y][x]
            raw += bytes((v, v, v))
    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xFFFFFFFF)
    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(bytes(raw), 6))
    png += chunk(b'IEND', b'')
    open(path, 'wb').write(png)

# ---------------------------------------------------------------- 指标
def metrics(name, out):
    tot = sum(1 for y in range(H) for x in range(W) if out[y][x])
    # 孤立点: 8 邻域内没有别的边
    iso = 0
    for y in range(1, H - 1):
        for x in range(1, W - 1):
            if out[y][x]:
                n = 0
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx or dy:
                            n += 1 if out[y + dy][x + dx] else 0
                if n == 0:
                    iso += 1
    # 真正的平坦噪声区: 竖条在 x%40<3, 横条在 y%30<2, 圆环在 (160,90) r=60,
    # 斜线在 x=2y, 弱方块在 x 200..280 / y 20..60 -> 330..375 x 250..268 全避开
    flat = sum(1 for y in range(250, 268) for x in range(330, 358) if out[y][x])
    # 弱边缘方块的上边框 (y=21, x=200..280), 14 级跳变
    weak = sum(1 for x in range(201, 279) if out[21][x] or out[22][x])
    # 竖线: 第 200 列附近 (x=200 是 40 的倍数 -> 竖条左沿)
    vert = sum(1 for y in range(10, 350) if out[y][200] or out[y][201])
    print("%-6s 边像素=%6d  孤立噪点=%4d  平坦区误检=%4d  弱边保留=%3d/78  竖线覆盖=%3d/340"
          % (name, tot, iso, flat, weak, vert))

if __name__ == '__main__':
    stream = make_stream(2)
    img = [[pattern(x, y) for x in range(W)] for y in range(H)]
    write_png('input.png', img, W, H)
    o = sim_old(stream)
    write_png('old.png', o, W, H)
    n = sim_new(stream)
    write_png('new.png', n, W, H)
    metrics('旧版', o)
    metrics('新版', n)
    print("已写出 input.png / old.png / new.png")
