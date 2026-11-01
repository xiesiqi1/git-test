# -*- coding: utf-8 -*-
"""
统一转弯验证 —— 抓出"每个半圆到底给了内轮/外轮多少 PWM"

为什么需要它:
    sim_H_track.py 的 run() 只报告"圈时 / 最大横偏 / 丢线", 看不到 L/R。
    而实车反馈的问题恰恰是"第一个弯 内0外60, 第二个弯 内15外40" ——
    这是【控制器输出】层面的不一致, 必须逐拍把 L/R 抓下来才看得见。

它做什么:
    复用 sim_H_track.py 的控制律(ctrl_step)与车辆模型, 自己跑一遍,
    在【两个半圆的中段】分别统计控制器下发的 内轮/外轮/差速 的中位数,
    直接回答:"两个弯统一了吗?"

用法:
    python tools/sim_turn_unify.py
"""
import math
import random
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_H_track as S          # noqa: E402

# ---- 赛道沿弧长的分段边界 (2 x 直线 + 2 x 半圆) ----
L_AB = S.STRAIGHT                      # 0 ~ 1.5            直线 AB
L_BC = L_AB + math.pi * S.R_CURVE      # 1.5 ~ 3.0708       半圆 BC
L_CD = L_BC + S.STRAIGHT               # 3.0708 ~ 4.5708    直线 CD
# 4.5708 ~ 6.1416 = 半圆 DA


def arc_mid(lo, hi, frac=0.30):
    """取一段弧的中间 frac 部分, 避开进/出弯的过渡拍"""
    d = hi - lo
    return lo + d * (0.5 - frac / 2.0), lo + d * (0.5 + frac / 2.0)


BC_LO, BC_HI = arc_mid(L_AB, L_BC)
DA_LO, DA_HI = arc_mid(L_CD, S.LAP)


def run_trace(**kw):
    """照抄 sim_H_track.run() 的车辆/运动学模型, 额外记录两个弯的 L/R"""
    C = S.make_ctrl(**kw)
    if S.NOISE > 0.0:
        random.seed(S.NOISE_SEED)

    cx = cy = th = 0.0
    hint = 0
    t = 0.0
    dist = 0.0
    lost = 0
    maxoff = 0.0
    a = 1.0 if C['tau'] <= 0 else min(1.0, S.DT / C['tau'])

    rec = {'BC': [], 'DA': []}        # 每项 = (inner, outer, |diff|)
    dstep_max = 0
    prev_diff = 0
    sat = 0
    n = 0

    while dist < S.LAP and t < 60.0:
        mask, hint = S.sensor_mask(cx, cy, th, hint)
        e = S.line_error(mask)
        if mask == 0:
            lost += 1
        L, R = S.ctrl_step(C, e)

        # --- 记录: 这个弯给了内轮/外轮多少 ---
        # 判"内/外"用差速符号: |L-R| 大的一侧是外轮
        if BC_LO <= dist <= BC_HI or DA_LO <= dist <= DA_HI:
            inner, outer = (L, R) if L < R else (R, L)
            slot = 'BC' if dist <= L_BC else 'DA'
            rec[slot].append((inner, outer, abs(C['last_diff']),
                              abs(e), abs(C['last_pred'])))

        # --- 差速剧烈程度 ---
        di = C['last_diff']
        if abs(di - prev_diff) > dstep_max:
            dstep_max = abs(di - prev_diff)
        prev_diff = di
        if abs(di) >= C['dmax']:
            sat += 1
        n += 1

        # --- 车辆模型(与 sim_H_track.run 逐行一致) ---
        C['vl'] += (S.wspeed(L) - C['vl']) * a
        C['vr'] += (S.wspeed(R) - C['vr']) * a
        vl, vr = C['vl'], C['vr']
        if S.YAW_DEAD > 0.0:
            dv = vr - vl
            cut = 2.0 * S.YAW_DEAD * S.K_V
            if dv > 0.0:
                dv = max(0.0, dv - cut)
            elif dv < 0.0:
                dv = min(0.0, dv + cut)
            mid = (vl + vr) * 0.5
            vl, vr = mid - dv * 0.5, mid + dv * 0.5
            if vl < 0.0:
                vl = 0.0
            if vr < 0.0:
                vr = 0.0
        v = (vl + vr) / 2.0
        w = (vr - vl) / S.D_WHEEL
        cx += v * math.cos(th) * S.DT
        cy += v * math.sin(th) * S.DT
        th += w * S.DT
        dist += v * S.DT
        _, off, _ = S.lateral(cx, cy, hint)
        maxoff = max(maxoff, abs(off))
        t += S.DT
        if abs(off) > 0.06:
            return dict(ok=False, t=t, lost=lost, maxoff=maxoff,
                        rec=rec, dstep=dstep_max,
                        satpct=100.0 * sat / max(1, n))

    return dict(ok=dist >= S.LAP, t=t, lost=lost, maxoff=maxoff,
                rec=rec, dstep=dstep_max, satpct=100.0 * sat / max(1, n))


def med(seq, idx):
    if not seq:
        return float('nan')
    v = sorted(x[idx] for x in seq)
    return v[len(v) // 2]


def mx(seq, idx):
    if not seq:
        return float('nan')
    return max(x[idx] for x in seq)


def show(label, **kw):
    r = run_trace(**kw)
    bc, da = r['rec']['BC'], r['rec']['DA']
    bci, bco, bcd = med(bc, 0), med(bc, 1), med(bc, 2)
    dai, dao, dad = med(da, 0), med(da, 1), med(da, 2)
    same = (abs(bci - dai) <= 3 and abs(bco - dao) <= 3)
    print("%-20s| 弯1 内%5.1f 外%5.1f | 弯2 内%5.1f 外%5.1f | %s | "
          "误差|e| %3.1f/%3.1f  预测|p| %4.1f/%4.1f | %5.2fs %5.1fmm 饱和%3.0f%% %s"
          % (label, bci, bco, dai, dao,
             "★一致" if same else "  不一致",
             med(bc, 3), med(da, 3), med(bc, 4), med(da, 4),
             r['t'], r['maxoff'] * 1000, r['satpct'],
             "" if r['ok'] else "  【出线】丢线%d" % r['lost']))
    return r


def main():
    print("统一转弯验证 —— 两个半圆各给了多少 PWM (取弯道中段的中位数)")
    print("赛道: %.1f m 直线 x2 + R%.2f m 半圆 x2, 周长 %.3f m, 探头前视 %.1f cm"
          % (S.STRAIGHT, S.R_CURVE, S.LAP, S.SENS_LONG * 100))
    print("内/外 = 同一拍里|差速|较小/较大那一侧")
    print("-" * 140)

    # ★ 2026-09-29: 对齐当前固件 —— 前视 20→30(治"两个弯打舵量不统一"),
    #   收油加绝对上限 6(治"弯道明显减速")。轮距已是实测 0.27 m。
    #   注意这里的数字会随这两项一起变, 看结论的【方向】而不是绝对值。
    common = dict(base=32, kp=14, kd=0, dl=3, db=0, minpct=60,
                  tki=150, tlim=16, ttau=50, boost=10, bcnt=3,
                  leadt=30, ltau=20, dslew=3, search=12, lostspd=20,
                  preserve=True, tau=S.TAU_MOTOR, slowmax=6)

    show("A 现状 dm32 sl50", **dict(common, dmax=32, slow=50))
    show("B 砍上限 dm20 sl50", **dict(common, dmax=20, slow=50))
    show("C 砍上限 dm15 sl30", **dict(common, dmax=15, slow=30))
    show("D 新档 dm13 sl30", **dict(common, dmax=13, slow=30))
    print("-" * 140)

    # 治根路线: 不砍上限, 只把"进弯的假预测值"掐掉 + 加大提前量
    show("E 现状+前视限幅4", **dict(common, dmax=32, slow=50, predmax=4))
    show("F 现状+前视30拍", **dict(common, dmax=32, slow=50, leadt=30))
    show("G 现状+前视30+限幅4", **dict(common, dmax=32, slow=50, leadt=30, predmax=4))
    show("H G更狠 前视40+限幅3", **dict(common, dmax=32, slow=50, leadt=40, predmax=3))
    print("-" * 140)

    # 带噪声看: 限幅/前视是不是把噪声也放大
    print()
    print("★ 噪声敏感性 (误判率 3%%, 固定种子可复现):")
    keep_n = S.NOISE
    S.NOISE = 0.03
    show("  现状 dm32", **dict(common, dmax=32, slow=50))
    show("  现状+限幅4", **dict(common, dmax=32, slow=50, predmax=4))
    show("  现状+前视30+限幅4", **dict(common, dmax=32, slow=50, leadt=30, predmax=4))
    S.NOISE = keep_n
    print("-" * 140)

    # 摩擦假设敏感性: 真车的偏航静摩擦是未知量, 结论必须在整个范围都成立
    print()
    print("★ 摩擦敏感性 (YAW_DEAD 未知, 5~15 都得成立):")
    keep = S.YAW_DEAD
    for yd in (5.0, 8.0, 12.0, 15.0):
        S.YAW_DEAD = yd
        print("   YAW_DEAD=%-5.0f" % yd, end="")
        for lab, kw in (("现状", dict(dmax=32, slow=50)),
                        ("砍13", dict(dmax=13, slow=30)),
                        ("限幅4", dict(dmax=32, slow=50, predmax=4)),
                        ("前视30", dict(dmax=32, slow=50, leadt=30))):
            r = run_trace(**dict(common, **kw))
            b = r['rec']['BC']
            print("  %s:%5.1f/%5.1f%5.2fs" %
                  (lab, med(b, 0), med(b, 1), r['t']), end="")
            if not r['ok']:
                print("(出线)", end="")
        print()
    S.YAW_DEAD = keep



if __name__ == "__main__":
    main()
