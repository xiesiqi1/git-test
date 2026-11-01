# -*- coding: utf-8 -*-
"""
弯道减速诊断 —— 抓出"每个弯里基础速度(base)到底掉到多少"

为什么需要它:
    实车反馈: "车跑到后面的转弯时, 明显减速"。
    而 sim_H_track.py / sim_turn_unify.py 都只看 圈时/横偏/内外轮,
    【看不到 base】。base 才是"减速"的直接体现。

它做什么:
    复用 sim_H_track.py 的控制律与车辆模型, 按【赛段】分别统计 base:
        AB 直线 / BC 半圆 / CD 直线 / DA 半圆
    给出每段的 base 中位数与最小值, 于是"弯道比直道慢多少"是算出来的,
    不是感觉出来的。

要看的关键数字:
    · 弯段 base 中位数 vs 直段 base 中位数  —— 差多少就是"弯道收了多少油"
    · 弯段 base 最小值                    —— 会不会掉到 CORNER_MIN 地板上
    · 圈时 / 横偏 / 丢线                  —— 提速有没有把车搞出线

用法:
    python tools/sim_corner_slow.py
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


def seg_of(dist):
    if dist < L_AB:
        return 'AB'
    if dist < L_BC:
        return 'BC'
    if dist < L_CD:
        return 'CD'
    return 'DA'


def run_trace(**kw):
    """照抄 sim_H_track.run() 的车辆/运动学模型, 额外按赛段记录 base"""
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

    # 每段存 (base, |diff|)：只在"没丢线"的拍上采, 免得 LOST_SPEED 混进来
    rec = {k: [] for k in ('AB', 'BC', 'CD', 'DA')}
    lost_by_seg = {k: 0 for k in ('AB', 'BC', 'CD', 'DA')}

    while dist < S.LAP and t < 60.0:
        mask, hint = S.sensor_mask(cx, cy, th, hint)
        e = S.line_error(mask)
        seg = seg_of(dist)
        if mask == 0:
            lost += 1
            lost_by_seg[seg] += 1
        L, R = S.ctrl_step(C, e)
        if mask != 0:
            rec[seg].append((C['last_base'], abs(C['last_diff']), abs(e)))

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
            return dict(ok=False, t=t, lost=lost, maxoff=maxoff, rec=rec,
                        lost_by_seg=lost_by_seg)

    return dict(ok=dist >= S.LAP, t=t, lost=lost, maxoff=maxoff, rec=rec,
                lost_by_seg=lost_by_seg)


def med(seq, idx=0):
    if not seq:
        return float('nan')
    v = sorted(x[idx] for x in seq)
    return v[len(v) // 2]


def mn(seq, idx=0):
    if not seq:
        return float('nan')
    return min(x[idx] for x in seq)


def show(label, **kw):
    r = run_trace(**kw)
    rec = r['rec']
    # 弯道 = BC 与 DA; 直道 = AB 与 CD。★ BC/DA 分开看才能回答
    # "是不是【后面的】弯更慢"(实车原话)。
    curve = rec['BC'] + rec['DA']
    straight = rec['AB'] + rec['CD']
    cb, cm = med(curve), mn(curve)
    sb = med(straight)
    drop = sb - cb if (sb == sb and cb == cb) else float('nan')
    print("%-18s| 直道 %5.1f | 弯BC %5.1f/%5.1f | 弯DA %5.1f/%5.1f | 弯降%4.1f "
          "| 弯|diff| %4.1f | %5.2fs %5.1fmm 丢%3d %s"
          % (label, sb,
             med(rec['BC']), mn(rec['BC']),
             med(rec['DA']), mn(rec['DA']),
             drop, med(curve, 1),
             r['t'], r['maxoff'] * 1000, r['lost'],
             "" if r['ok'] else "  【出线】"))
    return r


def main():
    print("弯道减速诊断 —— base = 本拍基础速度(已含弯道收油), 上/下 = 中位数/最小值")
    print("赛道: %.1f m 直线 x2 + R%.2f m 半圆 x2, 周长 %.3f m, 探头前视 %.1f cm"
          % (S.STRAIGHT, S.R_CURVE, S.LAP, S.SENS_LONG * 100))
    print("注意: 收油下限 CORNER_MIN = %d%% -> base 地板 = %.1f"
          % (60, 32 * 60 / 100))
    print("-" * 128)

    common = dict(base=32, kp=14, kd=0, dl=3, db=0, dmax=32, minpct=60,
                  tki=150, tlim=16, ttau=50, boost=10, bcnt=3,
                  leadt=30, ltau=20, dslew=3, search=12, lostspd=20,
                  preserve=True, tau=S.TAU_MOTOR)

    show("A 现状 sl50 无上限", **dict(common, slow=50))
    print("-" * 128)
    show("B sl50 +上限6", **dict(common, slow=50, slowmax=6))
    show("C sl35 +上限6", **dict(common, slow=35, slowmax=6))
    show("D sl30 +上限6", **dict(common, slow=30, slowmax=6))
    show("E sl30 +上限8", **dict(common, slow=30, slowmax=8))
    show("F sl25 +上限8", **dict(common, slow=25, slowmax=8))
    show("G sl30 无上限", **dict(common, slow=30))
    print("-" * 128)

    # 噪声敏感性: 收油变少 -> 弯里更快 -> 会不会更容易丢线
    print()
    print("★ 噪声敏感性 (误判率 3%%, 固定种子可复现):")
    keep_n = S.NOISE
    S.NOISE = 0.03
    show("  现状 sl50", **dict(common, slow=50))
    show("  sl50+上限6", **dict(common, slow=50, slowmax=6))
    show("  sl35+上限6", **dict(common, slow=35, slowmax=6))
    show("  sl30+上限6", **dict(common, slow=30, slowmax=6))
    show("  sl25+上限8", **dict(common, slow=25, slowmax=8))
    S.NOISE = keep_n
    print("-" * 128)

    # 摩擦敏感性: 真车偏航静摩擦未知
    print()
    print("★ 摩擦敏感性 (YAW_DEAD 未知, 5~15 都得成立):")
    keep = S.YAW_DEAD
    for yd in (5.0, 8.0, 12.0, 15.0):
        S.YAW_DEAD = yd
        print("   YAW_DEAD=%-5.0f" % yd, end="")
        for lab, kw in (("现状", dict(slow=50)),
                        ("sl30+上限6", dict(slow=30, slowmax=6)),
                        ("sl30+上限8", dict(slow=30, slowmax=8))):
            r = run_trace(**dict(common, **kw))
            c = r['rec']['BC'] + r['rec']['DA']
            print("   %s: 弯base%5.1f %5.2fs%s"
                  % (lab, med(c), r['t'], "" if r['ok'] else "(出线)"), end="")
        print()
    S.YAW_DEAD = keep


if __name__ == "__main__":
    main()
