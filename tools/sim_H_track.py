# -*- coding: utf-8 -*-
"""
电赛 H 题赛道循迹仿真 —— 用来标定 safe/config.h 里的参数

赛道(2026 电赛 H 题 车载平衡滚球):
    2 x 1.5 m 直线 + 2 x 半径 0.5 m 半圆, 黑线宽 1.8 cm
    周长 = 2*1.5 + 2*pi*0.5 = 6.142 m
    要求一圈 <= 20 s  ->  平均速度 >= 0.31 m/s

这个脚本做什么:
    把赛道几何 + 五路探头 + track.c 的控制律都建成模型, 扫一遍参数,
    直接给出"一圈多少秒 / 最大横向偏移 / 丢线拍数", 用来判断
    config.h 里的值该往哪调。

★★ 2026-09-25 前后补了三块以前没有的东西, 因为旧模型把三个最关键的真车效应
   都忽略掉了, 导致它一直报"完美"而真车已经冲出黑线:
     TAU_MOTOR —— 电机/整车的速度一阶滞后(原来按瞬时算);
     NOISE     —— 传感器误判概率(原来一个bit都不会错);
     YAW_DEAD  —— ★★★ 偏航静摩擦(原来车"想转就转")。这一块是最后也是
                  最关键的一块: 没有它, 模型里的车永远用 3 PWM 的差速就能
                  完美跟住 50cm 圆弧, 于是完全看不到"打舵量不够"这一类故障。
   ★ 还要记住它【没有】建模的: 轮胎纵向打滑、地面反光延时、探头高度/倾角、
     左右电机不一致。所以结论只能定方向, 最终以实车秒表为准。

★ 第二条最重要的结论(2026-09-25 才发现, 推翻了之前所有的参数结论):
   弯道的【曲率】不在传感器信号里, 纯比例控制跟不住弯 ——
     五路数字探头在 1.8cm 线上, "正中"(err=0)的窄带只有约 1mm 宽, 所以车
     实际稳定停在 err=±1: 直道 ±1, 弯道也 ±1。实测(本脚本): 直道误差
     "同侧连续"最长 828 拍, 弯道 822 拍, 【完全分不开】。
     于是纯 P 只能靠"车偏出去 → 误差变大 → 催出打舵量"跟弯, 弯中平均差速
     只有 3.3 PWM。而车要先克服前支撑的偏航静摩擦才转得动, 3 PWM 全被吃掉
     —— 表现就是"车头转一点、车身往外滑, 整条弯贴着探头边缘跑"。
   药方有两条, 仿真都验过(偏航门槛按 12 PWM 估, base 32):
     Kp 14 + 曲率积分(Ki=150/上限16/泄漏50) -> 弯中|err| 3.52 → 0.85~1.13,
                                              直道横偏 19mm → 1~8mm, 圈时也更快
     或者干脆把 Kp 提到 22~30(不用积分)  -> 也能压到 |err| ≤ 1
   固件里两条都上了(Kp=14 + 积分), 因为积分能自动适应未知的静摩擦大小。

★ 第一条最重要的结论:
   探头【必须】装在驱动轮轴前方, 而且(补上偏航静摩擦之后)悬崖在 2~3 cm ——
        轮轴后方 1cm -> 出线(丢线 46)      正好在轮轴上 -> 出线(丢线 189)
        前移 0.5 cm  -> 出线(丢线 244)     前移 1.0 cm  -> 最大横偏 49 mm
        前移 1.5 cm  -> 29 mm              前移 2.0 cm  -> 16 mm
        前移 3.0 cm  -> 8.2 mm  (可用)     前移 4.0 cm  -> 7.3 mm  (可用)
   探头在轮轴附近时, "进弯反应慢、等跑出弯道才转弯、然后找不到线"是【必然】
   结果, 先量这个距离。⚠ 要留到 3 cm 以上: "1 cm 就够"是没有偏航静摩擦的
   旧模型给的答案, 真车会比它更苛刻。

怎么用:
    python tools/sim_H_track.py
    改下面 【需要改的地方】 里的车辆参数和 CASES, 再跑一次即可。

改车辆参数(这几个必须按你的实车改, 否则数值不可比):
    D_WHEEL  左右轮中心距(量一下) / K_V 每 1 PWM 对应多少 m/s
    SENS_LONG 探头板相对【驱动轮轴】前移多少(★ 这个最要命)
    PITCH    相邻探头间距 / TAU_MOTOR 电机滞后时间常数
    YAW_DEAD 偏航静摩擦(★ 未知量, 用实车现象反推: 提高它就等于"车更难转")
"""
import math
import random

# 传感器误判概率(真车必然有: 反光、线宽不均、地面花纹)。
# 用来验证"虚拟前视"这类放大误差变化率的算法会不会把噪声也一起放大。
NOISE = 0.0
# 噪声实验要固定随机种子: 否则同一组参数每次跑出来的横偏都不一样, 文档里的
# 数字就复现不了。同一个种子让【所有参数配置看到同一串误判】, 比较才公平。
NOISE_SEED = 20260925

# ============================================================
#                 【需要改的地方 1】车辆参数
# ============================================================
D_WHEEL   = 0.27      # 左右轮中心距 m
                      # ★★ 2026-09-29 用户实测确认 = 0.27 m（此前模型里是 0.15）。
                      #   这个数直接决定"转同样的半径需要多大的轮速差":
                      #       转弯半径 R = v x D_WHEEL / 两轮速差
                      #   D_WHEEL 由 0.15 改 0.27 后, 同样的打舵量只能转出
                      #   0.56 倍的角速度 —— 仿真的"弯道难度"因此变高, 与实车
                      #   更接近。★ 改这个数会让下面所有表都变, 别当成"调参"。
                      #   旁证: 实车左右轮编码器计数差 31.5%, 按 0.27 的几何
                      #   正好解释得通（按 0.15 只解释得了 ~15%, 曾误判为丢脉冲）。
                      #   实测方法: 把车摆正, 左轮贴着直尺走一整圈, 量两轮印间距。
K_V       = 0.0115    # m/s per PWM  —— 实测方法: 直线跑 1.5 m 计时 t,
                      #   速度 = 1.5/t; K_V = 速度 / (基础速度 - PWM_DEAD)
PWM_DEAD  = 8         # 电机启动死区(PWM), 低于它轮子不转
SENS_LONG = 0.03      # 探头中心相对轮轴中心前移多少 m
PITCH     = 0.01      # 相邻探头间距 m
LINE_HW   = 0.009     # 黑线半宽 m (线宽 1.8 cm -> 半宽 9 mm)
DT        = 0.005     # 控制周期 s, 对应 config.h 的 CTRL_PERIOD_MS

# 赛道几何(H 题): 2 段直线 + 2 个半圆。练习用的"微型跑道"请按实况改小。
STRAIGHT  = 1.5       # 直线段长 m
R_CURVE   = 0.5       # 半圆半径 m

# ★ 2026-09-25 补上: 电机/整车的速度一阶滞后时间常数 (s)。
#   原来这里把 PWM->转速 当【瞬时】的, 后果是仿真永远看不到"进弯反应慢"
#   —— 而这恰恰是真车最要命的失效模式(实车反馈"等跑出弯道了才转弯, 直接
#   找不到线", 仿真却一直报"横偏 1.0 mm, 完美")。
#   直流减速电机 + 整车惯量, 典型 30~60 ms。实车若在弯道明显发飘/滞后,
#   把这个值往大调(0.06~0.08)再看结论。
TAU_MOTOR = 0.045

# ★★★ 2026-09-25 补上【偏航静摩擦】(PWM)。
#   这是整个仿真里最后一块、也是最关键的一块缺失物理。
#   为什么必须有它: 五路数字探头在 1.8cm 线上, err=0 的"正中"窄带只有约
#   1mm 宽, 所以车实际上稳定停在 err=±1 上 —— 直道是 ±1, 弯道也是 ±1,
#   曲率信息【根本不在误差信号里】(实测: 直道"同侧连续"最长 828 拍, 弯道
#   822 拍, 完全分不开)。纯 P 控制只能靠"滞后→误差变大→催出打舵量"来跟弯,
#   所以车在弯里只给出 3~5 PWM 的差速。真车还要先克服前支撑被横向拖动的
#   静摩擦才转得动 —— 那个门槛仿真里原本没有, 于是永远报"弯道横偏 1mm"。
#   物理: 偏航力矩 = (F外 − F内)·D/2, 要超过静摩擦扭矩 τc 车才开始转。
#   等效成"差速死区": 实际差速 = sign(差速)·max(0, |差速| − YAW_DEAD)。
#   取值参考: 3 PWM 以下车仍然跟得住(见 _diag5), 16 PWM 时车到探头边缘
#   才开始转 —— 所以真车的门槛大致落在 5~15 之间, 请用实车现象反推。
YAW_DEAD = 12.0

# ============================================================
#                 【需要改的地方 2】要扫描的参数组
#                 每项都是 track.c Stage 1 里的一整套参数
# ============================================================
CASES = []
# ★ 用例表已对齐当前固件参数(safe/config.h):
#     Kp=14  跳变限幅=3  死区=0  差速上限=32  减速=50%/下限 60%
#     虚拟前视=30 拍(低通 20 拍)  丢线回找速度=20
#     曲率积分: Ki=150 / 上限16 / 泄漏50
#     ★ 差速变化率限幅 dslew=3 PWM/拍(治"每次调速差速跳得太猛")
#   ★ 2026-09-29: 前视 20 -> 30。实车"两个弯打舵量不统一"(弯1 内0外60 /
#     弯2 内15外40)。反推得弯1 的差速到了 30, 那只有"误差打到最外侧(±4)"
#     才凑得出 —— 是车先偏出去、控制器才拼命补, 不是舵太多。仿真只改这一项:
#     最大横偏 8.5 -> 2.4 mm, 两弯误差都稳在 1.0, 饱和率 4% -> 0%。
#     (⚠ 反面对照: 把差速上限砍到 13 也能让两弯"看起来统一", 但弯里误差
#      1.0 -> 4.0、横偏 40.8 mm 超探头半跨 20 mm, 摩擦大时直接出线。
#      验证脚本见 tools/sim_turn_unify.py。)
#   base 那一列是主扫描(唯一能改一圈时间的旋钮), kp 顺带扫两个看敏感度。
#   kd 已不参与控制(被虚拟前视取代), 留字段只为兼容旧调用。
#   ⚠ 模型里含 YAW_DEAD(偏航静摩擦), 所以圈时比历史记录明显慢 —— 那是真车
#     本来就要付的代价, 不是这次改坏了。
for base in (24, 28, 32, 36, 42, 46):
    for kp in (14, 20):
        CASES.append(dict(base=base, kp=kp, kd=0, dl=3, db=0, dmax=32,
                          slow=50, slowmax=6, minpct=60,
                          tki=150, tlim=16, ttau=50,
                          boost=10, bcnt=3,
                          leadt=30, ltau=20, dslew=3,
                          search=12, lostspd=20,
                          preserve=True, tau=TAU_MOTOR))

LAP_TARGET = 20.0     # 题目要求的一圈上限 s
OK_MAXOFF  = 0.015    # 判定"跟得好"的最大横向偏移 m (探头半跨是 20 mm)

# ============================================================
#                        以下一般不用改
# ============================================================
STEP = 0.002          # 赛道离散化步长


def cdiv(a, b):       # C 的整数除法: 向零取整(和 track.c 一致)
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def limit(v, lim):
    return max(-lim, min(lim, v))


def wspeed(pwm):      # PWM -> 轮速 m/s
    pwm = max(0.0, min(100.0, pwm))
    return 0.0 if pwm <= PWM_DEAD else K_V * (pwm - PWM_DEAD)


def build_path():
    """2 x L 直线 + 2 x R 半圆, 顺时针闭合。
    半径 R_CURVE 可改 —— 练习道往往是"微型跑道", 半径比 H 题的 0.5 m 小得多,
    需要的打舵量和所需的探头前视距离都会跟着变大。"""
    L, R = STRAIGHT, R_CURVE
    pts = []
    x = 0.0
    while x < L:                                          # AB
        pts.append((x, 0.0)); x += STEP
    a = math.pi / 2
    while a > -math.pi / 2:                               # BC 半圆
        pts.append((L + R * math.cos(a), -R + R * math.sin(a)))
        a -= STEP / R
    x = L
    while x > 0:                                          # CD
        pts.append((x, -2 * R)); x -= STEP
    a = -math.pi / 2
    while a > -3 * math.pi / 2:                           # DA 半圆
        pts.append((R * math.cos(a), -R + R * math.sin(a)))
        a -= STEP / R
    return pts


def set_track(r=None, straight=None):
    """换赛道几何(半径/直线长度), 重建路径与切向量, 供扫描用"""
    global R_CURVE, STRAIGHT, PATH, N, LAP, TAN
    if r is not None:        R_CURVE = r
    if straight is not None: STRAIGHT = straight
    PATH = build_path()
    N = len(PATH)
    LAP = 2 * STRAIGHT + 2 * math.pi * R_CURVE
    TAN = []
    for _i in range(N):
        _j = (_i + 1) % N
        _dx, _dy = PATH[_j][0] - PATH[_i][0], PATH[_j][1] - PATH[_i][1]
        _n = math.hypot(_dx, _dy) or 1.0
        TAN.append((_dx / _n, _dy / _n))


PATH = build_path()
N = len(PATH)
LAP = 2 * STRAIGHT + 2 * math.pi * R_CURVE
TAN = []
for _i in range(N):
    _j = (_i + 1) % N
    _dx, _dy = PATH[_j][0] - PATH[_i][0], PATH[_j][1] - PATH[_i][1]
    _n = math.hypot(_dx, _dy) or 1.0
    TAN.append((_dx / _n, _dy / _n))


def lateral(px, py, hint):
    """返回 (最近路径点索引, 有符号横向偏移[左正右负]); 局部搜索, 更快"""
    best, bd = hint, 1e9
    for k in range(-20, 25):
        i = (hint + k) % N
        x, y = PATH[i]
        dd = (px - x) ** 2 + (py - y) ** 2
        if dd < bd:
            bd, best = dd, i
    tx, ty = TAN[best]
    x, y = PATH[best]
    ox, oy = px - x, py - y
    return best, (-ty * ox + tx * oy), math.hypot(ox, oy)


def sensor_mask(cx, cy, th, hint):
    """五路探头的位图。车体坐标: x 前, y 左; 第 0 路是最左那路"""
    mask, h = 0, hint
    for i in range(5):
        lx, ly = SENS_LONG, (2 - i) * PITCH
        gx = cx + lx * math.cos(th) - ly * math.sin(th)
        gy = cy + lx * math.sin(th) + ly * math.cos(th)
        h, off, _ = lateral(gx, gy, h)
        bit = 1 if abs(off) <= LINE_HW else 0
        if NOISE and random.random() < NOISE:
            bit ^= 1                      # 这一路误判
        if bit:
            mask |= 1 << i
    return mask, h


def line_error(mask):
    """与 track.c 的 Track_GetLineError 一致(夹线定位), 单位=半格"""
    if mask == 0:      return 99
    if mask == 0x1F:   return 0
    pos = [-2, -1, 0, 1, 2]
    lo = hi = -1
    for i in range(5):
        if mask & (1 << i):
            if lo < 0: lo = i
            hi = i
    return pos[lo] + pos[hi]


def make_ctrl(base, kp, kd, dl, db, dmax, slow, minpct,
              tki=0, tlim=0, ttau=50, boost=0, bcnt=3, preserve=False,
              leadt=20, ltau=20, search=12, lostspd=20, tau=0.0, dslew=0,
              predmax=0, slowmax=0):
    """tki/tlim/ttau —— 曲率积分: 速度(千分之一 PWM/拍/半格) / 上限(PWM) /
                       误差回到线上后的泄漏时间常数(拍)
       dslew       —— 差速【变化率】上限(PWM/拍)。0=不限制(原行为)。
                      用来治"每次调速差速跳得太猛、车一顿一顿往外甩"。
       predmax     —— 【预测误差】的限幅(单位: 半格)。0=不限制(原行为)。
                      为什么需要: 预测误差 = 误差 + 前视拍数 x 变化速度, 而真实
                      误差只有 ±4(半格), 变化速度限幅 ±3/拍, 20 拍外推后预测误差
                      理论能到 ±60 —— 它算的是"再等 100ms 车会偏 30 个探头间距",
                      这在物理上早就出线了。这个假值会把差速一把顶到上限, 于是
                      【进第一个弯(误差跳得快)和第二个弯(误差平稳)打出的舵完全
                      不同】—— 实车"弯1 内0外60 / 弯2 内15外40"就是这么来的。
       slowmax     —— 【弯道收油】的绝对上限(PWM)。0 = 不限制(原行为)。
                      为什么需要: 收油量 = |diff| x slow%, 而弯道里 diff 里
                      含有曲率积分(它恰恰在弯里顶到上限、且【不泄漏】), 于是
                      弯道中后段收油被叠加到最大值 —— base 从 32 掉到 19~22,
                      实车感觉就是"跑到后面的弯明显减速"。加上限之后不管
                      diff 飙到多少, 弯里最多只降 slowmax 个 PWM。"""
    return dict(base=base, kp=kp, kd=kd, dl=dl, db=db, dmax=dmax, slow=slow,
                minpct=minpct, tki=tki, tlim=tlim, ttau=ttau,
                boost=boost, bcnt=bcnt,
                preserve=preserve, last=0, trim=0, lost_cnt=0, lost_dir=0,
                last_diff=0, last_pred=0,
                search=search, lostspd=lostspd, tau=tau,
                vl=0.0, vr=0.0, leadt=leadt, ltau=ltau, dlp=0,
                dslew=dslew, last_out=0, predmax=predmax, slowmax=slowmax,
                last_base=0)


def ctrl_step(C, err):
    """与 track.c 的 Stage 1 Track_Control 逐行对应"""
    raw = err
    if err == 99:                       # 丢线
        C['lost_cnt'] += 1
        err = C['lost_dir'] * 4 if C['lost_dir'] else 0
    else:
        C['lost_cnt'] = 0
        if err != 0: C['lost_dir'] = 1 if err > 0 else -1
        if -C['db'] <= err <= C['db']: err = 0

    # 误差变化速度(限幅, 防单拍野值)
    d = limit(err - C['last'], C['dl']); C['last'] = err

    # 虚拟前视 -> 预测误差 -> 差速
    # (kd>0 时走【旧算法】只做对比用: 未低通的原始微分, 就是被淘汰的那个 KD)
    if C['kd'] > 0:
        pred = err
        diff = cdiv(C['kp'] * err + C['kd'] * d, 2)
    elif C['lost_cnt'] == 0:
        C['dlp'] += cdiv((d << 8) - C['dlp'], C['ltau'])
        pred = err + cdiv(C['leadt'] * C['dlp'], 256)
        diff = cdiv(C['kp'] * pred, 2)
    else:
        C['dlp'] = 0
        pred = err
        diff = cdiv(C['kp'] * pred, 2)

    # ★ 预测误差限幅(默认 0 = 关闭; 与 config.h 的 SIMPLE_PRED_MAX 对应)。
    #   见 make_ctrl 里 predmax 的注释: 前视拍数 x 变化速度 可以算出 ±60 半格
    #   这种物理上不可能的值(真实误差只有 ±4), 进弯瞬间它会一把把差速顶满。
    if C['predmax'] > 0:
        pred = limit(pred, C['predmax'])
        if C['kd'] == 0:                 # 旧 KD 算法只用于对比, 不参与重算
            diff = cdiv(C['kp'] * pred, 2)

    # ---- 曲率积分(trim / 弯道稳态分量), Q8 定点累加 ----
    # 它补的是"偏航静摩擦"那一块: 纯 P 在持续曲率上必须留一个稳态误差才凑得
    # 出打舵量, 积分把这个偏置自己学出来 —— 于是误差能回到 0, 而差速仍维持着。
    # 直道上误差在 0/±1 之间来回, 加上泄漏, 积分不会无限涨。
    # tki 单位 = 千分之一 PWM / (拍 × 半格)
    if C['tki'] > 0:
        if C['lost_cnt'] == 0:
            C['trim'] += cdiv((err * C['tki']) << 8, 1000)
            C['trim'] = limit(C['trim'], C['tlim'] << 8)
            if abs(err) <= 1:                # 已经在线上 -> 慢慢吐掉
                C['trim'] -= cdiv(C['trim'], C['ttau'])
        else:
            C['trim'] = 0
        diff += C['trim'] >> 8

    diff = limit(diff, C['dmax'])
    C['last_pred'] = pred

    # 基础速度
    # ★ 收油 = |diff| x slow%(有下限 floor)。slowmax>0 时再加一道【绝对上限】:
    #   弯道里 diff 含曲率积分(弯里顶到上限、不泄漏), 收油会被叠到最大,
    #   base 32 -> 19~22, 实车就是"跑到后面的弯明显减速"。
    base = C['base']
    floor = cdiv(C['base'] * C['minpct'], 100)
    if C['lost_cnt'] == 0:
        oil = cdiv(abs(diff) * C['slow'], 100)
        if C['slowmax'] > 0 and oil > C['slowmax']:
            oil = C['slowmax']
        base -= oil
        if base < floor: base = floor
    elif C['lost_dir'] == 0:
        base = C['search']              # 不知道线在哪 -> 只许爬行
    else:
        base = C['lostspd']             # 知道方向 -> 降速用力拧回来
    C['last_base'] = base               # 供诊断脚本抓"弯道里到底跑了多慢"

    # 严重丢线额外加力
    if C['lost_cnt'] >= C['bcnt'] and C['lost_dir']:
        diff = limit(diff + C['boost'] * C['lost_dir'], C['dmax'] + C['boost'])

    # ★ 差速【变化率】限幅(可选): 不让打舵量一拍就跳到位。
    #    "每次调速差速过猛、车一顿一顿往外甩"就靠它治 —— 力度上限不动,
    #    只是让力度"爬"上去而不是"跳"上去。
    if C['dslew'] > 0:
        dd = diff - C['last_out']
        if dd > C['dslew']:    diff = C['last_out'] + C['dslew']
        elif dd < -C['dslew']: diff = C['last_out'] - C['dslew']
    C['last_out'] = diff

    L, R = base + diff, base - diff

    # 限幅: 保住差速(越界时两侧一起平移)
    if C['preserve']:
        if L > 100: R -= (L - 100); L = 100
        if R > 100: L -= (R - 100); R = 100
        if L < 0:   R += (0 - L);   L = 0
        if R < 0:   L += (0 - R);   R = 0
    L = max(0, min(100, L)); R = max(0, min(100, R))
    C['last_diff'] = diff
    return L, R


def run(name, trace=False, **kw):
    C = make_ctrl(**kw)
    if NOISE > 0.0:
        random.seed(NOISE_SEED)     # 噪声实验可复现(见 NOISE_SEED 注释)
    cx, cy, th = 0.0, 0.0, 0.0          # 从 A 点出发, 朝 +x
    hint, t, dist, lost, maxoff = 0, 0.0, 0.0, 0, 0.0
    step_n = 0
    # ★ 差速的"剧烈程度"统计(2026-09-25 加): 用来量化"每次调速差速过大/过猛"。
    #   dsum/sum2 越小 = 打舵越平缓; flips = 差速左右翻面的次数(画龙); sat 越大
    #   = 打舵量一直顶在上限上(DIFF_MAX 夹住)。
    dsum = 0.0; dstep_max = 0; flips = 0; sat = 0; nz = 0; sum2 = 0.0
    prev_diff = 0; prev_sign = 0
    # ★ 电机速度的一阶滞后: 指令 PWM 换了之后, 轮速按 tau 慢慢跟上。
    #   tau=0 就是旧模型的"瞬时响应"。这一步是 2026-09-25 加的, 没有它
    #   仿真会永远报"弯道横偏 1 mm", 而真车已经冲出黑线了。
    a = 1.0 if C['tau'] <= 0 else min(1.0, DT / C['tau'])
    while dist < LAP and t < 60.0:
        mask, hint = sensor_mask(cx, cy, th, hint)
        e = line_error(mask)
        if mask == 0: lost += 1
        L, R = ctrl_step(C, e)
        C['vl'] += (wspeed(L) - C['vl']) * a
        C['vr'] += (wspeed(R) - C['vr']) * a
        vl, vr = C['vl'], C['vr']
        if YAW_DEAD > 0.0:
            # 偏航静摩擦: 差速要先"扣掉"这个门槛, 车才开始真的转
            dv = vr - vl
            cut = 2.0 * YAW_DEAD * K_V
            if dv > 0.0:    dv = max(0.0, dv - cut)
            elif dv < 0.0:  dv = min(0.0, dv + cut)
            mid = (vl + vr) * 0.5
            vl, vr = mid - dv * 0.5, mid + dv * 0.5
            if vl < 0.0: vl = 0.0        # 本车没有倒转能力: 内侧轮最多被拖到 0
            if vr < 0.0: vr = 0.0
        v = (vl + vr) / 2.0
        w = (vr - vl) / D_WHEEL
        cx += v * math.cos(th) * DT
        cy += v * math.sin(th) * DT
        th += w * DT
        dist += v * DT
        _, off, _ = lateral(cx, cy, hint)
        maxoff = max(maxoff, abs(off))
        # --- 差速统计 ---
        di = C['last_diff']
        dsum += abs(di - prev_diff)
        if abs(di - prev_diff) > dstep_max: dstep_max = abs(di - prev_diff)
        prev_diff = di
        if di != 0:
            sg = 1 if di > 0 else -1
            if prev_sign != 0 and sg != prev_sign: flips += 1
            prev_sign = sg
        if abs(di) >= C['dmax']: sat += 1
        nz += 1
        sum2 += off * off
        step_n += 1
        if trace and 4.2 < t < 10.0 and step_n % 10 == 0:
            print("      t=%5.2f s=%5.2f 横偏=%6.1fmm mask=%05s err=%3s "
                  "预测=%3d diff=%3d L=%3d R=%3d%s"
                  % (t, dist, off * 1000, format(mask, 'b'), e,
                     C['last_pred'], C['last_diff'], L, R,
                     "  <<丢线" if mask == 0 else ""))
        t += DT
        if abs(off) > 0.06:             # 偏离超过 3 格 = 完全脱离, 判失败
            return dict(ok=False, t=t, lost=lost, maxoff=maxoff, n=nz,
                        dsum=dsum, dstep=dstep_max, flips=flips,
                        satpct=100.0 * sat / max(1, nz),
                        rms=math.sqrt(sum2 / max(1, nz)),
                        why='偏离 %.1f cm' % (off * 100))
    return dict(ok=dist >= LAP, t=t, lost=lost, maxoff=maxoff, n=nz,
                dsum=dsum, dstep=dstep_max, flips=flips,
                satpct=100.0 * sat / max(1, nz),
                rms=math.sqrt(sum2 / max(1, nz)), why='')


def main():
    print("赛道: 2 x 1.5m 直线 + 2 x R0.5m 半圆, 周长 %.3f m" % LAP)
    print("题目要求: 一圈 <= %.0f s  ->  平均速度 >= %.3f m/s" % (LAP_TARGET, LAP / LAP_TARGET))
    print("标定基准: 直线段 1.5 m 要在 %.1f s 内跑完" % (1.5 / (LAP / LAP_TARGET)))
    print()
    print("%-6s %-5s %-5s %-6s %-8s %-10s %-7s %s"
          % ("基础速度", "Kp", "Kd", "积分Ki", "一圈(s)", "最大横偏", "丢线", "判定"))
    print("-" * 74)
    best = []
    for c in CASES:
        r = run("", **c)
        ok = r['ok'] and r['t'] <= LAP_TARGET and r['maxoff'] <= OK_MAXOFF and r['lost'] <= 5
        if not r['ok']:
            verdict = "出线"
        elif r['t'] > LAP_TARGET:
            verdict = "太慢"
        elif r['maxoff'] > OK_MAXOFF or r['lost'] > 5:
            verdict = "丢线/摆"
        else:
            verdict = "可用"
        print("%-9d %-5d %-5d %-8d %-9.2f %-11s %-8d %s"
              % (c['base'], c['kp'], c['kd'], c['tki'], r['t'],
                 "%.1f mm" % (r['maxoff'] * 1000), r['lost'], verdict))
        if ok:
            best.append((r['t'], c))
    print()
    if best:
        best.sort(key=lambda z: z[0])
        t, c = best[0]
        print("★ 该扫描下最优: base=%d Kp=%d Kd=%d Ki=%d -> 一圈 %.2f s"
              % (c['base'], c['kp'], c['kd'], c['tki'], t))
        print("  注意: 这是理想模型的结果, 真车会因为电机过渡/传感器延时更差,")
        print("        所以选参数时留 15% 以上余量, 最终以实车秒表为准。")
    else:
        print("★ 没有同时满足 20 s 与横向偏移的配置, 需要调整车辆参数或控制律")

    # ------------------------------------------------------------------
    # 探头前视距离体检 —— 这是最重要的一项, 比任何参数都关键。
    # ------------------------------------------------------------------
    print()
    print("★ 探头前视距离体检(先量这个, 再调任何参数):")
    print("   含义: 探头板中心相对【驱动轮轴】往前多少。装反/装近了任何参数都救不了。")
    print("   (下面用的是【当前固件参数】, 所以这一列的数字会随调参一起变;")
    print("    要看的是'哪一档开始出线'这个分界, 而不是具体秒数。)")
    print("   %-14s %-9s %-11s %-7s %s"
          % ("探头前移", "一圈(s)", "最大横偏", "丢线", "结果"))
    keep = SENS_LONG
    ref = dict(base=32, kp=14, kd=0, dl=3, db=0, dmax=32, slow=50, slowmax=6,
               minpct=60,
               tki=150, tlim=16, ttau=50, boost=10, bcnt=3, leadt=30, ltau=20,
               dslew=3, search=12, lostspd=20, preserve=True, tau=TAU_MOTOR)
    for sl in (-0.01, 0.0, 0.005, 0.010, 0.015, 0.020, 0.030, 0.040):
        globals()['SENS_LONG'] = sl
        r = run("", **ref)
        if not r['ok']:
            v = "★出线 " + r['why']
        elif r['maxoff'] <= OK_MAXOFF and r['lost'] <= 5:
            v = "可用"
        else:
            v = "★横偏超 15mm(会丢)"
        print("   %-14s %-9.2f %-11s %-7d %s"
              % ("%+.1f cm" % (sl * 100), r['t'],
                 "%.1f mm" % (r['maxoff'] * 1000), r['lost'], v))
    globals()['SENS_LONG'] = keep
    print("   -> ★ 补上偏航静摩擦(YAW_DEAD)之后, 这道悬崖从 1 cm 挪到了 2~3 cm:")
    print("      2 cm 以下【必然出线】, 3 cm 起才算稳, 4 cm 最舒服。")
    print("      小于 3 cm 请先把探头板往前挪, 别调参数 —— 软件补不回机械前视。")

    # ------------------------------------------------------------------
    # 对照实验 A: 临界前视(0.5cm)下, 加大 KD(旧算法) vs 虚拟前视
    # ------------------------------------------------------------------
    print()
    print("★ 对照实验 A: 探头贴到临界(前移 0.5 cm)时, 加 KD / 加前视 谁能救回来")
    print("   %-26s %-9s %-11s %-7s %s" % ("手段", "一圈(s)", "最大横偏", "丢线", "结果"))
    globals()['SENS_LONG'] = 0.005
    expA_base = dict(base=32, db=0, dmax=32, slow=50, slowmax=6, minpct=60,
                     tki=150, tlim=16, ttau=50, boost=10, bcnt=3,
                     dslew=3, search=12, lostspd=20, preserve=True, tau=TAU_MOTOR)
    expA = [
        ("旧 KD 项  KD=8  (限幅3)", dict(kp=14, kd=8,  dl=3, leadt=0,  ltau=20)),
        ("旧 KD 项  KD=16 (限幅4)", dict(kp=14, kd=16, dl=4, leadt=0,  ltau=20)),
        ("旧 KD 项  KD=24 (限幅5)", dict(kp=14, kd=24, dl=5, leadt=0,  ltau=20)),
        ("虚拟前视 前视=10 拍",     dict(kp=14, kd=0,  dl=3, leadt=10, ltau=20)),
        ("虚拟前视 前视=20 拍",     dict(kp=14, kd=0,  dl=3, leadt=20, ltau=20)),
        ("虚拟前视 前视=30 拍",     dict(kp=14, kd=0,  dl=3, leadt=30, ltau=20)),
        ("虚拟前视 前视=40 拍",     dict(kp=14, kd=0,  dl=3, leadt=40, ltau=20)),
    ]
    for label, kw in expA:
        r = run("", **dict(expA_base, **kw))
        if not r['ok']:
            v = "★出线 " + r['why']
        elif r['maxoff'] <= OK_MAXOFF and r['lost'] <= 5:
            v = "可用"
        else:
            v = "★横偏超 15mm(会丢)"
        print("   %-26s %-9.2f %-11s %-7d %s"
              % (label, r['t'], "%.1f mm" % (r['maxoff'] * 1000), r['lost'], v))
    globals()['SENS_LONG'] = keep
    print("   -> 加大 KD 用的是【未低通的原始微分】, 作用只持续 5 ms, 电机来不及响应,")
    print("      三次全都还是出线/超偏。虚拟前视把微分低通再外推, 才勉强压住。")
    print("      但 0.5 cm 这种极端前视下, 连前视也救不干净 —— 机械问题回机械解决。")

    # ------------------------------------------------------------------
    # 对照实验 B: 正常前视(3cm)下, 前视拍数会不会把直道搞毛躁(含噪声敏感性)
    # ------------------------------------------------------------------
    print()
    print("★ 对照实验 B: 正常前视(3 cm)下, 前视拍数对圈时/横偏的影响(再看噪声)")
    globals()['SENS_LONG'] = 0.030
    expB_base = dict(base=32, kp=14, kd=0, dl=3, db=0, dmax=32, slow=50,
                     slowmax=6,
                     minpct=60, tki=150, tlim=16, ttau=50, boost=10, bcnt=3,
                     dslew=3, search=12, lostspd=20, preserve=True, tau=TAU_MOTOR)
    keep_noise = NOISE
    for noise in (0.0, 0.03, 0.15):
        globals()['NOISE'] = noise
        row = []
        for lt in (0, 10, 20, 30, 40, 60):
            r = run("", **dict(expB_base, leadt=lt, ltau=20))
            if not r['ok']:
                row.append("前视%2d→【出线】" % lt)
            else:
                row.append("前视%2d→%5.1fmm" % (lt, r['maxoff'] * 1000))
        print("   误判 %4.0f%%: %s" % (noise * 100, "  ".join(row)))
    globals()['NOISE'] = keep_noise
    globals()['SENS_LONG'] = keep
    print("   -> 无噪声时前视越长越准且不花时间; 但有噪声时前视会【放大噪声】,")
    print("      误判 15% 时整条模型都失稳。真车直道细碎抖动就调小前视(20→12)。")


if __name__ == "__main__":
    main()
