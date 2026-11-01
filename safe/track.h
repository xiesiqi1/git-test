#ifndef __TRACK_H
#define __TRACK_H

#include "main.h"

/* 五路循迹传感器: 左到右 PA3 PA4 PA5 PA6 PA7 */

void Track_Init(void);                   /* 复位控制状态 */
void Track_SetEnabled(int16_t enable);   /* 1: 开始循迹; 0: 停止(并清状态) */
int16_t Track_GetEnabled(void);

uint8_t Track_GetMask(void);             /* 5bit, bit0(最左)~bit4(最右)=1 表示检测到黑线 */
int16_t Track_GetLineError(uint8_t mask);/* 行偏移误差: -4(最左)~0(居中)~+4(最右); 99=五路全丢 */

/* 启动前的自检: 确认探头能看到黑线。返回 0 表示"看不到", 此时不要启动 ——
 * 否则车会在完全不知道线在哪的情况下按基础速度冲出去。 */
uint8_t Track_StartOk(void);

/* 两个极性开关(实车"方向反了"时用, 不用改线、不用重新编译)。
 * ★ 切换后会立刻写进 Flash 保存, 断电/复位/重新烧录都保留。 */
uint8_t Track_GetActiveLevel(void);
void    Track_ToggleActiveLevel(void);
int8_t  Track_GetTurnSign(void);
void    Track_ToggleTurnSign(void);

/* 从 Flash 读回上次标定的极性。开机调用一次; 没存过就用 config.h 的默认值。 */
void Track_LoadSettings(void);

/* 控制节拍(周期 = config.h 的 CTRL_PERIOD_MS); 输入行偏移误差 */
void Track_Control(int16_t line_error);

/* ★ 2026-09-28 新增(只读, 给 OLED 显示用): 取回本拍真正下发的量。
 * 本工程没有串口, 想看清"差速到底多大、有没有被限幅夹住"只能靠它。
 * 任一指针可传 NULL。 */
void Track_GetOutput(int16_t *base, int16_t *diff, int16_t *left, int16_t *right);

/* 连续丢线拍数(0 = 正常)。 */
uint16_t Track_GetLostCount(void);

/* ★ 2026-09-29 新增: 停止线检测(压到停止线 -> 自动断油停车)。
 * 参数是本拍的五路位图(Track_GetMask 的返回值)。
 * 返回 1 = 这一拍已判定"压到停止线"并已把车停住 —— 调用方(CtrlLoop)
 * 应当就此结束本拍, 不要再算控制量, 否则会把刚停下的电机又推起来。
 *
 * ★ 判据 = 里程门槛 + 连续几路压黑 + 连续几拍, 三者在 config.h 里配。
 *   STOP_LINE_ENABLE = 0 时【依然会记录】压到宽线的事件(只是不停车) ——
 *   那是标定里程门槛要用的数据, 所以标定阶段就靠这个模式跑。
 *   ★ 判定顺序上它必须在 Track_Control() 之前调用。 */
uint8_t Track_StopDetect(uint8_t mask);

/* 车是【被什么自动停下来的】。OLED 第 1 行据此显示不同提示;
 * 下次短按启动会清掉(变回 0)。
 *     0 = 不是自动停的(没在跑 / 手动按键停 / 双击三击切极性)
 *     1 = 压到停止线停的      -> 屏上显示 "HIT LINE T xx.x"
 *     2 = 里程跑到"停车里程"停的 -> 屏上显示 "LAP DONE T xx.x"
 * ★ 2026-09-29 由 Track_GetStopLine() 改名而来: 加了里程停车之后,
 *   "停下来的原因"不再只有"压到线"一种, 名字要跟着准确。
 */
uint8_t Track_GetStopReason(void);

/* ★ 2026-09-29 新增: "压到宽线"事件的统计 —— 标定里程门槛的依据。
 *     evt   = 本次启动以来累计触发次数
 *     cnt   = 【本圈第一次】触发时的里程(编码器平均计数) ← 设门槛就看它
 *     width = 最近一次触发时"连续几路同时压黑"
 * 任一指针可传 NULL。
 * ★★ 注意它【只统计"够了门槛"的事件】: 如果 STOP_LINE_MIN_ACTIVE 设得比
 *   实际压出来的路数还大, 那就永远是 0, 看不出原因。要看"整圈最宽压了几路"
 *   请用下面的 Track_GetWidthMax()。
 * ★ 这些统计【停车后不会被清】—— 按键停止/双击三击切极性停止都保留,
 *   只在【按启动】时归零(每次启动 = 新的一圈)。这是为了标定时能读到数。 */
void Track_GetWideEvent(uint16_t *evt, int32_t *cnt, uint8_t *width);

/* ★★ 2026-09-29 新增: 本圈见过的【最大连续压黑路数】(最宽的一拍压了几路)。
 * ★ 和 Track_GetWideEvent 的区别: 它和门槛【无关】, 只要出现过就记。
 *   判"停止线能压几路、STOP_LINE_MIN_ACTIVE 该填几"看这个:
 *     跑完看到最大 3 路 -> 门槛改 3;  看到 5 路 -> 门槛改 5 最可靠。
 * ★ 只在"正在跑且已过启动保护"的拍上采样 —— 起步时车正停在启停线上,
 *   那一段不采, 免得把启停线自己的宽度混进来。
 * ★ 同样【停车不清】, 只在按启动时归零。 */
uint8_t Track_GetWidthMax(void);

/* ★★ 2026-09-29 新增: 本圈的"掉速档案" —— 回答"为什么跑着跑着明显减速"。
 *
 * 为什么必须有它: 屏上第 3 行的 B 只是【本拍的指令值】, 车一停那个数就没了。
 *   于是"到底哪一拍掉过速、掉到多少、是【收油】掉下去的还是【丢线】掉下去的"
 *   全都看不见 —— 实车"后面的弯明显减速"只能靠感觉描述, 定不了因。这里把
 *   一圈里最差的那几拍记下来, 停车后照样读得到。
 *
 *     bmin_ok   = 【看见线】时 base 的最小值   <- 只含"弯道收油"这一个因素
 *     bmin_all  = 含丢线拍在内 base 的最小值   <- 比 bmin_ok 低就说明丢过线
 *     lmax      = 一圈里最长的连续丢线拍数(0 = 全程没丢过线)
 *     bmin_dist = bmin_all 出现那一刻的里程(编码器平均计数) -> 判断是哪个弯
 *
 * 判读(配 config.h 的 SIMPLE_CORNER_SLOW_MAX, 直道 base = 32):
 *     BMIN 26/26 LMAX 0   一切正常: 弯里 base 26, 全程没丢线
 *     BMIN 19/19 LMAX 0   收油没封顶 / 封顶值太大 -> 掉到 CORNER_MIN 地板了
 *     BMIN 26/20 LMAX 30  指令值出现过 20 的坑 -> 【丢线】(LOST_SPEED) 才是元凶
 *     BMIN 32/32 LMAX 0   但车还是慢 -> 【指令值没问题】, 去查电池/打滑等机械
 *   ★ 实测首例(2026-09-29, 修正收油上限后): `BMIN 26/26 LMAX 0` + T 25.5 s
 *     —— 弯里 base 26 与仿真预测(25.21 s)几乎一样, 收油上限那一改确认生效。
 *
 * ★ 跳过软启动那段(RAMP_TICKS 拍): 那时输出被人为压低, 不算"掉速"。
 * ★★ 只有【持续 ≥ SIMPLE_LOST_SLOW_TICKS(8) 拍】的丢线才计入档案:
 *   LOST_SPEED 是"一丢线立刻降速", 3 拍(15ms)的瞬断也会把 base 压到 20,
 *   而 15ms 的指令在电机(时间常数 30~50ms)上什么都改变不了。不去掉它,
 *   屏上会出现 `26/20 LMAX 3` 这种"看着像故障、其实无意义"的读数
 *   (2026-09-29 第一次跑新版就踩到了)。★ LMAX 仍记【原始最大值】不去抖,
 *   所以"这一圈有没有瞬断"照样看得见 —— 只是它不再污染最小值。
 * 任一指针可传 NULL。 */
void Track_GetSlowStats(int16_t *bmin_ok, int16_t *bmin_all,
                        uint16_t *lmax, int32_t *bmin_dist);

#endif
