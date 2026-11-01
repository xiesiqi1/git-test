/* ============================================================================
 *  oled.c —— 1.3 寸 SH1106 (128x64) OLED 显示模块
 *  2026-09-28 新增。
 *
 *  本工程【没有串口】(PA9/PA2/PA3 都被电机/探头占了), 所以这块屏就是唯一的
 *  "调试窗口"。它把循迹环里最关键的几个量直接摊在屏幕上:
 *
 *      RUN  PL1 TS-1      <- 是否在跑 / 传感器极性 / 转向极性
 *      E  -2  LOST 0      <- 夹线误差(-4~+4, 99=丢线) / 连续丢线拍数
 *      B 32 DIFF-12       <- 本拍基础速度(已含弯道收油) / 本拍真正下发的差速
 *      L 44   R 20        <- 本拍真正下发的左/右轮 PWM
 *      [ ][X][X][ ][ ]    <- 五路探头(最左 -> 最右), 实心=压到黑线
 *      ENC  1234  1205    <- 左右轮累计编码器计数(H 题停车/里程要用)
 *
 *  ★★ 停车之后第 2、4 行会【换成另一套内容】(2026-09-29 新增) —— 车一停那三
 *     个"当前值"就没有信息量了, 让位给"刚跑完这一圈的档案":
 *      STOP PL0 TS-1 T 27.1  两个极性 + 刚跑完的成绩(第 1 行)
 *        ↑ 若是【自动停】的, 第 1 行换成 HIT LINE T… (压到停止线停的)
 *          或 LAP DONE T… (里程到一圈停的) —— 一眼看出是哪个入口停的。
 *      BMIN 26/19 LMAX 8     掉速档案: base 最小值(看见线时/含丢线) / 最长丢线拍数
 *      EVT  5 MAXW 5         压到宽线的次数 / 整圈最宽压了几路
 *      SLOW@ 543K WIDE@ 867K 最慢那一拍的位置 / 第一次压宽线的位置(单位 1000 计数)
 *      ENC  -1605  1224 M0   左右累计计数(单位 1000 计数) + A/B 点进度
 *
 *  ★ 调"差速过大"这类问题时, 就盯着 DIFF 和 L/R 两行看:
 *      DIFF 一拍之内跳多少、L 与 R 差多少, 一眼就清楚。
 *      (这以前只能靠上车跑圈看现象反推, 也正是 2026-09-25 连错两次的原因。)
 *  ★★ 调"为什么明显减速"这类问题时, 看停车后的第 2 行 —— 它把"弯道收油收狠了"
 *     和"丢线把速度踩下去了"分开, 还能看出"指令值根本没掉, 是车自己没劲
 *     (电池/打滑)"。完整判读表见 track.h 的 Track_GetSlowStats()。
 *
 *  ★ 刷屏开销: 128x64 全帧 = 1024 字节, 4.5 MHz 下阻塞约 1.8 ms。
 *    所以刷新被限速到 OLED_REFRESH_MS(默认 100 ms = 10 Hz), 占 CPU 不到 2%。
 *    控制环跑在 SysTick 中断里, 优先级高于主循环, 不受这个阻塞影响。
 * ========================================================================== */

#include "oled.h"
#include "u8g2.h"
#include "u8g2_hal.h"
#include "track.h"
#include "encoder.h"
#include "stopwatch.h"
#include "mileage.h"
#include "config.h"

/* 单独提取出来的那一个字库(见 csrc/u8g2_font_6x10_tr.c)。u8g2.h 不带字体声明,
 * 所以这里自己 extern 一下。 */
extern const uint8_t u8g2_font_6x10_tr[];

static u8g2_t   s_u8g2;
static uint32_t s_next_ms = 0;

#ifndef OLED_REFRESH_MS
#define OLED_REFRESH_MS   100u
#endif
#ifndef OLED_Y0
#define OLED_Y0           4
#endif
#ifndef OLED_ROW_H
#define OLED_ROW_H        9
#endif

/* 第 n 行(0 起)的 y 坐标。
 * ★ 只在这里算一次坐标: 全部行都用它, 上下留边和行距就只在 config.h
 *   的两个宏里控制。现场发现"最上面/最下面的字被切"时, 改 OLED_Y0
 *   就能整体上下移, 不用翻代码逐个改数字。
 *   (背景: 1.3 寸模块的塑料窗口会盖住最上/最下各约 3~4 行, 所以
 *    6 行不能从 y=0 一直排到 y=63。) */
#define OY(n)   ((u8g2_uint_t)((int16_t)OLED_Y0 + (int16_t)(n) * (int16_t)OLED_ROW_H))

/* ---------------------------------------------------------------------------
 *  极简字符串工具
 *  这里故意不用 sprintf: 它会把一大坨格式化代码(约 3~6 KB)链进来,
 *  而本机 Flash 只有 64 KB, 还是省着点用。
 * ------------------------------------------------------------------------- */

static char *PutStr(char *p, const char *s)
{
    while (*s != '\0')
    {
        *p++ = *s++;
    }
    return p;
}

/* 定宽右对齐输出整数。width 是最小字符宽度, 不够就用空格补齐。 */
static char *PutI32(char *p, int32_t v, uint8_t width)
{
    char     tmp[12];
    uint8_t  n   = 0;
    uint8_t  neg = 0;
    uint32_t u;

    if (v < 0)
    {
        neg = 1u;
        u   = (uint32_t)(-(v + 1)) + 1u;   /* 这样写是为了 -2147483648 也不溢出 */
    }
    else
    {
        u = (uint32_t)v;
    }

    do
    {
        tmp[n++] = (char)('0' + (uint8_t)(u % 10u));
        u /= 10u;
    } while (u != 0u);

    while ((uint8_t)(n + neg) < width)
    {
        tmp[n++] = ' ';
    }
    if (neg != 0u)
    {
        tmp[n++] = '-';
    }

    while (n > 0u)
    {
        *p++ = tmp[--n];
    }
    return p;
}

/* 定宽输出"秒.十分位", 例如 12345 ms -> " 12.3"(width=5)。
 * 为什么不用 sprintf: 它会把约 3~6 KB 的格式化代码链进来, 本机 Flash 只有 64 KB。
 * 计时只要 0.1 s 精度 —— 6x10 字体下小数点后一位就够了。 */
static char *PutTime(char *p, uint32_t ms, uint8_t width)
{
    uint32_t tenths = ms / 100u;                  /* 以 0.1 s 为单位的计数 */
    uint8_t  d10    = (uint8_t)(tenths % 10u);
    uint32_t sec    = tenths / 10u;

    if (width > 2u)
    {
        p = PutI32(p, (int32_t)sec, (uint8_t)(width - 2u));   /* 右对齐补空格 */
    }
    *p++ = '.';
    *p++ = (char)('0' + (char)d10);
    return p;
}

/* ---------------------------------------------------------------------------
 *  对外接口
 * ------------------------------------------------------------------------- */

void Oled_Init(void)
{
    u8g2_Setup_sh1106_128x64_winstar_f(&s_u8g2, U8G2_R0,
                                       u8x8_byte_hal_spi,
                                       u8x8_gpio_and_delay_hal);

    u8g2_InitDisplay(&s_u8g2);          /* 走一遍 SH1106 的复位/初始化序列 */
    u8g2_SetPowerSave(&s_u8g2, 0);      /* 0 = 打开显示 */
    u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);
    u8g2_SetFontMode(&s_u8g2, 1);        /* 1 = 不透明(带背景), 免得有残影 */
    u8g2_SetFontPosTop(&s_u8g2);         /* DrawStr 的 y 改成"字形顶部" */

    /* 开机自检画面。★ 最后一行原来写在 y=60, 加上字高已经超过 64 行 ——
     * 那一行【必然画到屏外】, 是硬 bug。现在全部改用 OY() 统一给坐标,
     * 5 行落在 y = 4 ~ 40 之间, 上下都有余量。
     * 能看到最后一行 "the screen is OK" 就说明屏和 SPI 都是好的。 */
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_DrawStr(&s_u8g2, 5, OY(0), "TEST1 TRACK CAR");
    u8g2_DrawStr(&s_u8g2, 5, OY(1), "OLED  SH1106 128x64");
    u8g2_DrawStr(&s_u8g2, 5, OY(2), "SPI2  PB13/PB15");
    u8g2_DrawStr(&s_u8g2, 5, OY(3), "if you can read this");
    u8g2_DrawStr(&s_u8g2, 5, OY(4), "the screen is OK");
    u8g2_SendBuffer(&s_u8g2);

    HAL_Delay(300);                       /* 让人来得及看到上面这行字 */

    s_next_ms = HAL_GetTick() + OLED_REFRESH_MS;
}

void Oled_Refresh(void)
{
    char     line[32];
    char    *p;
    uint8_t  mask;
    uint8_t  i;
    int16_t  err;
    int16_t  base, diff, lpwm, rpwm;
    uint16_t lost;
    uint16_t wide_evt;                 /* ★ 压到宽线的累计次数(标定用) */
    int32_t  wide_cnt;                 /* ★ 本圈第一次触发的里程 */
    uint8_t  wide_max;                 /* ★ 本圈最宽的一拍压了几路 */
    int16_t  bmin_ok;                  /* ★ 本圈"看见线时"最小的 base */
    int16_t  bmin_all;                 /* ★ 本圈"含丢线"在内最小的 base */
    uint16_t lmax;                     /* ★ 本圈最长连续丢线拍数 */
    int32_t  bmin_dist;                /* ★ 上面那个最小值出现时的里程 */

    mask = Track_GetMask();
    err  = Track_GetLineError(mask);
    lost = Track_GetLostCount();
    Track_GetOutput(&base, &diff, &lpwm, &rpwm);
    /* width 那一项不需要(屏上放不下), 传 0 = 不要 —— 见 track.h 的说明。 */
    Track_GetWideEvent(&wide_evt, &wide_cnt, 0);
    wide_max = Track_GetWidthMax();
    Track_GetSlowStats(&bmin_ok, &bmin_all, &lmax, &bmin_dist);

    u8g2_ClearBuffer(&s_u8g2);

    /* --- 第 1 行: 运行状态 + 跑圈计时 (2026-09-29 重排) ---
     * 6x10 字体下屏宽 128px = 21 个字符, 一行放不下"状态 + 两个极性 + 两个
     * 计时", 所以按状态分开显示:
     *   运行中 -> 以计时为主(标定值跑的时候本来也不用看):
     *             RUN T12.3 L11.8      T = 本次    L = 上一圈
     *   停止时 -> 把两个极性亮出来(标定时要盯), 再显示刚跑完的成绩:
     *             STOP PL1 TS-1 T12.3
     * ★ 放【两个】数字位是因为跑第二圈时能直接看到上一圈多少 —— 这对
     *   判断"改了这个参数到底快了没有"比什么都有用。 */
    if (Track_GetEnabled() != 0)
    {
        p = PutStr(line, "RUN T");
        p = PutTime(p, Stopwatch_GetNowMs(), 5);
        p = PutStr(p, " L");
        p = PutTime(p, Stopwatch_GetLastMs(), 5);
    }
    else if (Track_GetStopReason() != 0u)
    {
        /* ★ 2026-09-29: 【自动停下】的车要把原因写清楚, 不要让人去猜
         *   是按键停的、还是丢线停的。
         *     1 = 压到停止线  -> "HIT LINE T xx.x"
         *     2 = 里程到一圈  -> "LAP DONE T xx.x"
         *   两个入口都是自动断油, 计时表随之冻结, 那个数字就是这圈成绩。
         *   ★ 为什么要区分: 这正是"开始挪那条线"时唯一能用的线索 ——
         *     如果是 LAP DONE, 说明车是按【里程】停的(线没起作用);
         *     如果是 HIT LINE, 说明线被认出来了。 */
        if (Track_GetStopReason() == 2u)
        {
            p = PutStr(line, "LAP DONE T");
        }
        else
        {
            p = PutStr(line, "HIT LINE T");
        }
        p = PutTime(p, Stopwatch_GetNowMs(), 5);
    }
    else
    {
        p = PutStr(line, "STOP PL");
        p = PutI32(p, (int32_t)Track_GetActiveLevel(), 1);
        p = PutStr(p, " TS");
        p = PutI32(p, (int32_t)Track_GetTurnSign(), 2);
        p = PutStr(p, " T");
        p = PutTime(p, Stopwatch_GetNowMs(), 5);
    }
    *p = '\0';
    u8g2_DrawStr(&s_u8g2, 0, OY(0), line);

    /* --- 第 2 行: 跑的时候看"现在", 停下来看"这一圈最差的一拍" ---
     * 跑: E 误差 + LOST 连续丢线拍数
     * 停: BMIN 26/19 = 本圈 base 最小值(看见线时 / 含丢线拍), LMAX = 最长连续丢线拍数
     * ★ 为什么要换: 车一停, E 和 LOST 就是两个死数(车都不动了), 而"这一圈到底
     *   在哪掉过速"恰恰只有停车后才看得清 —— 实车"后面的弯明显减速"就是靠这
     *   三个数定因的(判读表见 track.h 的 Track_GetSlowStats)。 */
    if (Track_GetEnabled() != 0)
    {
        /* --- 第 2 行(运行中): 夹线误差 + 连续丢线拍数 --- */
        p = PutStr(line, "E");
        p = PutI32(p, (int32_t)err, 4);
        p = PutStr(p, "  LOST");
        p = PutI32(p, (int32_t)lost, 4);
    }
    else
    {
        /* --- 第 2 行(已停车): 本圈掉速档案 --- */
        p = PutStr(line, "BMIN");
        p = PutI32(p, (int32_t)bmin_ok, 3);
        p = PutStr(p, "/");
        p = PutI32(p, (int32_t)bmin_all, 3);
        p = PutStr(p, " LMAX");
        p = PutI32(p, (int32_t)lmax, 3);
    }
    *p = '\0';
    u8g2_DrawStr(&s_u8g2, 0, OY(1), line);

    /* --- 第 3、4 行: 跑的时候看控制量, 停下来看"标定数据" ---
     * ★ 为什么停下来要换内容: 车停了以后 B/DIFF/L/R 这三个数恒定不变,
     *   没有任何信息量; 而"这一圈在哪儿误判了"恰恰是停车后才该看的。
     *   所以停车后把这两行让给标定里程门槛要用的数:
     *     第 3 行  EVT  2  MAXW 5   本圈触发 2 次 / 最宽的一拍压了 5 路
     *     第 4 行  WIDE@ 12345      本圈【第一次】触发时的里程计数
     *   ★ 读法(标定里程门槛):
     *     ① 先看第 6 行 ENC 两个数取平均 = 一圈的总计数 -> 填 MILEAGE_LAP_COUNT
     *     ② 看第 4 行的 WIDE@(第一次触发位置): 若它明显小于一圈总计数, 说明
     *        中途误判了 -> 里程门槛 STOP_LINE_ARM_COUNT 要设得比它更大。
     *        若它和一圈总计数差不多, 说明触发就是真停止线 -> 门槛设一圈的 80% 即可
     *     ③ 看 MAXW: 它是"整圈最宽压了几路" -> 直接决定 STOP_LINE_MIN_ACTIVE
     *        该填几(填 3 还是 5, 不再靠猜)。
     *   ★ 这三个数【按按键停车后不会消失】(2026-09-29 修): 以前停车路径会顺手
     *     把触发次数清成 0, 屏上永远只剩 EVT 0, 根本没法标定。 */
    if (Track_GetEnabled() != 0)
    {
        /* --- 第 3 行: 基础速度 + 差速 (★ 调参时最该盯的一行) --- */
        p = PutStr(line, "B");
        p = PutI32(p, (int32_t)base, 4);
        p = PutStr(p, "  DIFF");
        p = PutI32(p, (int32_t)diff, 4);
        *p = '\0';
        u8g2_DrawStr(&s_u8g2, 0, OY(2), line);

        /* --- 第 4 行: 真正下发的左/右轮 PWM --- */
        p = PutStr(line, "L");
        p = PutI32(p, (int32_t)lpwm, 4);
        p = PutStr(p, "  R");
        p = PutI32(p, (int32_t)rpwm, 4);
        *p = '\0';
        u8g2_DrawStr(&s_u8g2, 0, OY(3), line);
    }
    else
    {
        /* --- 第 3 行: 触发次数 + 本圈最大连续压黑路数 --- */
        p = PutStr(line, "EVT");
        p = PutI32(p, (int32_t)wide_evt, 3);
        p = PutStr(p, "  MAXW ");
        p = PutI32(p, (int32_t)wide_max, 1);
        *p = '\0';
        u8g2_DrawStr(&s_u8g2, 0, OY(2), line);

        /* --- 第 4 行(已停车): 两个"位置" + 单位说明 ---
         *   SLOW@543K  本圈【最慢那一拍】出现在多少里程(单位 = 1000 计数)
         *   WIDE@867K  本圈【第一次压到宽线】时的里程(设停车门槛就看它)
         * ★ 为什么带 K 并且要除以 1000: 一圈就有 140 万计数(见第 6 行),
         *   原样打印必然撑破 21 个字符的行宽(旧版 "WIDE@867186" 就是这么溢出的)。
         *   除以 1000 之后 4 位就够, 而且【两个位置可以直接比大小】:
         *   一圈约 1414K, B/C/D 三个点分别在 345K / 707K / 1060K 附近 ——
         *   SLOW@ 落在 345~707K 就是【第一个弯】, 落在 1060~1414K 就是【第二个弯】。 */
        p = PutStr(line, "SLOW@");
        p = PutI32(p, bmin_dist / 1000, 4);
        p = PutStr(p, "K WIDE@");
        p = PutI32(p, wide_cnt / 1000, 4);
        p = PutStr(p, "K");
        *p = '\0';
        u8g2_DrawStr(&s_u8g2, 0, OY(3), line);
    }

    /* --- 第 5 行: 五路探头, 最左 -> 最右 = bit0 -> bit4 --- */
    for (i = 0u; i < 5u; i++)
    {
        u8g2_uint_t x = (u8g2_uint_t)(16 + (int16_t)i * 20);

        if (((mask >> i) & 0x01u) != 0u)
        {
            u8g2_DrawBox(&s_u8g2, x, OY(4), 16, 9);      /* 实心 = 压到黑线 */
        }
        else
        {
            u8g2_DrawFrame(&s_u8g2, x, OY(4), 16, 9);    /* 空心 = 在白底上 */
        }
    }

    /* --- 第 6 行: 左右轮累计编码器计数 + A/B 点进度 ---
     * ★★ 单位 = 【1000 个计数】(2026-09-29 改; 之前打原值)。
     *   为什么必须除: 一圈约 140 万个计数, 原值一行根本放不下 —— 旧版屏上
     *   是 "ENC-16055181224006 M0", 15 个数字挤成一片、最后一个字符还被挤出
     *   屏外。除以 1000 后成了 "ENC  -1605   1224 M0", 清清楚楚。
     *   两个 7 位字段保留着, 只是里面装的是"千计数"。
     * ★ 这一行是【标定里程的唯一数据来源】: 跑完一圈读这两个数, 各乘 1000、
     *   取绝对值再平均, 填进 config.h 的 MILEAGE_LAP_COUNT。
     *   例: 屏上 -1605 / 1224 -> (1605+1224)/2 = 1414 -> 填 1414000。
     * ★ 末尾的 M0/M1/M2 = 已经跑到哪个标记点了:
     *     M0 = 还没到 B 点      M1 = 已过 B 点      M2 = 已过 A 点(一圈跑完)
     *   ★ 现在一直显示 M0 是【正常】的: MILEAGE_LAP_COUNT 还是 0(未标定),
     *     按设计里程判定整个不动作 —— 免得拿一个假数去触发提示和停车。
     *   ★ 到 B 点/到 A 点的时候会听到蜂鸣器响、看到 PC13 灯闪/亮 —— 屏上
     *     这个数就是给"没注意到声响"时回头核对用的。 */
    p = PutStr(line, "ENC");
    p = PutI32(p, Encoder_GetLeftCount()  / 1000, 7);
    p = PutI32(p, Encoder_GetRightCount() / 1000, 7);
    p = PutStr(p, " M");
    p = PutI32(p, (int32_t)Mileage_GetMark(), 1);
    *p = '\0';
    u8g2_DrawStr(&s_u8g2, 0, OY(5), line);

    u8g2_SendBuffer(&s_u8g2);
}

void Oled_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* 用无符号差值比较, 这样即使 HAL_GetTick() 溢出(约 49 天)也不会出问题 */
    if ((int32_t)(now - s_next_ms) < 0)
    {
        return;
    }

    s_next_ms = now + OLED_REFRESH_MS;

    Oled_Refresh();
}
