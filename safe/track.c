/*
 * track.c —— 五路循迹控制
 *
 * 传感器(数字输出): 左->右 PA3 PA4 PA5 PA6 PA7
 *   极性(黑线对应哪种电平)与转向极性: 默认值在 config.h, 运行中可切 ——
 *     停止时【双击 PA15】切传感器极性, 【三击 PA15】切转向极性。
 *   ★ 两个开关都会写进 Flash 保存, 断电/复位/重新烧录都不会丢
 *     (以前只存在内存里, 一次复位就退回默认值 —— "突然直冲"的元凶之一)。
 *
 * 误差(夹线定位: 取最左/最右被触发探头的中点), 单位 = 半个探头间距:
 *     -4 最左 ... 0 居中 ... +4 最右,   99 = 五路全丢
 *   误差 < 0 说明黑线在车左边 -> 应当向左转。
 *
 * 控制律:
 *     预测误差 = 误差 + 前视拍数 × 低通后的"误差变化速度"   ← 虚拟前视
 *     diff = Kp × 预测误差 / 2 + 曲率积分                    ← 提前打舵 + 弯道稳态量
 *     diff 每拍最多变 DIFF_SLEW                             ← ★ 差速变化率限幅
 *     base = BASE_SPEED, 打舵越狠 base 降得越多             ← 弯道自动收油
 *     左PWM = base + diff,   右PWM = base - diff
 *     知道方向却丢线 -> 降到 LOST_SPEED 再额外加一把力        ← 原地拧回来
 *     从没见过黑线时只允许【爬行】, 绝不按基础速度冲           ← 防"一启动就飞出去"
 *     启动后 RAMP_MS 内输出线性升起                          ← 软启动, 防起步窜车
 *
 * ★★ 关于"差速变化率限幅"(2026-09-25 加, 治"每次调速差速过大、偏离赛道"):
 *   实车反馈原话是"力度是够的, 是每次左右调速时左右差速过大, 偏离赛道了"。
 *   根因不是最大舵量, 而是【舵量的变化速度】: 误差从 +2 跳到 -2 时, 差速在
 *   一拍(5ms)之内就从 +14 变到 -14。电机机械时间常数 30~50ms 跟不上, 于是
 *   车拿到的是一个阶跃指令 —— 车头被猛地甩过去, 等传感器看回来已经转过头,
 *   再反向猛拧, 就这样一顿一顿地被甩出赛道。
 *   限幅后差速只能"爬"上去(3 PWM/拍 ≈ 55ms 到满舵, 与电机时间常数同量级),
 *   但【上限一点没动】, 持续偏一侧照样能到满舵 —— 不削弱克服静摩擦的力气。
 *   ⚠ 教训: "车头转过来了、车身往外滑"这句症状, 我先前解释成"力度不够
 *     (偏航静摩擦)", 被实车否掉了 —— 同一句话同样可以解释成"一次打得太猛"。
 *     症状描述可以有完全相反的病因, 所以只能"改一处、跑一圈、看结果"。
 *
 * ★★ 关于"曲率积分"(2026-09-25 加):
 *   弯道的【曲率】这个信息根本不在传感器信号里 —— 五路数字探头在 1.8cm 线上
 *   "正中"的窄带只有约 1mm 宽, 车实际稳定停在 err=±1: 直道 ±1, 弯道也 ±1。
 *   实测(仿真): 直道误差"同侧连续"最长 828 拍, 弯道 822 拍, 完全分不开。
 *   积分项把"误差持续偏向同一侧"这个稳态偏置自己学出来, 于是误差能回到 0
 *   而差速仍然维持着。它提供的是【弯道稳态偏置】这一件事。
 *   (它曾被写成"解决了过不去弯的万能药", 现已按实车证据把定位收窄。)
 *
 * ★★ 关于"虚拟前视"(2026-09-25 加, 取代了原来的 KD 微分项):
 *   原来的 KD = 本拍误差 − 上一拍误差, 只在那一拍(5ms)出力一次。电机+齿轮箱
 *   的机械时间常数约 30~50ms, 5ms 的指令还没等轮子转起来就没了 —— 这就是
 *   "进弯反应慢、等跑出弯道才转弯"的直接原因。仿真验证: 把 KD 从 8 加到 16、
 *   24, 在探头前视不足的场景里一次都没救回来。
 *   现在的做法: 把误差变化速度【低通】(时间常数 LEAD_TAU, 默认 100ms)再外推
 *   LEAD_TICKS 拍, 等于把探头虚拟地往前挪。低通的作用就是把原来那个 5ms 尖峰
 *   摊成持续 ~100ms 的指令, 电机才跟得上。
 *   另一个更根本的解法是物理上的: 把探头板往车头方向挪, 见 config.h 开头。
 */

#include "track.h"
#include "motor.h"
#include "mileage.h"   /* ★ 2026-09-29: 停止线的"里程门槛"要读里程 */
#include "config.h"

/* 行偏移误差里的"五路全丢"标志 */
#define TRACK_ERR_LOST   99

/* 软启动需要几拍走完(SIMPLE_RAMP_MS = 0 时不启用) */
#define RAMP_TICKS   ((uint16_t)((SIMPLE_RAMP_MS) / CTRL_PERIOD_MS))

/* ============================================================
 *                       开 关 与 状 态
 * ============================================================ */
/* 传感器极性: 黑线对应哪种电平。开机默认值来自 config.h, 之后可能被
 * Flash 里保存过的值覆盖(Track_LoadSettings)。 */
static uint8_t s_active_level = TRACK_ACTIVE_LEVEL_DEFAULT;

/* 转向极性。推导: err>0 = 触发的探头偏右(黑线在车右边) -> 车偏左 ->
 * 应向右转(左轮快/右轮慢); err<0 反之。实测反了只可能是
 * (a) 模块排针左右顺序接反 或 (b) 左右电机接线对调, 现象完全一样,
 * 所以一个开关就能同时治好, 不用改线。 */
static int8_t  s_turn_sign = (int8_t)TRACK_TURN_SIGN_DEFAULT;

static int16_t  s_track_mode = 0;  /* 1 = 循迹中 */
static int16_t  s_lost_dir   = 0;  /* 丢线前的偏离方向: +1 偏右 / -1 偏左 / 0 还不知道 */
static int16_t  s_line_last  = 0;  /* 上一拍误差(用来求"误差变化速度") */
static int32_t  s_dlp        = 0;  /* 误差变化速度的低通值, 放大 256 倍保存(虚拟前视用) */
static int32_t  s_trim_q     = 0;  /* 曲率积分累计的稳态差速偏置, 同样按 256 放大(Q8) */
static int32_t  s_diff_out   = 0;  /* 上一拍真正下发的差速(差速变化率限幅用, 见 config.h) */
/* ★ 2026-09-28 新增: 把真正下发的几个量存一份, 供 OLED 显示(只读, 不参与控制)。
 *   加它的直接原因: 实车反馈"每次左右调速时左右差速过大", 而差速/左右 PWM
 *   以前根本没有办法看到 —— 本工程没有串口, OLED 就是唯一的观察窗口。 */
static int16_t  s_base_out   = 0;  /* 本拍的基础速度(已含弯道收油) */
static int16_t  s_left_out   = 0;  /* 本拍实际下发的左轮 PWM */
static int16_t  s_right_out  = 0;  /* 本拍实际下发的右轮 PWM */
static uint16_t s_lost_cnt   = 0;  /* 连续丢线拍数 */
static uint16_t s_ramp_tick  = 0;  /* 软启动进度(拍) */
/* ★ 2026-09-29 新增: 停止线检测(需求"检测到停止线就自动停止")。
 *   判定规则、启动保护、里程门槛都在 config.h 的 STOP_LINE_* 一组宏里。
 *   ★ 2026-09-29 下午修订: 原来的"连续 3 路"判据实车【误判严重】(车没跑完
 *     就停了), 因为"弯道里斜着压普通线"也能凑出 3 路 —— 与停止线分不开。
 *     现在加了【里程门槛】, 并把判定结果量化记录(连续几路 / 当时里程)。
 */
static uint16_t s_sl_tick    = 0;  /* 启动后经过的拍数(用来算启动保护期) */
static uint16_t s_sl_cnt     = 0;  /* 连续命中"连续 N 路压黑"的拍数 */
static uint8_t  s_stop_reason = 0;  /* 因为什么自动停的: 0=没自动停 1=压线 2=里程 */
/* ★ 供标定用的量化记录(不管最终停不停车都会记):
 *   标定里程门槛时就靠它 —— "跑到多少里程、压了几路"一目了然。 */
static uint16_t s_sl_evt     = 0;  /* 本次启动以来累计触发次数 */
static int32_t  s_sl_evt_cnt = 0;  /* 最近一次触发时的里程(编码器平均计数) */
static uint8_t  s_sl_evt_w   = 0;  /* 最近一次触发时"连续几路同时压黑" */
/* ★★ 2026-09-29 新增: 本圈见过的【最大连续压黑路数】。
 *   为什么要它: 触发次数(EVT)只在"宽度够 + 连续够拍"时才加一。万一停止线
 *   其实只压出 3 路(门槛却是 4), 那就一个字都不记 —— 屏上全 0, 人根本分不清
 *   是"没压到线"还是"宽度不够"。有了 wmax 就一眼能定 MIN_ACTIVE 该填几:
 *      跑完看到 wmax = 3  -> 停止线只能压 3 路 -> MIN_ACTIVE 改 3
 *      看到 wmax = 5      -> MIN_ACTIVE 改成 5 最可靠
 *   这是"让数据决定方向"的具体做法, 不要删。 */
static uint8_t  s_wmax       = 0;  /* 本圈最大连续压黑路数 */

/* ★★ 2026-09-29 新增: 本圈的"掉速档案"(回答"为什么后面的弯明显减速")。
 *   屏上第 3 行的 B 是【本拍】的指令值, 车一停就没了 —— 一圈里"最慢的那一拍
 *   慢到多少、在哪个弯、是因为收油还是因为丢线", 以前全看不见。
 *   这几个数在停车后【不清】(只在按启动时归零), 所以跑完可以慢慢看。
 *   含义与判读见 track.h 的 Track_GetSlowStats()。 */
static int16_t  s_bmin_ok   = 0;   /* 看见线时 base 的最小值(0 = 还没采到) */
static int16_t  s_bmin_all  = 0;   /* 含丢线拍在内 base 的最小值 */
static uint16_t s_lmax      = 0;   /* 本圈最长连续丢线拍数 */
static int32_t  s_bmin_dist = 0;   /* 上面那个最小值出现时的里程 */

/* ============================================================
 *              极 性 设 置 的 掉 电 保 存 (Flash 最后一页)
 * ------------------------------------------------------------
 * 为什么需要: 这两个开关是"上车试出来"的(本工程没有串口, 只能试)。
 * 如果只存在内存里, 那么任何一次复位/断电/重新烧录都会把它退回
 * config.h 的默认值 —— 表现就是"昨天还好好的车, 今天一启动就往
 * 赛道对角直冲"。写进 Flash 之后就一直有效。
 *
 * 位置: Flash 最后一页(本芯片 64KB 时是 0x0800FC00)。
 * 程序只有几 KB, 离这一页很远; 烧录用默认的 "Erase Sectors" 时
 * 也不会被擦掉(用 "Erase Full Chip" 才会, 那就恢复成默认值)。
 * ============================================================ */
#define CFG_MAGIC   0x534Bu          /* "SK" */
#define CFG_VER     2u               /* 记录布局版本(只有改布局时才需要 +1) */

/* ★ 出厂默认值的"签名"。config.h 里那两个默认值一旦被改动, 签名就跟着变,
 *   于是 Flash 里保存的旧标定会自动作废, 直接采用新的默认值。
 *   为什么必须有它: 否则会出现"明明改了默认值、也重新烧录了, 车的行为却
 *   一点没变" —— 因为 Flash 里的旧值还盖在上面。这正是本项目踩过的坑,
 *   所以做成自动的, 不靠人工记得去改版本号。 */
#define CUR_DEFSIG   ((uint8_t)(((TRACK_ACTIVE_LEVEL_DEFAULT) ? 1u : 0u) << 1) \
                    | (uint8_t)(((TRACK_TURN_SIGN_DEFAULT) > 0) ? 1u : 0u))

typedef struct
{
    uint16_t magic;
    uint16_t ver;
    uint8_t  level;                  /* 传感器极性 0/1 */
    int8_t   sign;                   /* 转向极性 +1/-1 */
    uint8_t  defsig;                 /* 写这条记录时, 固件的默认值签名 */
    uint8_t  rsv;                    /* 保留, 固定 0 */
    uint16_t chk;                    /* 校验: magic^ver^level^sign^defsig */
    uint16_t pad;                    /* 补齐到 12 字节 = 3 个 32bit 字 */
} TrackCfg_t;                        /* 12 字节 */

/* 上面这条记录占几个 32bit 字(自动跟着结构体走, 改结构不用改这里) */
#define CFG_WORDS   ((sizeof(TrackCfg_t) + 3u) / 4u)

static uint32_t Cfg_Addr(void)
{
    /* FLASH_SIZE 在 0x1FFFF7E0, 单位 KB; 读不到就按 64KB(C8T6)算。
     * 页大小: F103 容量 <= 128KB 时为 1KB, 更大时为 2KB。 */
    uint32_t kb = *(volatile uint16_t *)0x1FFFF7E0u;
    uint32_t page;

    if (kb < 32u || kb > 512u) kb = 64u;
    page = (kb > 128u) ? 2048u : 1024u;

    return (uint32_t)(0x08000000u + kb * 1024u - page);
}

static uint16_t Cfg_Chk(uint8_t level, int8_t sign, uint8_t defsig)
{
    return (uint16_t)(CFG_MAGIC ^ CFG_VER ^ (uint16_t)level
                      ^ (uint16_t)(uint8_t)sign ^ (uint16_t)defsig);
}

void Track_LoadSettings(void)
{
    const TrackCfg_t *p = (const TrackCfg_t *)Cfg_Addr();

    if (p->magic == CFG_MAGIC && p->ver == CFG_VER
        && p->level <= 1u
        && (p->sign == 1 || p->sign == -1)
        && p->defsig == CUR_DEFSIG
        && p->chk == Cfg_Chk(p->level, p->sign, p->defsig))
    {
        s_active_level = (uint8_t)p->level;
        s_turn_sign    = (int8_t)p->sign;
    }

}

static void Cfg_Save(void)
{
    const TrackCfg_t *cur = (const TrackCfg_t *)Cfg_Addr();
    TrackCfg_t cfg;
    FLASH_EraseInitTypeDef er;
    uint32_t page_err = 0;
    uint32_t addr;
    uint32_t w[CFG_WORDS];
    uint8_t *b;
    uint8_t  i;
    uint32_t k;

    cfg.magic  = CFG_MAGIC;
    cfg.ver    = CFG_VER;
    cfg.level  = s_active_level;
    cfg.sign   = s_turn_sign;
    cfg.defsig = CUR_DEFSIG;
    cfg.rsv    = 0u;
    cfg.pad    = 0u;
    cfg.chk    = Cfg_Chk(cfg.level, cfg.sign, cfg.defsig);

    /* 和 Flash 里已有的内容一样就不写(少擦几次) */
    if (cur->magic == cfg.magic && cur->ver == cfg.ver
        && cur->level == cfg.level && cur->sign == cfg.sign
        && cur->defsig == cfg.defsig
        && cur->chk == cfg.chk)
    {
        return;
    }

    addr = Cfg_Addr();
    b    = (uint8_t *)&cfg;
    for (i = 0; i < CFG_WORDS; i++)
    {
        w[i] = (uint32_t)b[i * 4u]
             | ((uint32_t)b[i * 4u + 1u] << 8)
             | ((uint32_t)b[i * 4u + 2u] << 16)
             | ((uint32_t)b[i * 4u + 3u] << 24);
    }

    __disable_irq();                 /* 擦写期间不让控制中断插进来 */
    if (HAL_FLASH_Unlock() == HAL_OK)
    {
        er.TypeErase   = FLASH_TYPEERASE_PAGES;
        er.Banks       = 0;
        er.PageAddress = addr;
        er.NbPages     = 1;
        if (HAL_FLASHEx_Erase(&er, &page_err) == HAL_OK)
        {
            for (k = 0u; k < CFG_WORDS; k++)
            {
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  addr + k * 4u, (uint64_t)w[k]);
            }
        }
        HAL_FLASH_Lock();
    }
    __enable_irq();
}

/* ============================================================
 *              两 个 极 性 开 关 (切换即保存)
 * ============================================================ */
uint8_t Track_GetActiveLevel(void) { return s_active_level; }
int8_t  Track_GetTurnSign(void)    { return s_turn_sign; }

void Track_ToggleActiveLevel(void)
{
    s_active_level = (s_active_level != 0u) ? 0u : 1u;
    Cfg_Save();
}

void Track_ToggleTurnSign(void)
{
    s_turn_sign = (int8_t)(-s_turn_sign);
    Cfg_Save();
}

/* ============================================================
 *                        传 感 器 读 取
 * ============================================================ */

/* 读取五路传感器, bit(i)=1 表示第 i 路(0=最左)检测到黑线。
 * 比较目标就是 s_active_level, 所以极性一改整个逻辑(含丢线判断)一起跟着对。 */
uint8_t Track_GetMask(void)
{
    uint8_t mask = 0;

    mask |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == (GPIO_PinState)s_active_level) ? 0x01u : 0u;
    mask |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == (GPIO_PinState)s_active_level) ? 0x02u : 0u;
    mask |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == (GPIO_PinState)s_active_level) ? 0x04u : 0u;
    mask |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == (GPIO_PinState)s_active_level) ? 0x08u : 0u;
    mask |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == (GPIO_PinState)s_active_level) ? 0x10u : 0u;

    return mask; 
}

/* 由传感器位图算行偏移误差。
 *
 * ★ 用【夹线定位】: 取"最左被触发的路"和"最右被触发的路"的【中点】,
 *   而不是"把所有被触发的路取平均"。
 *   为什么: 黑线较宽时正常行驶会【相邻两路同时压线】, 取平均会把
 *       压住第2、3路 -> (-1+0)/2 = 0
 *       压住第3、4路 -> ( 0+1)/2 = 0
 *   两种明明"偏左半格/偏右半格"却都算成 0(C 整除丢掉了 ±0.5),
 *   车就完全不纠偏, 分辨率少一半。取两端中点: 一个是 -1、一个是 +1。
 *
 * 返回值单位 = 【半个探头间距】, 范围 -4 ~ +4:
 *   -4 只有第1路        -1 第2、3路同时        +2 只有第4路
 *   -3 第1、2路同时      0 只有第3路 / 五路全  +3 第4、5路同时
 *   -2 只有第2路         +1 第3、4路同时       +4 只有第5路
 */
int16_t Track_GetLineError(uint8_t mask)
{
    static const int16_t pos[5] = { -2, -1, 0, 1, 2 };
    int16_t lo = -1, hi = -1;
    int     i;

    if (mask == 0)    return TRACK_ERR_LOST;  /* 五路全丢 */
    if (mask == 0x1F) return 0;               /* 五路全压 = 十字口/宽黑区, 视为居中 */

    for (i = 0; i < 5; i++)
    {
        if (mask & (1u << i))
        {
            if (lo < 0) lo = i;               /* 第一路被触发的位置 */
            hi = i;                           /* 最后一路被触发的位置 */
        }
    }

    /* lo、hi 是整格坐标, 相加正好是"中点 ×2" —— 单位就是半个探头间距,
     * 而且不需要除法(不会丢精度)。 */
    return (int16_t)(pos[lo] + pos[hi]);
}

/* 启动前自检: 探头能不能看到黑线?
 * 连续采几次, 只要有任意一次看到就认为可以启动(容忍偶尔漏检/反光)。
 * ★ 这一步是"防止一启动就冲出去"的第一道闸: 看不到线时干脆不动。 */
uint8_t Track_StartOk(void)
{
    uint8_t i;
    uint8_t mask = 0;

    for (i = 0; i < 8u; i++)
    {
        mask |= Track_GetMask();
        HAL_Delay(5);
    }
    return (mask != 0u) ? 1u : 0u;
}

/* 限幅 */
static int32_t Limit(int32_t v, int32_t lim)
{
    if (v > lim)  return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* ============================================================
 *                       控 制 接 口
 * ============================================================ */
void Track_Init(void)
{
    s_lost_dir  = 0;
    s_line_last = 0;
    s_dlp       = 0;
    s_trim_q    = 0;
    s_diff_out  = 0;
    s_lost_cnt  = 0;
    s_ramp_tick = 0;
    /* ★ 停止线检测的状态也一起复位。s_stop_reason 在这里清是有意的:
     *   它只在"被停止线停下"之后为 1 供屏上显示, 而下次启动会走
     *   Track_SetEnabled(1) -> 本函数, 提示自动熄灭。 */
    s_sl_tick   = 0;
    s_sl_cnt    = 0;
    s_stop_reason  = 0;
    /* ★ 2026-09-29 修订: 这里【不再】清事件统计(s_sl_evt / s_wmax)。
     *   为什么要改: Track_Init() 会被【停车】路径调用(main.c 按键停止 ->
     *   Track_SetEnabled(0) -> 本函数), 于是"触发次数"恰好在按键松手那一刻
     *   被擦成 0, 屏上永远只剩 EVT 0 —— 标定数据拿不到。
     *   现在改成【只在启动时清零】, 见 Track_SetEnabled()。
     *   (s_sl_evt_cnt / s_sl_evt_w 是"第一次/最近一次"的快照, 同样保留,
     *    不清才能分清"这圈没触发"和"被清了"。) */
}

void Track_SetEnabled(int16_t enable)
{
    s_track_mode = enable;
    Track_Init();

    /* ★ 统计只在【启动】时归零 —— 每次按启动就是新的一圈。
     *   停车(enable = 0)时一个字都不动: 屏上要保留刚跑完那一圈的标定数据,
     *   不管是按键停的、双击/三击切极性停的, 还是被停止线停的。 */
    if (enable != 0)
    {
        s_sl_evt  = 0u;
        s_wmax    = 0u;
        /* ★ 掉速档案同样"一圈一清零" */
        s_bmin_ok   = 0;
        s_bmin_all  = 0;
        s_lmax      = 0u;
        s_bmin_dist = 0;
    }
}

int16_t Track_GetEnabled(void)
{
    return s_track_mode;
}

/* ★ 2026-09-28 新增(只读): 取回本拍真正下发给电机的几个量, 给 OLED 显示用。
 *   任何一个指针传 NULL 都可以, 表示不需要那一项。 */
void Track_GetOutput(int16_t *base, int16_t *diff, int16_t *left, int16_t *right)
{
    if (base  != 0) { *base  = s_base_out; }
    if (diff  != 0) { *diff  = (int16_t)s_diff_out; }
    if (left  != 0) { *left  = s_left_out; }
    if (right != 0) { *right = s_right_out; }
}

/* 连续丢线拍数(0 = 正常看见线)。OLED 上显示成 "LOST n"。 */
uint16_t Track_GetLostCount(void)
{
    return s_lost_cnt;
}

/* ============================================================
 *        自 动 停 车 (两个入口: ①压到停止线  ②里程到一圈)
 * ------------------------------------------------------------
 * 需求1(2026-09-29 下午): "检测到停止线就自动停止"。
 * 需求2(2026-09-29 晚):   "我想实现从起步点出发, 在起步点停止"。
 *   ★ 两个需求【不能只用一个判据满足】—— 见下面"为什么必须有两个入口"。
 *
 * ---- 入口① 压到停止线: 三个维度同时成立 ----
 *   ★★ 2026-09-29 下午【重写】: 原来的"连续 3 路 + 连续 2 拍"实车误判严重
 *      (车没跑完就被停住)。根因见 config.h 里那段长注释 —— 简单说就是
 *      "弯道里斜着压普通线"也能凑出 3 路, 和停止线【分不开】。
 *   现在的判据 = 【三个维度同时成立】:
 *   ① 里程门槛: Mileage_GetCount() >= STOP_LINE_ARM_COUNT
 *        误判都发生在中途, 而停止线只在终点前 —— 这一条就把它们挡掉了。
 *   ② 宽度: 探头里【连续 STOP_LINE_MIN_ACTIVE 路】同时压黑(默认 4)。
 *   ③ 时间: 连续 STOP_LINE_CONFIRM 拍都成立(默认 6 拍 = 30 ms,
 *        车速 0.26 m/s 约等于 8 mm) —— 滤掉单拍毛刺。
 *   ★ 判定结果【无论停不停车都会被记录下来】(s_sl_evt / _cnt / _w):
 *     这正是标定里程门槛需要的数据 —— 跑一圈就能看到"在第几米处误判了、
 *     压了几路"。所以 STOP_LINE_ENABLE = 0 时本函数依然在工作, 只是不停车。
 *     ★ 这是"让数据决定方向"的具体做法, 不要改成"关了就不记录"。
 *
 * ---- 入口② 里程到一圈 (STOP_MILEAGE_*) ----
 *   ★★ 为什么必须有它: **压线停车只能把车停在【那条线的位置】。**
 *   实测那条 5 cm 线在起步点前方 850K(一圈的 61%)处, 而用户要的是
 *   "从起步点出发、在起步点停止" —— 靠压线只会停在 61% 处, 回不到起步点。
 *   要停在起步点, 参照物只能是【里程】。
 *   ★ 两个入口可以同时开着, 互不干扰, 而且互相印证:
 *     · 线正好在起步点(H 题的做法): 两者在同一瞬间成立 -> 一起把车停住,
 *       压线那一侧还额外提供了"物理位置"这个高精度参照。
 *     · 线不在起步点(现在的布局): 压线那一侧被里程门槛挡住(不误停),
 *       由里程把车停在起步点。
 *
 * ★ 停车动作 = 立即断油(Motor_All_Stop), 用户 2026-09-29 选定。
 *   比"先收油再停"少滑 2~3 cm, 代价是轮胎有一点点抱死拖滑。
 *   两个入口共用 StopCar(), 保证动作完全一致。
 *
 * ============================================================ */
/* ============================================================
 *              停 车 动 作 (压线 / 里程, 两个入口共用)
 * ------------------------------------------------------------
 * ★ 2026-09-29 抽成函数: 原来只有"压到停止线"一个入口, 现在多了"里程到一圈",
 *   两个入口的停车动作必须完全一样(断油 + 清状态 + 保留"因为什么停的"标记),
 *   复制两份迟早会改漏一处。
 * ★ 这里【故意】直接改 s_track_mode 而不调 Track_SetEnabled(0):
 *   后者会调 Track_Init(), 把 s_stop_reason 一起清掉, 屏上就看不到停车原因了。
 *   所以状态在这里手动清一遍。
 *   reason: 1 = 压到停止线(屏上 HIT LINE)  2 = 里程到一圈(屏上 LAP DONE)
 * ============================================================ */
static void StopCar(uint8_t reason)
{
    s_stop_reason   = reason;
    s_track_mode = 0;
    s_lost_dir   = 0;
    s_line_last  = 0;
    s_dlp        = 0;
    s_trim_q     = 0;
    s_diff_out   = 0;
    s_lost_cnt   = 0;
    s_ramp_tick  = 0;
    Motor_All_Stop();
}

uint8_t Track_StopDetect(uint8_t mask)
{
    uint8_t i;
    uint8_t run  = 0u;                 /* 当前连续压黑的路数 */
    uint8_t best = 0u;                 /* 这一拍里最长的连续路数 */

    if (s_track_mode == 0)
    {
        return 0u;                     /* 没在跑, 不检测 */
    }

    /* 启动保护: 刚起步这段时间不检测。车是【停在启停线上】起步的(H 题
     * A 点), 没有这一段保护, 一按启动就会立刻"压到停止线"把自己停住。 */
    s_sl_tick++;
    if (s_sl_tick < (uint16_t)STOP_LINE_ARM_TICKS)
    {
        s_sl_cnt = 0u;
        return 0u;
    }

#if (STOP_MILEAGE_ENABLE)
    /* ---- ★★ 里程停车(2026-09-29 新增): 与那条线完全无关 ----
     * 需求原话: "我想实现从起步点出发, 在起步点停止"。
     * ★ 为什么不能只靠那条 5 cm 线: 线是【固定的物理位置】, 压线停车只能把车
     *   停在【线的位置】。而实测那条线在起步点前方 850K(一圈的 61%)处 ——
     *   也就是说"压线停"会在跑完 61% 圈时就停下, 回不到起步点。
     *   要让车停在【起步点】, 唯一的参照物就是【里程】。
     * ★ 里程精度: 编码器计数与真实距离成正比, 标定过一圈之后误差量级是
     *   百分之几(≈几厘米)。想做 H 题的"停车偏差 <=2cm", 就把那条线挪到
     *   起步点、靠压线停(那时两个判据会在同一瞬间成立, 互相印证)。
     * ⚠ 这个判定【不看传感器】, 所以就算中途丢线、或者赛道上有别的黑斑,
     *   它都会准时停 —— 也是一道"跑完就停"的保险。 */
    if (Mileage_GetCount() >= (int32_t)STOP_MILEAGE_COUNT)
    {
        StopCar(2u);                   /* 2 = 里程到, 不是压线 */
        return 1u;
    }
#endif

    /* 找【最长连续压黑的路数】。
     * ★ 用"数连续 1 的个数"而不是"滑动窗口匹配": 得到的 best 本身就是一个
     *   有用的量化数据(能显示在屏上), 标定时直接看它该设几路。 */
    for (i = 0u; i < 5u; i++)
    {
        if (((mask >> i) & 0x01u) != 0u)
        {
            run++;
            if (run > best)
            {
                best = run;
            }
        }
        else
        {
            run = 0u;
        }
    }

    /* ★ 先记"本圈最大连续路数" —— 不管这一拍够不够门槛都要记。
     *   这是判"停止线到底能压几路"的唯一手段(EVT 是够了门槛才加一的,
     *   门槛设高了就永远是 0, 分不清原因)。 */
    if (best > s_wmax)
    {
        s_wmax = best;
    }

    if (best < (uint8_t)STOP_LINE_MIN_ACTIVE)
    {
        s_sl_cnt = 0u;                 /* 这一拍不够宽, 连续确认归零 */
        return 0u;
    }

    s_sl_cnt++;
    if (s_sl_cnt < (uint16_t)STOP_LINE_CONFIRM)
    {
        return 0u;                     /* 宽度够了, 但还没连续够拍数 */
    }

    s_sl_cnt = 0u;                     /* 这次判定结清, 下一拍重新数 */

    /* --- 记录这次"压到宽线"事件(标定用, 与停不停车无关) ---
     * ★ 位置记的是【本圈第一次】触发时的里程, 不是最近一次:
     *   设里程门槛要知道的是"最早的一次出现在哪儿"(比它再靠后就安全了),
     *   最后一次的里程没有用。而且第一次通常就是真停止线的位置 ——
     *   正好可以拿它和 ENC 的总数对照, 校验"一圈到底多长"。
     * ★ s_sl_evt_cnt / s_sl_evt_w 在停车时【不清】(见 Track_Init 的说明),
     *   所以按按键停下来之后屏上照样能看到这些数。 */
    if (s_sl_evt == 0u)
    {
        s_sl_evt_cnt = Mileage_GetCount();
    }
    s_sl_evt++;
    s_sl_evt_w = best;

    /* --- 里程门槛: 只有跑到足够远, 才允许真停车 ---
     * ★ STOP_LINE_ARM_COUNT = 0 时这条【永远成立】, 也就等于没有保护 ——
     *   所以标定完一定要把它填上(见 config.h 的"使用顺序")。 */
    if (Mileage_GetCount() < (int32_t)STOP_LINE_ARM_COUNT)
    {
        return 0u;                     /* 位置太靠前, 判定为误判: 只记录不停车 */
    }

#if (STOP_LINE_ENABLE)
    StopCar(1u);                       /* 1 = 压到停止线 */
    return 1u;
#else
    return 0u;                         /* 只记录模式: 不停车 */
#endif
}

uint8_t Track_GetStopReason(void)
{
    return s_stop_reason;
}

/* ★ 2026-09-29 新增: 取"压到宽线"事件的统计, 给 OLED 显示用(标定里程门的
 *   依据)。任一指针可传 NULL。
 *     evt   = 本次启动以来累计触发次数
 *     cnt   = 最近一次触发时的里程(编码器平均计数)
 *     width = 最近一次触发时"连续几路同时压黑" */
void Track_GetWideEvent(uint16_t *evt, int32_t *cnt, uint8_t *width)
{
    if (evt   != 0) { *evt   = s_sl_evt; }
    if (cnt   != 0) { *cnt   = s_sl_evt_cnt; }
    if (width != 0) { *width = s_sl_evt_w; }
}

/* ★ 2026-09-29 新增: 本圈见过的最大连续压黑路数(= 最宽的一拍压了几路)。
 *   ★ 和 Track_GetWideEvent 的区别(别混):
 *       GetWideEvent = "够门槛的那些次"里最近一次宽几路(门槛不满足就永远 0);
 *       GetWidthMax  = 整圈里单纯最宽的一拍是多少路(和门槛无关)。
 *     判"停止线能压几路、门槛该填几"要看后者。 */
uint8_t Track_GetWidthMax(void)
{
    return s_wmax;
}

/* ★★ 2026-09-29 新增: 取"掉速档案", 给 OLED 用(回答"为什么后面的弯明显减速")。
 *   含义与判读表见 track.h。任一指针可传 NULL。 */
void Track_GetSlowStats(int16_t *bmin_ok, int16_t *bmin_all,
                        uint16_t *lmax, int32_t *bmin_dist)
{
    if (bmin_ok   != 0) { *bmin_ok   = s_bmin_ok;   }
    if (bmin_all  != 0) { *bmin_all  = s_bmin_all;  }
    if (lmax      != 0) { *lmax      = s_lmax;      }
    if (bmin_dist != 0) { *bmin_dist = s_bmin_dist; }
}

void Track_Control(int16_t line_error)
{
    int32_t diff, left_pwm, right_pwm, base, d_line, mag, pred, oil;

    if (s_track_mode == 0)
    {
        Motor_All_Stop();
        s_line_last = 0;
        s_dlp       = 0;
        s_trim_q    = 0;
        s_diff_out  = 0;
        s_lost_cnt  = 0;
        s_ramp_tick = 0;              /* 下次启动重新软启动 */
        return;
    }

    /* ---- 1) 丢线 ----
     * s_lost_dir 是"上一次看到的偏离方向": 有它就能朝那个方向满舵回找;
     * 还没有(刚启动 / 从没见过线) 就只能直行, 而且下面会把速度压到爬行。 */
    if (line_error == TRACK_ERR_LOST)
    {
        s_lost_cnt++;
        if (SIMPLE_LOST_TURN && s_lost_dir != 0)
        {
            line_error = (int16_t)(s_lost_dir * 4);   /* 满舵回找 */
        }
        else
        {
            line_error = 0;
        }
    }
    else
    {
        s_lost_cnt = 0;
        if (line_error != 0)
        {
            s_lost_dir = (line_error > 0) ? 1 : -1;
        }
        /* 死区: 宽线双路夹线时"居中"有 -1 / +1 两个位置, 靠它消掉横跳。
         * 居中(误差 0) 时不改 s_lost_dir, 万一马上丢线还知道往哪边找。 */
        if (line_error <= SIMPLE_DEAD_BAND && line_error >= -SIMPLE_DEAD_BAND)
        {
            line_error = 0;
        }
    }

    /* ---- 2) 求"误差变化速度" ----
     * 传感器只看得到车正下方, 属"事后"感知。用"误差比上一拍变了多少"来预测
     * 它接下来会偏到哪 —— 这就是提前打舵。
     * 必须先算差值, 再更新 s_line_last。 */
    d_line      = line_error - s_line_last;
    s_line_last = line_error;
    d_line      = Limit(d_line, SIMPLE_D_LIMIT);

    /* ---- 3) 虚拟前视 -> 预测误差 -> 差速 ----
     * 低通: 数字探头是"一格格跳"的, 变化速度是个瞬时尖峰, 只在那一拍(5ms)
     *       出现; 而电机+齿轮箱的机械时间常数约 30~50ms, 尖峰它根本来不及
     *       响应 —— 这就是旧版 KD 项"加了也没用"的原因。
     *       低通(时间常数 LEAD_TAU 拍)把这个尖峰摊成持续约 100ms 的指令。
     * 外推: 预测误差 = 当前误差 + 前视拍数 × 变化速度, 等效于把探头虚拟地
     *       往前挪(等效距离 ≈ 拍数 × 控制周期 × 车速), 刚进弯就压舵。
     * ★ 丢线时不外推: 那时误差是"满舵"这个人造值, 外推它没有意义; 同时把
     *   低通状态清零, 免得把丢线期间的变化速度带到重新找到线之后。
     * ★ 整数除法在 s_dlp 很小时会被截断成 0, 效果相当于给"变化速度"配了一个
     *   很小的死区: 缓慢漂移和传感器噪声不会被外推, 只有进弯那种"一格格快速
     *   变化"才触发前视。这正是我们要的 —— 直道手感因此保持"微小修正"不变。
     * 末尾 /2 是把"半格"为单位的误差折回"整格", 让 Kp 的含义仍是
     * "每偏离一整格给多少 PWM", 调参数值不用改习惯。 */
    if (s_lost_cnt == 0)
    {
        s_dlp += (((int32_t)d_line << 8) - s_dlp) / (int32_t)SIMPLE_LEAD_TAU;
        pred   = (int32_t)line_error
               + ((int32_t)SIMPLE_LEAD_TICKS * s_dlp) / 256;
    }
    else
    {
        s_dlp = 0;
        pred  = line_error;
    }

    diff = (SIMPLE_KP * pred) / 2;

    /* ---- 3b) 曲率积分: 弯道的稳态打舵量 ----
     * 原理见文件开头。要点: 弯道的曲率不在传感器信号里, 纯比例控制只能靠
     * "车偏出去"催出打舵量, 于是整条弯都贴着探头边缘跑。这里把"误差持续
     * 偏向同一侧"这个稳态偏置自己学出来, 误差就能回到 0 而差速仍维持着。
     * ★ 用 Q8 定点(1 PWM = 256)累加: 误差只有 ±1 时每拍才涨 0.15 PWM,
     *   直接对整数做除法会被截断成 0, 根本积不起来。
     * ★ 丢线时不积分: 那时误差是"满舵"这个人造值, 积它没有意义; 顺手把
     *   状态清零, 免得把丢线期间积累的量带到重新找到线之后。 */
    if (s_lost_cnt == 0)
    {
        s_trim_q += ((int32_t)line_error * SIMPLE_TRIM_KI * 256) / 1000;
        s_trim_q  = Limit(s_trim_q, (int32_t)SIMPLE_TRIM_LIMIT * 256);
        if (line_error <= 1 && line_error >= -1)
        {
            /* 已经在线上 -> 慢慢把弯道的偏置吐掉, 免得出弯后车往一边偏 */
            s_trim_q -= s_trim_q / (int32_t)SIMPLE_TRIM_TAU;
        }
    }
    else
    {
        s_trim_q = 0;
    }
    diff += (s_trim_q >> 8);

    diff = Limit(diff, SIMPLE_DIFF_MAX);

    /* ---- 4) 基础速度 ---- */
    base = SIMPLE_BASE_SPEED;

    if (s_lost_cnt == 0)
    {
        /* 弯道自动收油: 打舵越狠, 基础速度降得越多(下限 CORNER_MIN%)。
         * 转弯半径和速度是平方关系, 降一点速就能换来不少转弯能力;
         * 而且它不像加大 Kp 那样让直道画龙。丢线时不收油(正需要用力转)。
         *
         * ★★ 2026-09-29 加【绝对上限】SIMPLE_CORNER_SLOW_MAX。
         *   实车反馈"车跑到后面的转弯时明显减速" —— 根因就在这里:
         *   mag 用的是 |diff|, 而 diff 里含有【曲率积分】(s_trim_q),
         *   积分在持续弯道里会顶到上限、而且【不泄漏】(泄漏要求 |误差|<=1,
         *   弯里误差通常 >1)。于是弯道中后段 base 被叠加着压到最低:
         *       直道 32 -> 弯道中位 24 -> 最深 19(正好撞上地板 19.2)。
         *   加上限之后, 不管 diff 飙到多少, 弯里最多只降这几个 PWM:
         *       弯 base 24 -> 26, 最深 19 -> 26 (+37%), 圈时快 2.2 s。
         *   ★ 注意: 这不是在削打舵力度 —— diff 本身一个字节都没动,
         *     动的只是"顺手收多少油"。所以跟前面的 DIFF_SLEW / DIFF_MAX
         *     不是一回事, 不会影响纠偏能力。 */
        mag = (diff < 0) ? -diff : diff;
        oil = (mag * SIMPLE_CORNER_SLOW) / 100;
        if (SIMPLE_CORNER_SLOW_MAX > 0 && oil > SIMPLE_CORNER_SLOW_MAX)
        {
            oil = SIMPLE_CORNER_SLOW_MAX;   /* 封顶: 弯道最多收这么多 */
        }
        base -= oil;
        if (base < ((int32_t)SIMPLE_BASE_SPEED * SIMPLE_CORNER_MIN) / 100)
        {
            base = ((int32_t)SIMPLE_BASE_SPEED * SIMPLE_CORNER_MIN) / 100;
        }
    }
    else if (s_lost_dir == 0)
    {
        /* ★ 丢线, 而且根本不知道线在哪(启动就没看见) -> 只允许慢爬。
         *   旧版本这里用的是 BASE_SPEED, 于是"启动瞬间没压到线"就等于
         *   车全速直线冲出去, 而且越走离线越远, 永远回不来。 */
        base = SIMPLE_SEARCH_SPEED;
    }
    else
    {
        /* ★ 知道往哪边找 -> 降速。以前这里保持基础速度, 于是"满舵 + 全速"
         *   变成轮胎侧滑空转, 车照样直着滑出去 —— 这就是"丢线以后怎么都
         *   找不回来"的原因。降到 LOST_SPEED 后轮子抓得住地, 内侧轮会被
         *   差速夹推到 0, 车变成【原地小半径拧】而不是高速侧滑。 */
        base = SIMPLE_LOST_SPEED;
    }

    /* ---- 4b) ★★ 记录本圈的"掉速档案"(2026-09-29 新增) ----
     * 实车反馈"车跑到后面的转弯时明显减速", 而屏上的 B 是【本拍】的值、车一停
     * 就没了 —— 到底哪一拍掉过速、掉到多少、是收油掉下去的还是丢线掉下去的,
     * 全靠感觉描述, 定不了因。这里把一圈里最差的那几拍记下来:
     *     s_bmin_ok  : 只在【看见线】的拍上取最小 -> 只含"弯道收油"一个因素
     *     s_bmin_all : 再加上【持续丢线】的拍     -> 比上面低就说明真丢过线
     *     s_lmax     : 最长连续丢线拍数(原始值, 不去抖)
     *     s_bmin_dist: 最小值出现时的里程         -> 判断是哪一个弯
     * 判读表见 track.h 的 Track_GetSlowStats()。停车后不清, 只在按启动时归零。
     * ★ 0 当"还没采到"用: base 在运行中永远是 12 以上(CORNER_MIN 地板 19.2 /
     *   LOST_SPEED 20 / SEARCH_SPEED 12), 不会真的等于 0。
     * ★ 跳过软启动那段: 那时输出被人为压低, 不是"掉速"。
     * ★★ 只把【持续】丢线算进档案(SIMPLE_LOST_SLOW_TICKS):
     *   LOST_SPEED 是"一丢线立刻降速", 于是 3 拍(15ms)的瞬断也会把 base 压到
     *   20 —— 而 15ms 的指令在电机(机械时间常数 30~50ms)上什么都改变不了。
     *   若照原样记, 屏上就成了 `BMIN 26/20 LMAX 3`, 看着像真出故障(2026-09-29
     *   实测第一次跑新版就踩到)。丢掉比电机时间常数还短的瞬断, 档案才干净。 */
    if (s_ramp_tick >= (uint16_t)RAMP_TICKS)
    {
        if (s_lost_cnt == 0
            || s_lost_cnt >= (uint16_t)SIMPLE_LOST_SLOW_TICKS)
        {
            if (s_lost_cnt == 0
                && (s_bmin_ok == 0 || base < (int32_t)s_bmin_ok))
            {
                s_bmin_ok = (int16_t)base;
            }
            if (s_bmin_all == 0 || base < (int32_t)s_bmin_all)
            {
                s_bmin_all  = (int16_t)base;
                s_bmin_dist = Mileage_GetCount();   /* 记"在哪儿掉下去的" */
            }
        }
        if (s_lost_cnt > s_lmax)
        {
            s_lmax = s_lost_cnt;    /* 原始最大值: 瞬断也照样看得见 */
        }
    }

    /* ---- 5) 严重丢线: 连续丢了几拍还没找到, 再补一把力 ---- */
    if (s_lost_cnt >= SIMPLE_LOST_BOOST_CNT && s_lost_dir != 0)
    {
        diff += (int32_t)SIMPLE_LOST_BOOST * s_lost_dir;
        diff  = Limit(diff, SIMPLE_DIFF_MAX + SIMPLE_LOST_BOOST);
    }

    /* ---- 5.5) ★ 差速【变化率】限幅 ----
     * 2026-09-25 实车反馈: "力度是够的, 是每次左右调速时左右差速过大,
     * 偏离赛道了。" —— 那不是最大舵量的问题, 而是舵量【变化太快】:
     *   误差从 +2 跳到 -2 时, 差速一拍(5ms)之内就从 +14 变到 -14。
     *   电机机械时间常数 30~50ms 根本跟不上, 于是车拿到的是一个"阶跃"
     *   指令: 先猛地一拧把车头甩过去, 等传感器看回来已经转过头了, 再反向
     *   猛拧 —— 就这样一顿一顿地被甩出赛道。
     * 加上这个限幅之后, 差速只能"爬"上去而不能"跳"上去
     * (3 PWM/拍 → 从 0 到满舵约 55ms, 与电机时间常数同量级)。
     * ★ 关键: 它【一点也没降低上限】—— 持续偏一侧时照样能到达满舵,
     *   所以不会削弱"克服静摩擦所需的力气"。 */
#if (SIMPLE_DIFF_SLEW > 0)
    {
        int32_t slew = (int32_t)SIMPLE_DIFF_SLEW;
        int32_t step = diff - s_diff_out;
        if (step >  slew) { diff = s_diff_out + slew; }
        if (step < -slew) { diff = s_diff_out - slew; }
    }
#endif
    s_diff_out = diff;

    /* ---- 6) 合成左右轮 ----
     * err>0(黑线在右) -> 左轮快右轮慢 -> 车向右追线;
     * s_turn_sign 决定这套动作的极性, 实车反了就用三击 PA15 翻过来。 */
    left_pwm  = base + (int32_t)s_turn_sign * diff;
    right_pwm = base - (int32_t)s_turn_sign * diff;

    /* ---- 7) 输出限幅: 保住差速 ----
     * 差速就是转向能力。如果简单地把两侧各自夹到 0~100, 内侧轮一溢出到
     * 负数, 差速就被削掉了 —— 恰好在最需要转的时候转不动。
     * 做法: 哪一侧越界就把【两侧一起平移】回来, 让 (left − right) 不变。 */
    if (left_pwm  > 100) { right_pwm -= (left_pwm  - 100); left_pwm  = 100; }
    if (right_pwm > 100) { left_pwm  -= (right_pwm - 100); right_pwm = 100; }
    if (left_pwm  < 0)   { right_pwm += (0 - left_pwm);    left_pwm  = 0;   }
    if (right_pwm < 0)   { left_pwm  += (0 - right_pwm);   right_pwm = 0;   }
    /* 平移后另一侧可能又越界(说明要求的差速超出物理极限), 再夹一次兜底 */
    if (left_pwm  > 100) left_pwm  = 100;
    if (right_pwm > 100) right_pwm = 100;
    if (left_pwm  < 0)   left_pwm  = 0;
    if (right_pwm < 0)   right_pwm = 0;

    /* ---- 8) 软启动: 起步阶段按比例升起, 不是一步给到目标 ----
     * 这就是"一按启动就窜出去/轮子打滑"的对策: 输出在 RAMP_MS 内线性升起。 */
    if (RAMP_TICKS > 0u)
    {
        if (s_ramp_tick < RAMP_TICKS) s_ramp_tick++;
        if (s_ramp_tick < RAMP_TICKS)
        {
            left_pwm  = left_pwm  * (int32_t)s_ramp_tick / (int32_t)RAMP_TICKS;
            right_pwm = right_pwm * (int32_t)s_ramp_tick / (int32_t)RAMP_TICKS;
        }
    }

    /* 存一份真正下发的值, 供 OLED 显示(只读) */
    s_base_out  = (int16_t)base;
    s_left_out  = (int16_t)left_pwm;
    s_right_out = (int16_t)right_pwm;

    Motor_Left_Fwd((uint8_t)left_pwm);
    Motor_Right_Fwd((uint8_t)right_pwm);
}
