/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "track.h"
#include "key.h"
#include "oled.h"
#include "stopwatch.h"
#include "mileage.h"
#include "signal.h"
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 控制节拍: 由 SysTick(1ms) 分频得到, 周期 = config.h 里的 CTRL_PERIOD_MS。
 * 传感器采样 + 循迹控制都放在这一拍里, 保证时间基准一致。 */
void CtrlLoop(void)
{
    uint8_t mask;
    int16_t err;

    /* 电机自检(长按 PA15)期间, 控制环必须让路, 否则它会每拍把电机停掉 */
    if (g_motor_busy) return;

    mask = Track_GetMask();

    /* ★ 里程 / A·B 点(2026-09-29 新增)。必须放在最前面:
     *   ① 它每拍都要刷新里程(用编码器);
     *   ② 下面停止线的"里程门槛"要读它刚算好的值;
     *   ③ 跨过 B 点/A 点时它会给一次声光提示(只记事件, 实际发声由主循环的
     *      Signal_Task 做 —— 这里是中断上下文, 不能在里面做有时序的动作)。 */
    Mileage_Task();

    /* ★ 停止线检测(2026-09-29 新增)。放在控制计算【之前】:
     *   一旦判定压到停止线, Track_StopDetect 内部已经断油停车,
     *   这里必须直接结束本拍 —— 若继续往下算控制量, 会把刚停住的
     *   电机又推起来, 车就"停不下来"了。
     *   ★ 判据是【里程门槛 + 连续几路压黑 + 连续几拍】三者同时成立,
     *     详见 config.h 的 STOP_LINE_* 一组宏(以及"为什么三路会误判")。 */
    if (Track_StopDetect(mask) != 0u)
    {
        return;
    }

    err  = Track_GetLineError(mask);
    Track_Control(err);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();      /* 编码器只初始化/复位, 不参与电机控制(H 题里程/停车精度要用)。
                        * ★★ 累计计数靠 Mileage_Task() 里的 Encoder_Update() 每拍推进 ——
                        *   它挂在 CtrlLoop 上, 漏了它 ENC 会一直是 0。见 encoder.h 开头。 */
  Track_Init();
  Track_LoadSettings();/* 读回上次用按键标定并保存的极性; 没存过就用 config.h 默认值 */

  /* 跑圈计时(2026-09-29 加)。不用按键 —— 它自动跟着循迹开关走:
   * 短按启动表归零开跑, 短按停止就冻结成这一圈的成绩。 */
  Stopwatch_Init();

  /* ★ 声光提示(2026-09-29 加)。放在这里而不放在后面是有意的:
   *   蜂鸣器是【高电平响】(2026-09-29 实测确认), 而 CubeMX 生成的 gpio.c
   *   把 PB0 初置成 SET(高) —— 那一瞬间它会响一下。Signal_Init() 立刻把它
   *   置回"不响"(低), 所以必须尽早调用。 */
  Signal_Init();

  /* ★ 里程 / A·B 点(2026-09-29 加)。标定值(MILEAGE_LAP_COUNT)没填之前
   *   它什么都不做 —— 这是有意的, 免得拿一个假数去触发提示。 */
  Mileage_Init();

  /* OLED (1.3 寸 SH1106, SPI2)。本工程没有串口, 这块屏就是调试窗口 ——
   * 它会显示误差 / 基础速度 / 差速 / 左右 PWM / 五路探头 / 编码器计数。
   * 屏幕能正常点亮即说明 SPI2 + 那三根控制线都接对了。 */
  Oled_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t key = Key_Scan();

    if (key == KEY_LONG)
    {
      /* 长按 PA15: 电机自检(左轮转1s -> 右轮转1s), 用来确认接线/左右/转向。
       * 先强制停车, 免得自检和循迹环抢电机。请把轮子架空再做。 */
      Track_SetEnabled(0);
      Motor_All_Stop();
      Motor_SelfTest();
    }
    else if (key == KEY_DOUBLE)
    {
      /* 双击 PA15: 切换传感器极性(黑线对应低电平 / 高电平)。
       * 做成可切换是因为本工程没串口, 极性搞反了只能上车试,
       * 这样不用重新编译下载就能很快试出来。
       * ★ 切换结果会立刻写进 Flash —— 断电/复位/重新烧录都不会丢。
       * 反馈: 两个轮子一起短促抖两下 = 已切换。然后短按启动看车是不是在追黑线。 */
      Track_SetEnabled(0);
      Motor_All_Stop();
      Track_ToggleActiveLevel();
      Motor_Ack();
    }
    else if (key == KEY_TRIPLE)
    {
      /* 三击 PA15: 切换【转向极性】(修正方向反过来)。
       * 用于"巡线时左右轮修正方向反了": 黑线偏左车却往右拐, 越拐越远。
       * 成因是传感器排针左右顺序接反 或 电机左右接线对调, 现象一样,
       * 翻一次这个开关即可, 不用改线。
       * ★ 切换结果会立刻写进 Flash —— 以前只存在内存里, 重新烧录/断电就
       *   退回默认值, 于是"好端端的车突然朝赛道对角直冲"; 现在不会了。
       * 反馈: 左轮抖一下 -> 右轮抖一下(和双击的"两轮同时抖两下"区分开)。
       * 切完短按启动, 把黑线放最左探头下, 看车往哪边转。
       * (本车实车标定的正确值是 (-1), 也就是 config.h 里的默认值 ——
       *  正常情况下不用动它。只有换了车/重接了线才需要在这里切。) */
      Track_SetEnabled(0);
      Motor_All_Stop();
      Track_ToggleTurnSign();
      Motor_AckTurn();
    }
    else if (key == KEY_SHORT)
    {
      /* 单击 PA15: 启动 / 停止 循迹 */
      if (Track_GetEnabled() == 0)
      {
        /* ★ 启动前先确认探头能看到黑线。
         *   看不到就别动 —— 否则车会在完全不知道线在哪的情况下按基础
         *   速度冲出去(旧版本正是这样一路冲到赛道对角去的, 因为此时
         *   "偏离方向"还是未知, 控制量算出来是 0 = 直行且全速)。
         *   看不到时的反馈: 两个轮子抖两下。遇到这种情况:
         *     ① 看车是不是真的压在黑线上;  ② 试【双击 PA15】切一次极性。 */
        if (Track_StartOk())
        {
          Encoder_Reset();
          Track_SetEnabled(1);
        }
        else
        {
          Motor_Ack();
        }
      }
      else
      {
        Track_SetEnabled(0);
        Motor_All_Stop();
      }
    }

    /* 跑圈计时。必须在 Oled_Task() 之前 —— 屏上显示的就是它刚更新的值。
     * 它同时负责检测"循迹 0->1 / 1->0"这两个边沿(表归零 / 表冻结),
     * 所以这里每圈都要调。 */
    Stopwatch_Task();

    /* ★ 声光提示(2026-09-29 加)。控制中断里只记"到过 B 点/A 点"的事件,
     * 真正让蜂鸣器响、让灯闪是在这里做的 —— 响/闪都要持续几十上百毫秒,
     * 只能在主循环用非阻塞的状态机推进。
     * ⚠ 必须在 Oled_Task() 之前: 它很快(几十微秒), 而刷屏要阻塞约 1.8 ms,
     *   放后面会让提示音的时序被拉长。 */
    Signal_Task();

    /* 刷 OLED。内部自己按 OLED_REFRESH_MS(默认 100ms) 限速,
     * 所以这里每圈都调也没关系。 */
    Oled_Task();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  /* ⚠⚠ 2026-09-28 重要: 这里必须是 RCC_HSE_ON, 不能是 RCC_HSE_OFF!!
   * 本板用 8 MHz 外部晶振, PLL 源也选的是 HSE(PLLSource = HSE)。
   * 如果 HSEState 写成 OFF, PLL 永远锁不上 -> HAL_RCC_OscConfig / ClockConfig
   * 直接失败 -> 掉进 Error_Handler 死循环, 开发板表现就是"上电什么反应都没有"。
   * (这个坑 2026-09-25 踩过一次, 原因是 .ioc 里的 PD0-OSC_IN 引脚被误删,
   *  CubeMX 于是以为 HSE 没启用, 重新生成代码就把它写成 OFF 了。
   *  .ioc 已修正, 但每次重新生成后都建议扫一眼这里。) */
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
