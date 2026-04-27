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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "oled.h"
#include <stdio.h> // 支持 sprintf 函数
#include "sr04.h"
#include "MOTOR.h"
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

float Pitch, Roll, Yaw; // 定义用来接数据的变量
volatile uint8_t dmp_ready_flag = 0; // 定义全局标志位
volatile uint8_t flag_10ms = 0; // 全局标志位
uint8_t display_buf[32]; //定义用于屏幕显示的字符串缓存数组
extern float distance; // 定义超声波检查距离的变量
int Motor_PWM = 0; // 全局变量，用于记录当前计算出的 PWM 输出，方便 OLED 显示

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Set_Motor_PWM(int moto1, int moto2); // 提前声明电机控制函数
int my_abs(int value);                    // 提前声明绝对值函数
int Vertical_PID_Control(float Current_Angle); // 提前声明 PID 函数

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    // 1. 释放 PB3 和 PB4,让它们可以作为普通 IO 模拟 IIC
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    OLED_Init();
    OLED_Clear();
    Set_Motor_Speed(0, 0);

    // 2. 测试第一步：基础 I2C 通信
    uint8_t res = MPU_Init();
    if(res == 0) {
        OLED_ShowString(0, 0, "MPU Init OK!  ", 16);
    } else {
        // 如果基础通信失败，打印出错误码
        sprintf((char *)display_buf, "MPU Fail: %d  ", res);
        OLED_ShowString(0, 0, display_buf, 16);
        while(1) HAL_Delay(100); // 卡死在这里，不往下跑了
    }
    
    HAL_Delay(500); // 稍微等一下让人看清屏幕

    // 3. 测试第二步：加载 DMP 固件
    while(mpu_dmp_init() != 0) {
        OLED_ShowString(0, 2, "DMP Error!    ", 16);
        HAL_Delay(500);
    }
    OLED_ShowString(0, 2, "DMP Success!  ", 16); 
    HAL_Delay(500);
    OLED_Clear(); // 成功后清屏，准备显示角度


    //4. 启动电机控制的 PWM 通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    Set_Motor_Speed(2000, 2000); // 这里调用你之前定义的电机控制函数，确保它能正常工作

  // 启动 TIM4 定时器中断，作为 10ms 的 PID 核心控制周期！
    HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      static uint16_t oled_cnt = 0; // 定义一个静态变量用来降频显示屏幕

      // --- 核心控制任务：严格保证每 10ms 执行一次 ---
      if (flag_10ms == 1)
      {
          flag_10ms = 0; // 进来了就马上清除标志位
          
          // 1. 读取最新姿态数据 (耗时操作，放这里完美)
          mpu_dmp_get_data(&Pitch, &Roll, &Yaw);
          
          // 2. 计算直立环 PID 输出
          Motor_PWM = Vertical_PID_Control(Roll);

          // 3. 摔倒保护 (倾角超过 40 度，认为已经摔倒，关闭电机输出)
          if (Roll > 40.0 || Roll < -40.0) 
          {
              Motor_PWM = 0;
          }

          // 4. 将计算结果立刻作用于左右电机
          Set_Motor_Speed(Motor_PWM, Motor_PWM);

          // --- 慢速任务：降频执行，防止拖慢主循环 ---
          oled_cnt++;
          if(oled_cnt >= 10) // 10ms * 10 = 100ms (一秒刷新10次屏幕，人眼看着非常顺畅且不占资源)
          {
              oled_cnt = 0; // 计数器清零
              
              // 刷新 OLED 角度和 PWM
              sprintf((char *)display_buf, "Roll:%.2f PWM:%d  ", Roll, Motor_PWM);
              OLED_ShowString(0, 2, display_buf, 16);
              
              // 触发超声波并刷新距离
              GET_Distance(); 
              sprintf((char *)display_buf, "Distance:%.2f     ", distance);
              OLED_ShowString(0, 4, display_buf, 16);
          }
      }
      
      // 注意：这里绝对不能再加 HAL_Delay(50) 了！
      // 因为上面我们已经通过 flag_10ms 和 oled_cnt 实现了精准的时间调度。
      // 加 Delay 会严重破坏 10ms 的控制周期。
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
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
}

/* USER CODE BEGIN 4 */

// --- 平衡车直立环 PID 参数 ---
float Kp = 50.0;  // 比例系数：给 0 时小车无力软绵绵；从小到大加，直到小车出现明显的高频震荡
float Kd = 0;    // 微分系数：在有震荡的基础上加 D，直到震荡消失，手感变得非常“黏”
float Target_Angle = -4; // 机械中值：你需要手动把小车扶到刚好平衡的角度，记录下此时的 Roll 值填在这里

float Last_Roll = 0.0; 

#define PWM_MAX 3500  // 暂时限制在 50% 的最高速度，防止占空比越界 (你的 ARR 是 7199)
#define PWM_MIN -3500

// 直立环 PD 控制器
int Vertical_PID_Control(float Current_Angle) 
{
    float Angle_Error;
    float Gyro_Rate;
    int PWM_Output;

    // 1. 计算角度偏差
    Angle_Error = Current_Angle - Target_Angle;

    // 2. 近似计算角速度 (注意：真正的做法是用 MPU6050 直接读出的 Y 轴陀螺仪原始数据，这里先用差值近似)
    Gyro_Rate = Current_Angle - Last_Roll;
    Last_Roll = Current_Angle;

    // 3. PD 核心公式
    PWM_Output = (Kp * Angle_Error) + (Kd * Gyro_Rate);

    // 假设死区补偿是 1000（具体数值根据你的电机实际测试修改，一般在 500~2000 之间）
    if(PWM_Output > 0) PWM_Output += 500;
    else if(PWM_Output < 0) PWM_Output -= 500;

    // 4. 输出限幅 (极为关键，防止单次突变算出一个几万的数值炸掉定时器)
    if (PWM_Output > PWM_MAX) PWM_Output = PWM_MAX;
    if (PWM_Output < PWM_MIN) PWM_Output = PWM_MIN;

    return PWM_Output;
}

// 绝对值函数
int my_abs(int value) {
    return value >= 0 ? value : -value;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 如果是 TIM4 进来的中断 (10ms 到了)
    if (htim->Instance == TIM4) 
    {
        flag_10ms = 1; // 纯置位，不要在这里面跑 MPU6050 读取代码
    }
}
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

#ifdef  USE_FULL_ASSERT
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
