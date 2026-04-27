#include "MOTOR.h"
#include "main.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;

// 定义最大速度限制，对应你的 ARR 值
#define MAX_SPEED 7199

/**
 * @brief  限制绝对值大小的辅助函数
 */
int Limit_Speed(int speed)
{
    if (speed > MAX_SPEED) return MAX_SPEED;
    if (speed < -MAX_SPEED) return -MAX_SPEED;
    return speed;
}

/**
 * @brief  设置左右电机的速度和方向 (带限幅保护)
 * @param  left_speed: 左电机速度 (正数前进，负数后退)
 * @param  right_speed: 右电机速度 (正数前进，负数后退)
 */
void Set_Motor_Speed(int left_speed, int right_speed)
{
    // 1. 先进行限幅，防止 PID 输出炸掉定时器
    left_speed = Limit_Speed(left_speed);
    right_speed = Limit_Speed(right_speed);

    /* ---------------- 2. 处理左侧电机 (电机2) ---------------- */
    if (left_speed >= 0) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // BIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // BIN2
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_speed); // 正数直接填入
    } 
    else 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // BIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // BIN2
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -left_speed); // 负数取反变成正数填入
    }

    /* ---------------- 3. 处理右侧电机 (电机1) ---------------- */
    if (right_speed >= 0) 
    {
         // 物理反向安装，方向引脚和左边相反
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // AIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // AIN2
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, right_speed); 
    } 
    else 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // AIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // AIN2
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -right_speed); 
    }
}