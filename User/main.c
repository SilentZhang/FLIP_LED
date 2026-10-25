/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *USART Print debugging routine:
 *USART1_Tx(PA9).
 *This example demonstrates using USART1(PA9) as a print debug port output.
 *
 */

#include "debug.h"
#include "SandSim.h"
#include "charlie.h"
#include "LIS2DH.h"

/* Global typedef */

/* Global define */

#define PUSH_ITER 1
#define GRID_ITER 12
uint8_t ticks=0;
/* Global Variable */


void GPIOallPU(void){

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD |RCC_APB2Periph_AFIO, ENABLE);
GPIOA->OUTDR = 0xFFFFFFFF;
GPIOB->OUTDR = 0xFFFFFFFF;
GPIOC->OUTDR = 0xFFFFFFFF;
GPIOD->OUTDR = 0xFFFFFFFF;

GPIOA->CFGLR=0x88888888;
GPIOA->CFGHR=0x88888888;
GPIOB->CFGLR=0x88888888;
GPIOB->CFGHR=0x88888888;
GPIOC->CFGLR=0x88888888;
GPIOC->CFGHR=0x88888888;
GPIOD->CFGLR=0x88888888;
GPIOD->CFGHR=0x88888888;

}

void shutdown(void){

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
    LIS2DH_Deinit();

    // 关闭LED刷新定时器，避免其挂起中断导致STOP被立即唤醒
    TIM_Cmd(TIM1, DISABLE);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3);

    GPIOallPU();  // PA0此时为内部上拉输入，与按键(按下接GND)匹配

    // 配置PA0(按键WK，按下为下降沿)为EXTI线0中断，作为STOP唤醒源
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    EXTI_ClearITPendingBit(EXTI_Line0);

    EXTI_InitTypeDef extiCfg = {0};
    extiCfg.EXTI_Line = EXTI_Line0;
    extiCfg.EXTI_Mode = EXTI_Mode_Interrupt;
    extiCfg.EXTI_Trigger = EXTI_Trigger_Falling;
    extiCfg.EXTI_LineCmd = ENABLE;
    EXTI_Init(&extiCfg);

    NVIC_InitTypeDef nvicCfg = {0};
    nvicCfg.NVIC_IRQChannel = EXTI0_IRQn;
    nvicCfg.NVIC_IRQChannelPreemptionPriority = 1;
    nvicCfg.NVIC_IRQChannelSubPriority = 0;
    nvicCfg.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicCfg);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
    PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);
    NVIC_SystemReset();
}

// STOP模式下PA0按键下降沿唤醒中断：仅清挂起标志，唤醒后由shutdown()复位重启
void EXTI0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}


void GetAcce(uint32_t i, _iq * accex, _iq * accey, _iq * accez)
{
    int16_t x,y,z;
    LIS2DH_Get(&x,&y,&z);
    // 传感器轴 -> 仿真轴映射（仿真坐标：x向右，y向下）
    // 实测校准：两轴符号与初版推导均相反，最终映射 (xp,yp) = (+x,-y)
    float xp = (float) x * 0.3f;
    float yp = (float) y * -0.3f;
    float zp = (float) z * 0.3f;

    *accex = _IQ(xp);
    *accey = _IQ(yp);
    *accez = _IQ(zp);
}


void Show(void)
{
    
    screen_update();
    
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    GPIOallPU();
    while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == Bit_RESET);
    USART_Printf_Init(115200);

    PRINT("SystemClk:%d\r\n", SystemCoreClock);
    PRINT( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    PRINT("This is FLIP example\r\n");

    InitParticles();
    ParticleIntegrate(0, _IQ(9.8f));
    PushParticlesApart(PUSH_ITER);
    particles_to_grid();
    density_update();
    compute_grid_forces(GRID_ITER);
    grid_to_particles();

    LED_InitPeri();
    LED_Show();
    Show();

    LIS2DH_Init();

    SysTick->CTLR = 0;
    SysTick->CNT = 0;
    SysTick->CTLR = 1;
    uint32_t time = SysTick->CNT;
    _iq accex = _IQ(0);
    _iq accey = _IQ(9.8f);
    _iq accez = _IQ(0);
    for (int i=0;i<5;i++){
        GetAcce(7000,&accex,&accey,&accez);
        ParticleIntegrate(accex, accey);
        PushParticlesApart(PUSH_ITER);
        particles_to_grid();
        density_update();
        compute_grid_forces(GRID_ITER);
        grid_to_particles();
        Show();
    }

    time = SysTick->CNT - time;
    uint32_t fps = ( (5*SystemCoreClock) >> 3 )/time;
    PRINT("fps: %d \r\n",fps);



    uint32_t timer = 0;
    // 节电功能：静止超过15秒关闭LED显示，检测到姿态变化后重新点亮
    _iq last_accex = accex;
    _iq last_accey = accey;
    uint16_t stillness_frames = 0;  // 帧计数器，每帧约11ms，15秒=1364帧
    uint8_t display_off = 0;
    const _iq motion_threshold = _IQ(0.8f);   // 姿态变化判定阈值(m/s^2)
    const uint16_t stillness_timeout_frames = 1364U; // 15秒对应的帧数

    // 摇晃换容器：持续剧烈摇晃约1秒，按 矩形 -> 心形 -> 沙漏 -> 矩形 顺序切换
    uint8_t wall_mode = WALL_RECT;
    uint16_t shake_frames = 0;
    uint16_t shake_hold = 0;
    uint8_t shake_rearm = 0;
    const _iq shake_high2 = _IQ(0.74f);   // (13.8/16)^2，合加速度 > 1.4g
    const _iq shake_low2 = _IQ(0.034f);   // (2.9/16)^2，合加速度 < 0.3g（甩动/失重）
    while(1)
    {   
        NVIC_DisableIRQ(TIM1_CC_IRQn);
        NVIC_DisableIRQ(TIM1_UP_IRQn);
        GetAcce(7000,&accex,&accey,&accez);

        // 剧烈摇晃判定：合加速度模长明显偏离1g（各轴先缩1/16再平方，避免Q24溢出）
        _iq axs = _IQmpy(accex, _IQ(0.0625f));
        _iq ays = _IQmpy(accey, _IQ(0.0625f));
        _iq azs = _IQmpy(accez, _IQ(0.0625f));
        _iq mag2 = _IQmpy(axs, axs) + _IQmpy(ays, ays) + _IQmpy(azs, azs);
        uint8_t impulse = (mag2 > shake_high2 || mag2 < shake_low2) ? 1U : 0U;

        // 单次冲击后保持约0.2秒的“摇晃中”状态，避免帧间隔错过峰值导致计分被扣光
        if (impulse) {
            shake_hold = 18U;
        } else if (shake_hold > 0U) {
            shake_hold--;
        }
        uint8_t shaking = (shake_hold > 0U) ? 1U : 0U;

        // 节电：检测姿态是否变化
        _iq dx = accex - last_accex;
        _iq dy = accey - last_accey;
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;
        if (dx > motion_threshold || dy > motion_threshold) {
            stillness_frames = 0;
            if (display_off) {
                LED_Show();
                display_off = 0;
            }
        }
        last_accex = accex;
        last_accey = accey;

        // 摇晃计时：持续剧烈摇晃累计约1秒触发切换；平静时缓慢回落
        if (shake_rearm > 0U) {
            shake_frames = 0;
            if (!shaking) {
                shake_rearm--;   // 切换后需平静约0.3秒，避免连续翻转
            }
        } else if (shaking) {
            shake_frames += 1U;
            if (shake_frames >= 80U) {
                wall_mode++;
                if (wall_mode > WALL_HOURGLASS) {
                    wall_mode = WALL_RECT;
                }
                SandSim_SetWallMode(wall_mode);
                shake_frames = 0;
                shake_rearm = 30U;
                if (display_off) {
                    LED_Show();
                    display_off = 0;
                }
            }
        } else if (shake_frames > 0U) {
            shake_frames--;
        }

        // 静止超过15秒则关闭显示
        if (!display_off) {
            if (stillness_frames < stillness_timeout_frames) {
                stillness_frames++;
            } else {
                LED_Stop();
                display_off = 1;
            }
        }

        // 显示关闭时跳过仿真以省电
        if (!display_off) {
            ParticleIntegrate(accex, accey);
            PushParticlesApart(PUSH_ITER);
            particles_to_grid();
            density_update();
            compute_grid_forces(GRID_ITER);
            grid_to_particles();
            Show();
        }
        NVIC_EnableIRQ(TIM1_CC_IRQn);
        NVIC_EnableIRQ(TIM1_UP_IRQn);
        while(timer ++ < 330)
        {__WFI();}
        timer = 0;
        if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == Bit_RESET){
            Delay_Ms(30);
            if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == Bit_RESET){
                while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == Bit_RESET);
                Delay_Ms(30);
                shutdown();
            }
        }
        //Delay_Us(500);

    }
}
