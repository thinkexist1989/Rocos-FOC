//
// Created by think on 2022/10/3.
//

#include <delay.h>
#include <main.h>
#include <stm32f1xx_hal.h>

extern TIM_HandleTypeDef htim4;
extern unsigned long timer_counter = 0;

void delay_us(unsigned long us) {
    __HAL_TIM_SetCounter(&htim4, 0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim4) < us); // wait for the counter to reach the us input in the parameter
}

unsigned long timestamp_us() {
    // 我们需要记录老的时间值，因为不保证这个函数被调用的期间TIM4事件中断不会被触发。
    static uint64_t OldTimeVal;
    uint64_t NewTimeVal;

    NewTimeVal = (timer_counter * 65536) + TIM4->CNT;

    // 当计算出来的时间值小于上一个时间值的时候，说明在函数计算的期间发生了TIM2事件中断，此时应该补正时间值。
    if (NewTimeVal < OldTimeVal) NewTimeVal += 65536;
    OldTimeVal = NewTimeVal;

    // 返回正确的时间值
    return NewTimeVal;
}