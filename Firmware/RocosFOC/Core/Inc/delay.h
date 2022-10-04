//
// Created by think on 2022/10/3.
//

#ifndef __ROCOSFOC_DELAY_H
#define __ROCOSFOC_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

//! @brief Delay for ms
void delay_ms(unsigned long ms);

//! @brief Delay for us
void delay_us(unsigned long us);

//! @brief 获取微秒时间戳
unsigned long timestamp_us();

#ifdef __cplusplus
}
#endif

#endif //ROCOSFOC_DELAY_H
