//
// Created by think on 2022/10/7.
//

#ifndef ROCOSFOC_TASK_H
#define ROCOSFOC_TASK_H

#include "main.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* C CODE BEGIN */
void TaskSetup(void);//初始化函数声明，在main.c的while(1)前调用该接口
void TaskDo(void);//循环内容函数声明，在main.c的while(1)内调用该接口
/* C CODE END */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //ROCOSFOC_TASK_H
