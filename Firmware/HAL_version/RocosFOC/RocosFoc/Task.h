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
void TaskSetup(void);//��ʼ��������������main.c��while(1)ǰ���øýӿ�
void TaskDo(void);//ѭ�����ݺ�����������main.c��while(1)�ڵ��øýӿ�
/* C CODE END */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //ROCOSFOC_TASK_H
