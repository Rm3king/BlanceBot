#ifndef INITALTASK_H
#define INITALTASK_H
#include "main.h"

extern float BatVal;

#ifdef __cplusplus
extern "C" {
void Task_Init(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void UART7_IRQHandler(void);
}
#endif
#endif