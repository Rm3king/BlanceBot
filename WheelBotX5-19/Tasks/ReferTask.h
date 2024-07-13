#ifndef REFERTASK_H
#define REFERTASK_H
#include <main.h>
#ifdef __cplusplus
extern "C" {
	void UART5_IRQHandler(void);
	void DMA2_Stream0_IRQHandler(void);
	void DMA1_Stream7_IRQHandler(void);
	void DMA2_Stream1_IRQHandler(void);
}
#endif
#endif