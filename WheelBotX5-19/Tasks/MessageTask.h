#ifndef MESSAGETASK_H
#define MESSAGETASK_H
#include "main.h"
#include "UART_LL.h"
#include "stdarg.h"
#include "stdio.h"
class cMSG : public cUART
{
	protected:
	uint8_t *pbufprintf = 0;
	uint16_t pbuflen = 0;
	uint8_t PrintfLock = 1;
	public:
	uint8_t UartRecBuf[10];
	uint8_t SetPrintf(uint8_t* pbuf, uint16_t buflen)
	{
		if((pbuf&&buflen) == 0){return 1;}
		this->pbufprintf = pbuf;
		this->pbuflen = buflen;
		this->PrintfLock = 0;
		return 0;
	}
	void Printf(const char *__format,...)
	{
		if(PrintfLock){return;}
		PrintfLock = 1;
		va_list ap;
		uint16_t len;
		va_start(ap, __format);
		
		len = vsnprintf((char*)this->pbufprintf, this->pbuflen, (const char *)__format, ap);
		va_end(ap);
		
		this->Transmit(this->pbufprintf, len, 0xFFFF);
		PrintfLock = 0;
	}
	
};



extern "C" {

	void UART7_IRQHandler(void);
	void DMA1_Stream2_IRQHandler(void);
	void DMA1_Stream3_IRQHandler(void);
	
}

#endif