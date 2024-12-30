/* 
  Модуль для приема данных с ИК пульта по протоколу NEC.  
*/

#ifndef _RMT_H_
#define _RMT_H_

#include <stdbool.h>
#include "reLed.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

void rxIsrHandler();
void startIrRMT();

void rxIR_Init(const uint8_t gpioRx, QueueHandle_t queueProc);
void rxIR_Enable();

#ifdef __cplusplus
}
#endif

#endif // _RMT_H_
