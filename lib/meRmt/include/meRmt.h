/* 
  Модуль для приема данных с ИК пульта по протоколу NEC. 
  https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html 
*/

#ifndef _MERMT_H_
#define _MERMT_H_

#include <stdbool.h>
//#include "reLed.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif




//void rxIsrHandler();
//void startIrRMT();

void rxIR_Init(const uint8_t gpioRx, QueueHandle_t queueProc);
void rxIR_Enable();




#ifdef __cplusplus
}
#endif

#endif // _MERMT_H_
