/* 
  Модуль для приема данных с ИК пульта по протоколу NEC. 
  https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html 
*/

#ifndef _ME_RMT_H_
#define _ME_RMT_H_

#include <stdbool.h>
//#include "reLed.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

void rxIR_Init(const uint8_t gpioRx, QueueHandle_t queueProc);
void rxIR_Enable();

// void rxIR_Disable();

// bool rxIR_IsAvailable();
// void rxIR_ResetAvailable();
uint32_t rxIR_GetReceivedValue();
uint16_t rxIR_GetReceivedBitLength();
uint16_t rxIR_GetReceivedDelay();
uint16_t rxIR_GetReceivedProtocol();

#ifdef __cplusplus
}
#endif

#endif // _ME_RMT_H_
