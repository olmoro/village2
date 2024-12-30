/* 
   Модуль настроек
   Управление через MQTT и Telegram
*/

#ifndef _SECURITY_H_
#define _SECURITY_H_

// #include "reRx433.h"  /* !!! reAlarm.cpp:17:10  */
// //#include "reBeep.h"   /* !!! reAlarm.cpp:20:10  */

//#include "reAlarm.h"
#include "meAlarm.h"

#ifdef __cplusplus
  extern "C" {
#endif

void alarmStart();

#ifdef __cplusplus
}
#endif

#endif // _SECURITY_H_
