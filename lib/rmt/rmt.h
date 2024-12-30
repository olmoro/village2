/* 
  Модуль для приема данных с ИК пульта по протоколу NEC.  
*/

#ifndef _RMT_H_
#define _RMT_H_

#ifdef __cplusplus
extern "C" {
#endif

void rxIsrHandler();
void startIrRMT();

#ifdef __cplusplus
}
#endif

#endif // _RMT_H_
