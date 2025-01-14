/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html
 * 
 * https://dzen.ru/a/Y1a95dnUa1r1hl7I
 * Обработчик прерывания: https://easyelectronics.ru/freertos_manual.html
 * 
 * https://github.com/espressif/esp-idf/tree/master
 * https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/ir_nec_transceiver
 * Заменен на
 * https://github.com/huseyin-yildiz/ESP32-Idf-NEC-IR-Receiver
 */

#include "meRmt2.h"
#include "project_config.h"
#include "common_config.h"
#include "def_tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
//#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "mTypes.h"

// #include <stdint.h>
// #include <stdio.h>
// #include "time.h"
// #include "esp_err.h"
// #include "esp_timer.h"
#include <driver/gpio.h>
#include "rLog.h"
//#include "rTypes.h"

static const char *TAG_RECEIVER = "IR-RECEIVER";
// #define ERR_CHECK(err, str) if (err != ESP_OK) ESP_LOGE(TAG_RECEIVER, "%s: #%d %s", str, err, esp_err_to_name(err));
// #define ERR_GPIO_SET_MODE "Failed to set GPIO mode"
// #define ERR_GPIO_SET_ISR  "Failed to set ISR handler"

static gpio_num_t _gpioRx = GPIO_NUM_MAX;

static volatile uint32_t _receivedValue = 0;
static volatile uint16_t _receivedBitlength = 0;
static volatile uint16_t _receivedDelay = 0;
static volatile uint16_t _receivedProtocol = 0;

// rmt_channel_handle_t rx_channel = NULL;
// rmt_receive_config_t receive_config;
// rmt_symbol_word_t raw_symbols[64]; /* 64 symbols should be sufficient for a standard NEC frame */
// QueueHandle_t receive_queue;



/* Dimensions the buffer that the task being created will use as its stack.
  NOTE:  This is the number of bytes the stack will hold, not the number of
  words as found in vanilla FreeRTOS. */
#define STACK_SIZE 2048

// Structure that will hold the TCB of the task being created.
StaticTask_t xTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
  an array of StackType_t variables.  The size of StackType_t is dependent on
  the RTOS port. */
StackType_t xStack[ STACK_SIZE ];




extern "C" { 
/**
 * @brief Saving NEC decode results
 */
static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols)
{
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code) {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            address |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            address &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            command |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            command &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    // save address and command
    s_nec_code_address = address;
    s_nec_code_command = command;
    return true;
}

// // /**
// //  * @brief Check whether the RMT symbols represent NEC repeat code
// //  */
// // static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
// // {
// //     return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
// //            nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
// // }

/**
 * @brief Decode RMT symbols into NEC scan code and print the result
 */
static void parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
    // printf("NEC frame start---\r\n");
    // for (size_t i = 0; i < symbol_num; i++) {
    //     printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
    //            rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    // }
    // printf("---NEC frame end: ");

    /* decode RMT symbols */
    // switch (symbol_num) {
    // case 34: /* NEC normal frame */
    //     // if (nec_parse_frame(rmt_nec_symbols)) {
    //     //     printf("Address=%04X, Command=%04X\r\n", s_nec_code_address, s_nec_code_command);
    //     // }
    //   nec_parse_frame(rmt_nec_symbols);
    //   break;
    // case 2: /* NEC repeat frame */
    //     // if (nec_parse_frame_repeat(rmt_nec_symbols)) {
    //     //     printf("Address=%04X, Command=%04X, repeat\r\n", s_nec_code_address, s_nec_code_command);
    //     // }
    //   nec_parse_frame_repeat(rmt_nec_symbols);
    //   break;
    // default:
    //   printf("Unknown NEC frame\r\n");
    //   break;
    // }
  if (symbol_num == 34 || symbol_num == 2) nec_parse_frame(rmt_nec_symbols);
  //else printf("Unknown NEC frame\r\n");
  else ESP_DRAM_LOGE(TAG_RECEIVER, "Unknown NEC frame\r\n");
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

} // extern "C"



// /* ------------------------------------------------------------------------
// *                          ISR handler
// * - Обработчик прерывания должен постоянно находится в быстрой памяти IRAM, 
// * поэтому его следует пометить соответствующим атрибутом IRAM_ATTR.
// * - В обработчиках прерываний не допускается использование библиотечных 
// * функций ESP_LOGx, но можно использовать специальную облегченную версию 
// * ESP_DRAM_LOGx.
// * - Обработчик прерываний выполняется вне контекста прикладных задач. 
// * Для соблюдать потокобезопасности из обработчика прерываний лучше всего 
// * отправить какие-либо данные в очередь другой задачи.
// *
// *
// * - После завершения прерывания можно выполняется переключение контекста 
//   путем вызова portYIELD_FROM_ISR.
// // ------------------------------------------------------------------------*/
// static void IRAM_ATTR rxIsrHandler(void* arg)
// {
//   static rmt_rx_done_event_data_t rx_data;



//   /* Перед вызовом xQueueReceiveFromISR() должен принудительно устанавливаться в pdFALSE */
//   BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//   // while (1)
//   // {
//     /* Ожидание входного сигнала приёмника. Возвращает pdTRUE если элемент был успешно 
//        получен из очереди, в противном случае значение pdFALSE */
//     if (xQueueReceiveFromISR(receive_queue, &rx_data, &xHigherPriorityTaskWoken) == pdPASS )
//     {
//       /* Преобразование символов в новый скан-код и распечатка результата */
//       parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
//       ESP_DRAM_LOGI(TAG_RECEIVER, "Address=%04X, Command=%04X\r\n", s_nec_code_address, s_nec_code_command);

//       _receivedValue = (s_nec_code_address << 16) | s_nec_code_command;

//       /* Завершено успешно, отправка данных во внешнюю очередь */
//       QueueHandle_t queueProc = (QueueHandle_t)arg;
//       if (queueProc) 
//       {
//         input_data_t data;
//         data.source = IDS_RXIR;      /* Идентификатор источника */
//         data.rxIR.protocol = 0;
//         data.rxIR.value = _receivedValue;
//     //data.count = 34;  //rx_data.num_symbols;
//         ESP_DRAM_LOGI(TAG_RECEIVER,"Value=%" PRIX32 "\n", data.rxIR.value);
//           //    vRingbufferReturnItem(rb, (void *) items);    // Очистка буфера

              

//         // we have not woken a task at the start of the ISR
//         //BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//         /* Публиковать данные */
//         xQueueSendFromISR(queueProc, &data, &xHigherPriorityTaskWoken);
//         // now the buffer is empty we can switch context if necessary.
//         if (xHigherPriorityTaskWoken) 
//         {
//           portYIELD_FROM_ISR();
//         };
//       }
//       else
//       {
//         /* reset recieved value */
//         rxIR_ResetAvailable();
//       }
//       // break;
//       /* start receive again */
//       ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
//     }
//   // }
// } 


/* Function that implements the task being created. */
vTaskCode( void* pvParameters )
{
  /* The parameter value is expected to be 1 as 1 is passed in the
     pvParameters value in the call to xTaskCreateStatic(). */
  configASSERT( ( uint32_t ) pvParameters == 1UL );

  ESP_LOGI(TAG_RECEIVER, "create RMT RX channel");
  rmt_rx_channel_config_t rx_channel_cfg = 
  {
    .gpio_num = static_cast<gpio_num_t>(IR_RX_GPIO_NUM),
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = IR_RESOLUTION_HZ,
    .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
  };
  rmt_channel_handle_t rx_channel = NULL;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

  ESP_LOGI(TAG_RECEIVER, "register RX done callback");
  QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  assert(receive_queue);
  rmt_rx_event_callbacks_t cbs = 
  {
    .on_recv_done = rmt_rx_done_callback,
  };
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

  // the following timing requirement is based on NEC protocol
  rmt_receive_config_t receive_config = 
  {
    .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
    .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
  };

  ESP_LOGI(TAG_RECEIVER, "enable RMT RX channel");
  // ESP_ERROR_CHECK(rmt_enable(tx_channel));
  ESP_ERROR_CHECK(rmt_enable(rx_channel));

  // save the received RMT symbols
  rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
  rmt_rx_done_event_data_t rx_data;
  // ready to receive
  ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));

  for( ;; )
  {
        // wait for RX done signal
    if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) 
    {
      // parse the receive symbols and print the result
      parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);

      //printf("Address=%04X, Command=%04X\r\n", s_nec_code_address, s_nec_code_command);
    //  rlog_i(TAG_RECEIVER, "Address=%04X, Command=%04X\r\n", s_nec_code_address, s_nec_code_command);

      _receivedValue = (s_nec_code_address << 16) | s_nec_code_command;

      /* Завершено успешно, отправка данных во внешнюю очередь */
  //    QueueHandle_t queueProc = (QueueHandle_t)pvParameters;    //arg;
      if (1)  //queueProc) 
      {
        input_data_t data;
        data.source = IDS_RXIR;      /* Идентификатор источника */
        data.rxIR.protocol = 0;
        data.rxIR.value = _receivedValue;
    //data.count = 34;  //rx_data.num_symbols;
        rlog_i(TAG_RECEIVER,"Value=%" PRIX32 "\n", data.rxIR.value);
          //    vRingbufferReturnItem(rb, (void *) items);    // Очистка буфера
        /* Публиковать данные */
  //      xQueueSend(queueProc, &data, portMAX_DELAY);
      }
      // start receive again
      ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    }

  }
  vTaskDelete(NULL);
}



void rxIR_Init(const uint8_t gpioRx, QueueHandle_t queueProc)
{
//  /* cpp - преобразование указателя (базовый класс) в указатель на производный класс */
//  _gpioRx = static_cast<gpio_num_t>(gpioRx);

//   /* Init receiver here */
//   ESP_LOGI(TAG_RECEIVER, "create RMT RX channel");
//   rmt_rx_channel_config_t rx_channel_cfg = 
//   {
//     .gpio_num = _gpioRx,
//     .clk_src = RMT_CLK_SRC_DEFAULT,
//     .resolution_hz = IR_RESOLUTION_HZ,
//     .mem_block_symbols = 64, /* amount of RMT symbols that the channel can store at a time */
//   };
// //  rmt_channel_handle_t rx_channel = NULL;
//   ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

//   ESP_LOGI(TAG_RECEIVER, "register RX done callback");
//   //QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
//   receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
//   assert(receive_queue);
//   rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_done_callback };
//   ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

//   /* the following timing requirement is based on NEC protocol */
// //  rmt_receive_config_t receive_config = 
//   receive_config = 
//   {
//     .signal_range_min_ns = 1250,     /* the shortest duration for NEC signal is 
//                       560us, 1250ns < 560us, valid signal won't be treated as noise */
//     .signal_range_max_ns = 12000000, /* the longest duration for NEC signal is 
//                       9000us, 12000000ns > 9000us, the receive won't stop early */
//   };

//   ESP_LOGI(TAG_RECEIVER, "enable RMT RX channel");
//   // ESP_ERROR_CHECK(rmt_enable(tx_channel));
//   ESP_ERROR_CHECK(rmt_enable(rx_channel));

//   /* save the received RMT symbols */
// //  rmt_symbol_word_t raw_symbols[64]; /* 64 symbols should be sufficient for a standard NEC frame */
//   //rmt_rx_done_event_data_t rx_data;
//   /* ready to receive */
//   ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));




//   rlog_i(TAG_RECEIVER, "Initialization of IR receiver on gpio #%d", _gpioRx);
//   gpio_reset_pin(_gpioRx);

//   ERR_CHECK(gpio_set_direction(_gpioRx, GPIO_MODE_INPUT), ERR_GPIO_SET_MODE);
//   ERR_CHECK(gpio_set_pull_mode(_gpioRx, GPIO_FLOATING), ERR_GPIO_SET_MODE);
//   ERR_CHECK(gpio_set_intr_type(_gpioRx, GPIO_INTR_ANYEDGE), ERR_GPIO_SET_ISR);
//   ERR_CHECK(gpio_isr_handler_add(_gpioRx, rxIsrHandler, queueProc), ERR_GPIO_SET_ISR);

//   esp_err_t err = gpio_intr_enable(_gpioRx);
//   if (err == ESP_OK) {
//     rlog_i(TAG_RECEIVER, "Receiver IR started");
//   } else {
//     rlog_e(TAG_RECEIVER, "Failed to start IR receiver");
//   };


//   // Для первого включения
//  xTaskCreate(rxIsrHandler, "Pult_task", CONFIG_RX_TASK_STACK_SIZE, NULL, CONFIG_TASK_PRIORITY_IR_RX, NULL);

  TaskHandle_t xHandle = NULL;

  /* Create the task without using any dynamic memory allocation. */
  xHandle = xTaskCreateStatic(
                vTaskCode,            // Function that implements the task.
                "ir_receiver",        // Text name for the task.
                STACK_SIZE,           // Stack size in bytes, not words.
                ( void * ) 1,       // Parameter passed into the task.
                //queueProc,       // Parameter passed into the task.
                tskIDLE_PRIORITY,     // Priority at which the task is created.
                xStack,               // Array to use as the task's stack.
                &xTaskBuffer );       // Variable to hold the task's data structure.

  /* puxStackBuffer and pxTaskBuffer were not NULL, so the task will have
    been created, and xHandle will be the task's handle.  Use the handle
    to suspend the task. */
//  vTaskSuspend( xHandle );

}




/* Function that creates a task. */
void rxIR_Enable()
{

}


// void rxIR_Disable()
// {
//   esp_err_t err = gpio_intr_disable(_gpioRx);
//   if (err == ESP_OK) {
//     rlog_i(TAG_RECEIVER, "Receiver IR stopped");
//   } else {
//     rlog_e(TAG_RECEIVER, "Failed to stop IR receiver");
//   };
// }

// // ------------------------------------------------------------------------
// //                          Public functions 
// // ------------------------------------------------------------------------

// bool rxIR_IsAvailable()
// {
//   return _receivedValue != 0;
// }

// void rxIR_ResetAvailable()
// {
//   _receivedValue = 0;
// }

// uint32_t rxIR_GetReceivedValue()
// {
//   return _receivedValue;
// }

// uint16_t rxIR_GetReceivedBitLength()
// {
//   return _receivedBitlength;
// }

// uint16_t rxIR_GetReceivedDelay()
// {
//   return _receivedDelay;
// }

// uint16_t rxIR_GetReceivedProtocol()
// {
//   return _receivedProtocol;
// }






