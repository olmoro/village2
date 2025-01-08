/*
   ------------------------------------------------------------------------
   Файл конфигурации проекта, доступен из всех файлов проекта, в том числе 
   и библиотек
   ------------------------------------------------------------------------
*/

#pragma once

#include <stdint.h>
#include "esp_task.h"
#include "common_config.h"  // Общие параметры

// ------------------------------------------------------------------------
//                              Версии 
// ------------------------------------------------------------------------
#define APP_VERSION "VILLAGE2-20241230.11"
// 20250108.11: ИК приемник - новая версия RX без ir_nec_encoder
// 20241230.10: ИК приемник - новая версия 

// ------------------------------------------------------------------------
//                             TARGET
// ------------------------------------------------------------------------

#define EXAMPLE_RMT_RX

/* BOILER - прототип выбирается по умолчанию */
//#define FREEZER   // Вариант автомата морозильника
//#define PUMPER    // Вариант автомата подкачки
//#define SOLAR     // Вариант солнечного бойлера
//#define ROTATION  // Вариант устройства ротации кондиционеров


#if PUMPER
  // Светодиоды
  #define CONFIG_GPIO_SYSTEM_LED      CONFIG_GPIO_LED_BLUE  // Системный Синий
  #define CONFIG_GPIO_MODE_LED        CONFIG_GPIO_LED_GREEN // зел
  #define CONFIG_GPIO_ALARM_FLASH     CONFIG_GPIO_LED_RED   // Красный мигающий сигнал аварии

  // Датчики
  #define CONFIG_GPIO_TEMP_OUTDOOR    CONFIG_GPIO_TEMP_X3    // Датчик температуры внешний

 // Выходы
  #define CONFIG_GPIO_RELAY_BOILER    CONFIG_GPIO_RELAY_K2  // Выход на реле насоса
  #define CONFIG_GPIO_RELAY_ALARM1    CONFIG_GPIO_RELAY_K4  // Выход на реле аварии (сирена)
  #define CONFIG_GPIO_RELAY_ALARM2    CONFIG_GPIO_RELAY_K3  // Выход на реле аварии (маяк)

  // Зоны сигнализации
  #define CONFIG_GPIO_ALARM_ZONE_1    CONFIG_GPIO_X4        // Вход контроля уровня (резерв)
  #define CONFIG_GPIO_ALARM_ZONE_2    CONFIG_GPIO_X5        // Вход шлейфа охраны (резерв)
  //#define CONFIG_GPIO_ALARM_ZONE_3    CONFIG_GPIO_POWER_X6   // Вход контроля напряжения на реле 
  #define CONFIG_GPIO_ALARM_ZONE_4    CONFIG_GPIO_POWER_X7   // Вход контроля напряжения насоса
  #define CONFIG_GPIO_ALARM_ZONE_5    CONFIG_GPIO_RX433        // резерв
  #define CONFIG_GPIO_ALARM_LEVEL     0x01                  // уточнить

#elif SOLAR
  // Светодиоды
  #define CONFIG_GPIO_SYSTEM_LED      CONFIG_GPIO_LED_BLUE  // Системный Синий
  #define CONFIG_GPIO_MODE_LED        CONFIG_GPIO_LED_GREEN // зел
  #define CONFIG_GPIO_ALARM_FLASH     CONFIG_GPIO_LED_RED   // Красный мигающий сигнал аварии
 
   // Датчики
  #define CONFIG_GPIO_TEMP_OUTDOOR    CONFIG_GPIO_TEMP_X3    // Датчик температуры в коллекторе
  #define CONFIG_GPIO_TEMP_BOILER     CONFIG_GPIO_TEMP_X1    // Датчик температуры в накопителе

  // Выходы
  #define CONFIG_GPIO_RELAY_BOILER    CONFIG_GPIO_RELAY_K2  // Выход на реле насоса
  #define CONFIG_GPIO_RELAY_ALARM1    CONFIG_GPIO_RELAY_K4  // Выход на реле аварии (сирена)
  #define CONFIG_GPIO_RELAY_ALARM2    CONFIG_GPIO_RELAY_K3  // Выход на реле аварии (маяк)
  
  // Зоны сигнализации
  #define CONFIG_GPIO_ALARM_ZONE_1    CONFIG_GPIO_X4        // Вход шлейфа охраны (резерв)
  #define CONFIG_GPIO_ALARM_ZONE_2    CONFIG_GPIO_X5        // Вход шлейфа охраны (резерв)
  #define CONFIG_GPIO_ALARM_ZONE_3    CONFIG_GPIO_POWER_X7   // Вход контроля напряжения 
  #define CONFIG_GPIO_ALARM_ZONE_4    CONFIG_GPIO_IR        // Вход ИК датчика (резерв)
  #define CONFIG_GPIO_ALARM_LEVEL     0x01                  // уточнить

#elif ROTATION
  // Светодиоды
  #define CONFIG_GPIO_SYSTEM_LED      CONFIG_GPIO_LED_BLUE  // Системный Синий
  #define CONFIG_GPIO_MODE_LED        CONFIG_GPIO_LED_GREEN // зел
  #define CONFIG_GPIO_ALARM_FLASH     CONFIG_GPIO_LED_RED   // Красный мигающий сигнал аварии

  // Датчики
  #define CONFIG_GPIO_TEMP_BOILER     CONFIG_GPIO_TEMP_X1    // Датчик температуры первого блока
  #define CONFIG_GPIO_TEMP_INDOOR     CONFIG_GPIO_TEMP_X2    // Датчик температуры второго блока
  #define CONFIG_GPIO_TEMP_OUTDOOR    CONFIG_GPIO_TEMP_X3    // Датчик температуры помещения
  
  // Выходы
  #define CONFIG_GPIO_RELAY_BOILER1    CONFIG_GPIO_RELAY_K1  // Выход на силовое реле первого блока
  #define CONFIG_GPIO_RELAY_BOILER2    CONFIG_GPIO_RELAY_K2  // Выход на силовое реле второго блока
  #define CONFIG_GPIO_RELAY_ALARM1     CONFIG_GPIO_RELAY_K4  // Выход на реле аварии первого блока
  #define CONFIG_GPIO_RELAY_ALARM2     CONFIG_GPIO_RELAY_K3  // Выход на реле аварии второго блока

  // Зоны сигнализации
  #define CONFIG_GPIO_ALARM_ZONE_1    CONFIG_GPIO_X4        // Вход термостата (резерв)
  #define CONFIG_GPIO_ALARM_ZONE_2    CONFIG_GPIO_X5        // Вход шлейфа охраны (резерв)
  #define CONFIG_GPIO_ALARM_ZONE_3    CONFIG_GPIO_POWER_X6   // Вход контроля напряжения на реле первого блока
  #define CONFIG_GPIO_ALARM_ZONE_4    CONFIG_GPIO_POWER_X7   // Вход контроля напряжения на реле второго блока
  #define CONFIG_GPIO_ALARM_ZONE_5    CONFIG_GPIO_RX433        // резерв
  #define CONFIG_GPIO_ALARM_LEVEL     0x01                  // уточнить

#else // BOILER (Прототип К12)
  // Светодиоды
  #define CONFIG_GPIO_SYSTEM_LED    CONFIG_GPIO_LED_BLUE  // Системный Синий
  #define CONFIG_GPIO_EXEC_LED      CONFIG_GPIO_LED_GREEN // зел
  #define CONFIG_GPIO_ALARM_LED     CONFIG_GPIO_LED_RED   // Красный (мигающий?) сигнал аварии
  #define CONFIG_GPIO_IR_1          CONFIG_GPIO_IR_X1     // Выход на ИК блока 1 (резерв) 
  #define CONFIG_GPIO_IR_2          CONFIG_GPIO_IR_X2     // Выход на ИК блока 2 (резерв)

// Входы
  // #define CONFIG_GPIO_TSOP          CONFIG_GPIO_IR        // Вход ИК датчика
  // #define CONFIG_GPIO_FIRE          CONFIG_GPIO_X5        // Вход шлейфа охраны (резерв)
  // #define CONFIG_GPIO_TSTAT         CONFIG_GPIO_X4        // Вход термостата (резерв)
  // #define CONFIG_GPIO_POWER_1        CONFIG_GPIO_X6   // Вход контроля напряжения на реле бойлера
  // #define CONFIG_GPIO_POWER_2        CONFIG_GPIO_X7   // Вход контроля напряжения (резерв)

  // Датчики
  #define CONFIG_GPIO_TEMP_BOILER   CONFIG_GPIO_TEMP_X1    // Датчик температуры теплоносителя
  #define CONFIG_GPIO_TEMP_INDOOR   CONFIG_GPIO_TEMP_X2    // Датчик температуры помещения
  #define CONFIG_GPIO_TEMP_OUTDOOR  CONFIG_GPIO_TEMP_X3    // Датчик температуры внешний

  // Выходы
  #define CONFIG_GPIO_RELAY_BOILER  CONFIG_GPIO_RELAY_K1  // Выход на силовое реле бойлера
  //#define CONFIG_GPIO_RELAY_BOILER2   CONFIG_GPIO_RELAY_K2  // Выход на силовое реле (резерв)
  #define CONFIG_GPIO_ALARM_SIREN   CONFIG_GPIO_RELAY_K4  // Выход на реле сирены
  #define CONFIG_GPIO_ALARM_FLASH   CONFIG_GPIO_RELAY_K3  // Выход на реле маяка
  #define CONFIG_GPIO_RMT_RX_GPIO   CONFIG_GPIO_TEST_PAD  // Тестовый выход


  // Зоны сигнализации
  #define CONFIG_GPIO_ALARM_ZONE_1  CONFIG_GPIO_DOOR_X4   // Вход "Двери"
  #define CONFIG_GPIO_ALARM_ZONE_2  CONFIG_GPIO_FIRE_X5   // Вход шлейфа охраны (резерв)
  #define CONFIG_GPIO_ALARM_ZONE_3  CONFIG_GPIO_POWER_X6  // Вход контроля напряжения на реле бойлера
  #define CONFIG_GPIO_ALARM_ZONE_4  CONFIG_GPIO_POWER_X7  // Вход контроля напряжения (резерв)
  #define CONFIG_GPIO_ALARM_ZONE_5  CONFIG_GPIO_RXIR      // Вход ИК датчика
  #define CONFIG_GPIO_ALARM_LEVEL   0x01                  // уточнить

#endif

// ------------------------------------------------------------------------
//                          RU - MQTT клиент 
// ------------------------------------------------------------------------
/* Параметры MQTT-брокеров. Можно определить два брокера: основной и резервный
    CONFIG_MQTTx_TYPE :: 0 - public, 
                         1 - local, 
                         2 - gateway (CONFIG_MQTT1_HOST not used) */

/********************* primary server ************************/
#define CONFIG_MQTT1_TYPE 0
#define CONFIG_MQTT1_HOST "m8.wqtt.ru"
#define CONFIG_MQTT1_PING_CHECK 1
#define CONFIG_MQTT1_PING_CHECK_DURATION 250
#define CONFIG_MQTT1_PING_CHECK_LOSS 50.0
#define CONFIG_MQTT1_PING_CHECK_LIMIT 3
#define CONFIG_MQTT1_PORT_TCP 18668
#define CONFIG_MQTT1_PORT_TLS 18669
#define CONFIG_MQTT1_USERNAME "u_TMSO6A"
#define CONFIG_MQTT1_PASSWORD "xczIvuDx"
#define CONFIG_MQTT1_TLS_ENABLED 1
#define CONFIG_MQTT1_TLS_STORAGE CONFIG_DEFAULT_TLS_STORAGE
#define CONFIG_MQTT1_TLS_PEM_START CONFIG_DEFAULT_TLS_PEM_START
#define CONFIG_MQTT1_TLS_PEM_END CONFIG_DEFAULT_TLS_PEM_END
#define CONFIG_MQTT1_CLEAN_SESSION 1
#define CONFIG_MQTT1_AUTO_RECONNECT 1
#define CONFIG_MQTT1_KEEP_ALIVE 60
#define CONFIG_MQTT1_TIMEOUT 10000
#define CONFIG_MQTT1_RECONNECT 10000
#define CONFIG_MQTT1_CLIENTID "esp32_thermostat"
// #define CONFIG_MQTT1_LOC_PREFIX ""
// #define CONFIG_MQTT1_PUB_PREFIX ""
#define CONFIG_MQTT1_LOC_LOCATION "local/village"
#define CONFIG_MQTT1_PUB_LOCATION "village"
#define CONFIG_MQTT1_LOC_DEVICE "thermostat"
#define CONFIG_MQTT1_PUB_DEVICE "thermostat"

/****************** MQTT : pinger ********************/
// Разрешить публикацию результатов пинга на MQTT брокере
#define CONFIG_MQTT_PINGER_ENABLE 1
#if CONFIG_MQTT_PINGER_ENABLE
  // Название топика результатов пинга
  #define CONFIG_MQTT_PINGER_TOPIC "ping"
  #define CONFIG_MQTT_PINGER_LOCAL 0
  #define CONFIG_MQTT_PINGER_QOS 0
  #define CONFIG_MQTT_PINGER_RETAINED 1
  #define CONFIG_MQTT_PINGER_AS_PLAIN 0
  #define CONFIG_MQTT_PINGER_AS_JSON 1
#endif // CONFIG_MQTT_PINGER_ENABLE

/*************** MQTT : remote control ***************/
// Разрешить обработку входящих команд устройством
#define CONFIG_MQTT_COMMAND_ENABLE 1
// Разрешить OTA обновления через сторонний сервер
#define CONFIG_MQTT_OTA_ENABLE 1

/***************** MQTT : sensors ********************/
// Интервал публикации данных с сенсоров в секундах
#define CONFIG_MQTT_SENSORS_SEND_INTERVAL 600
// QOS для данных с сенсоров
#define CONFIG_MQTT_SENSORS_QOS 1
#define CONFIG_MQTT_SENSORS_LOCAL_QOS 2
// Сохранять на брокере последние отправленные данные
#define CONFIG_MQTT_SENSORS_RETAINED 1
#define CONFIG_MQTT_SENSORS_LOCAL_RETAINED 0

// ------------------------------------------------------------------------
//                 http://open-monitoring.online/ 
// ------------------------------------------------------------------------
// Включить отправку данных на open-monitoring.online
#define CONFIG_OPENMON_ENABLE 1
#if CONFIG_OPENMON_ENABLE
  // Периодичность отправки данных в секундах
  #define CONFIG_OPENMON_SEND_INTERVAL 300
  // Идентификаторы контроллеров и токены для open-monitoring.online
  #define CONFIG_OPENMON_CTR01_ID 9998
  #define CONFIG_OPENMON_CTR01_TOKEN "3W3aYt"
  // Разрешить публикацию результатов пинга и системной информации на open-monitoring.online
  #define CONFIG_OPENMON_PINGER_ENABLE 1
  #if CONFIG_OPENMON_PINGER_ENABLE
    #define CONFIG_OPENMON_PINGER_ID 9999
    #define CONFIG_OPENMON_PINGER_TOKEN "LUs2hg"
    #define CONFIG_OPENMON_PINGER_INTERVAL 300000
    #define CONFIG_OPENMON_PINGER_RSSI 1
    #define CONFIG_OPENMON_PINGER_HEAP_FREE 1
    #define CONFIG_OPENMON_PINGER_HOSTS 1
  #endif // CONFIG_OPENMON_PINGER_ENABLE
#endif // CONFIG_OPENMON_ENABLE

// ------------------------------------------------------------------------
//                        https://narodmon.ru/ 
// ------------------------------------------------------------------------
// Включить отправку данных на narodmon.ru
#define CONFIG_NARODMON_ENABLE 1
#if CONFIG_NARODMON_ENABLE
  // Периодичность отправки данных в секундах
  #define CONFIG_NARODMON_SEND_INTERVAL 300
  #define CONFIG_NARODMON_DEVICE01_ID 81693
  #define CONFIG_NARODMON_DEVICE01_KEY "9c:9e:9f:77:18:f7"
#endif // CONFIG_NARODMON_ENABLE

// ------------------------------------------------------------------------
//                        https://thingspeak.com/ 
// ------------------------------------------------------------------------
// RU: Включить отправку данных на thingspeak.com
#define CONFIG_THINGSPEAK_ENABLE 0
#if CONFIG_THINGSPEAK_ENABLE
  // Периодичность отправки данных в секундах
  #define CONFIG_THINGSPEAK_SEND_INTERVAL 300
  // Токены для записи на thingspeak.com
  #define CONFIG_THINGSPEAK_CHANNEL01_ID 99999999
  #define CONFIG_THINGSPEAK_CHANNEL01_KEY "LAJ6AA9AKBVTZ7AP"
#endif // CONFIG_THINGSPEAK_ENABLE

// ------------------------------------------------------------------------
//                  Отправка на свой хостинг 
// ------------------------------------------------------------------------
// Включить отправку данных на свой хостинг
#define CONFIG_HOSTING_ENABLE 0

// ------------------------------------------------------------------------
//                      Уведомления в Telegram 
// ------------------------------------------------------------------------
// Разрешить уведомления в Telegram (общий флаг)
#define CONFIG_TELEGRAM_ENABLE 1
// Токен бота API Telegram
#define CONFIG_TELEGRAM_TOKEN "7731157620:AAFO8eSNRMuGyqeMp5jPhU5mlQZpr_iHgoQ"
// Идентификатор чата или группы
#define CONFIG_TELEGRAM_CHAT_ID_MAIN     "-4568554589"
#define CONFIG_TELEGRAM_CHAT_ID_SERVICE  "-4539244144"
#define CONFIG_TELEGRAM_CHAT_ID_PARAMS   CONFIG_TELEGRAM_CHAT_ID_SERVICE
#define CONFIG_TELEGRAM_CHAT_ID_SECURITY CONFIG_TELEGRAM_CHAT_ID_MAIN
// Отправлять заголовок сообщения (название устройства, см. CONFIG_TELEGRAM_DEVICE)
#define CONFIG_TELEGRAM_TITLE_ENABLED 0
// Название устройства (публикуется в качестве заголовка сообщения)
#define CONFIG_TELEGRAM_DEVICE "🏡 ДОМ"

// ------------------------------------------------------------------------
//                              Уведомления 
// ------------------------------------------------------------------------
// Мигать системным светодиодом при отправке данных
#define CONFIG_SYSLED_MQTT_ACTIVITY 0
#define CONFIG_SYSLED_SEND_ACTIVITY 0
#define CONFIG_SYSLED_TELEGRAM_ACTIVITY 0
// Разрешить удаленную включение или отключение уведомлений без перепрошивки устройства
#define CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE 1
// Уведомления в Telegram при переключении режима охраны
#define CONFIG_NOTIFY_TELEGRAM_ALARM_MODE_CHANGE 1
#define CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_MODE_CHANGE 1
// Разрешить уведомления в Telegram, если поступил сигнал от датчика ОПС, который не зарегистрирован в системе
#define CONFIG_NOTIFY_TELEGRAM_ALARM_SENSOR_UNDEFINED 0
#define CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_SENSOR_UNDEFINED 0
// Разрешить уведомления в Telegram, если поступила незарегистрованная команда от датчика ОПС
#define CONFIG_NOTIFY_TELEGRAM_ALARM_COMMAND_UNDEFINED 0
#define CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_COMMAND_UNDEFINED 0
// Уведомления в Telegram при тревогах охраны
#define CONFIG_NOTIFY_TELEGRAM_ALARM_ALARM 1
#define CONFIG_NOTIFY_TELEGRAM_ALARM_ALERT_ALARM 1

// ------------------------------------------------------------------------
//                                Сенсоры 
// ------------------------------------------------------------------------
// Разрешить публикацию данных сенсора в простом виде (каждое значение в отдельном субтопике)
#define CONFIG_SENSOR_AS_PLAIN 0
// Разрешить публикацию данных сенсора в виде JSON в одном топике
#define CONFIG_SENSOR_AS_JSON 1
// Разрешить публикацию статуса сенсора
#define CONFIG_SENSOR_STATUS_ENABLE 1

// ------------------------------------------------------------------------
//                                ИК пульт 
// ------------------------------------------------------------------------
// Разрешить работу по протоколу NEC
#define CONFIG_IR_PROTOCOL_NEC 1
// Разрешить работу по протоколу RC5
#define CONFIG_IR_PROTOCOL_RC5 0

// ------------------------------------------------------------------------
//                            Log - Отладка 
// ------------------------------------------------------------------------
// Уровень отладочных сообщений. Всё, что выше этого уровня, будет исключено из кода
#define CONFIG_RLOG_PROJECT_LEVEL RLOG_LEVEL_DEBUG
// Добавлять коды цвета текста к сообщениям. Не работает на Win7
#define CONFIG_RLOG_PROJECT_COLORS 1
// Добавлять отметку времени к сообщениям
#define CONFIG_RLOG_SHOW_TIMESTAMP 1
// Добавлять информацию о файле и строке к сообщениям
#define CONFIG_RLOG_SHOW_FILEINFO 0


// Сохранять отладочную информацию при программном перезапуске устройства
#define CONFIG_RESTART_DEBUG_INFO 1
/* Разрешить периодическое сохранение информации о куче / памяти с 
    последующем выводом при перезапуске */
#define CONFIG_RESTART_DEBUG_HEAP_SIZE_SCHEDULE 1
// Глубина сохранения стека процессора при перезапуске. 0 - не сохранять
#define CONFIG_RESTART_DEBUG_STACK_DEPTH 28
// Разрешить публикацию отладочной информации из модуля WiFi
#define CONFIG_WIFI_DEBUG_ENABLE 0
// Разрешить периодическую публикацию системной информации
#define CONFIG_MQTT_SYSINFO_ENABLE 1
#define CONFIG_MQTT_SYSINFO_INTERVAL 1000*60*5
#define CONFIG_MQTT_SYSINFO_SYSTEM_FLAGS 1           
/* Разрешить периодическую публикацию информации о задачах. Должен быть 
    разрешен CONFIG_FREERTOS_USE_TRACE_FACILITY / configUSE_TRACE_FACILITY */
#define CONFIG_MQTT_TASKLIST_ENABLE 0
#define CONFIG_MQTT_TASKLIST_INTERVAL 1000*60*60
