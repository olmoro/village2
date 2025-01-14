#ifndef _COMMON_CONFIG_H_
#define _COMMON_CONFIG_H_

#define PCB54           // Новая плата

#ifdef PCB54
  // Номер выводов периферии, версия платы URC-2E.54
  #define CONFIG_GPIO_LED_RED        4    // Красный, катод на GND (7mA)
  #define CONFIG_GPIO_LED_GREEN      2    // Зелёный, катод на GND (5mA)
  #define CONFIG_GPIO_LED_BLUE      12    // Синий,   катод на GND (4mA)

  // Входы
  #define CONFIG_GPIO_DOOR_X4       27    // Вход X4 Двери
  #define CONFIG_GPIO_FIRE_X5       26    // Вход X5 Пожар
  #define CONFIG_GPIO_POWER_X6      32    // Вход контроля напряжения на X6
  #define CONFIG_GPIO_POWER_X7      33    // Вход контроля напряжения на X7
  #define CONFIG_GPIO_RXIR          35    // Вход ИК датчика

  // Датчики температуры, все DS18B20
  #define CONFIG_GPIO_TEMP_X1       25    // Датчик температуры, подключен к X1
  #define CONFIG_GPIO_TEMP_X2        5    // Датчик температуры, подключен к X2
  #define CONFIG_GPIO_TEMP_X3       21    // Датчик температуры, подключен к X3

  // Выходы
  #define CONFIG_GPIO_RELAY_K1      19    // Выход на силовое реле K1 (X6)
  #define CONFIG_GPIO_RELAY_K2      18    // Выход на силовое реле K2 (X7)
  #define CONFIG_GPIO_RELAY_K3      22    // Выход на реле K3 (X8)
  #define CONFIG_GPIO_RELAY_K4      23    // Выход на реле K4 (X9)

  #define CONFIG_GPIO_TXD1          16    // UART
  #define CONFIG_GPIO_RXD1          17    // UART

  #define CONFIG_GPIO_TEST_PAD      14    // Test 

  // Отсутствующие на плате ресурсы
  #define CONFIG_GPIO_I2C           -1
  #define CONFIG_GPIO_RX433         -1
  #define CONFIG_GPIO_BUZZER        -1
  #define CONFIG_GPIO_BUZZER_ACTIVE -1


#else
  // Номер выводов периферии, версия платы 023 (с доработкой)
  // Светодиоды
  #define CONFIG_GPIO_LED_RED       12    // Красный, катод на GND (7mA)
  #define CONFIG_GPIO_LED_GREEN      2    // Зелёный, катод на GND (5mA)
  #define CONFIG_GPIO_LED_BLUE       4    // Синий,   катод на GND (4mA)
  #define CONFIG_GPIO_IR_X1         14    // Выход на ИК светодиод, подключен к X1, катод на GND 
  #define CONFIG_GPIO_IR_X2          5    // Выход на ИК светодиод, подключен к X2, катод на GND

  // Входы
  #define CONFIG_GPIO_IR            21    // Вход ИК датчика
  #define CONFIG_GPIO_X5            26    // Вход X5
  #define CONFIG_GPIO_X4            27    // Вход X4
  #define CONFIG_GPIO_LINE_X6       32    // Вход контроля напряжения на X6
  #define CONFIG_GPIO_LINE_X7       33    // Вход контроля напряжения на X7

  // Датчики температуры, все DS18B20
  #define CONFIG_GPIO_TEMP_X1       25    // Датчик температуры, подключен к X1
  #define CONFIG_GPIO_TEMP_X2       16    // Датчик температуры, подключен к X2
  #define CONFIG_GPIO_TEMP_X3       17    // Датчик температуры, подключен к X3

  // Выходы
  #define CONFIG_GPIO_RELAY_K1      19    // Выход на силовое реле K1 (X6)
  #define CONFIG_GPIO_RELAY_K2      18    // Выход на силовое реле K2 (X7)
  #define CONFIG_GPIO_RELAY_K3      22    // Выход на реле аварии K3 (X8)
  #define CONFIG_GPIO_RELAY_K4      23    // Выход на реле аварии K4 (X9)
#endif
// ------------------------------------------------------------------------
//                            Общие параметры 
// ------------------------------------------------------------------------
// Перезапустить устройство при ошибках выделения памяти
#define CONFIG_HEAP_ALLOC_FAILED_RESTART 0
#define CONFIG_HEAP_ALLOC_FAILED_RETRY 0

/* Разрешить "тихий" режим. Тихий режим - это период времени суток, когда 
   блокируются вспышки светодиодов и звуки. */
#define CONFIG_SILENT_MODE_ENABLE 1
#define CONFIG_SILENT_MODE_EXTENDED 0
/* Интервал в формате H1M1H2M2. То есть интервал 21:00 -> 06:00 это 21000600 */
#define CONFIG_SILENT_MODE_INTERVAL 22000600UL

// ------------------------------------------------------------------------
//                          Контроль температуры 
// ------------------------------------------------------------------------
// Здесь можно указать вообще любые параметры, связанные с прикладной задачей устройства

// Интервал чтения данных с сенсоров в миллисекундах
#define CONFIG_SENSORS_TASK_CYCLE 30000
/* Использовать статическое выделение памяти под задачу и очередь. 
     Должен быть включен параметр CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION! */
#define CONFIG_SENSORS_STATIC_ALLOCATION 1
// Размер стека для главной задачи
#define CONFIG_SENSORS_TASK_STACK_SIZE 4*1024

/* Разрешить публикацию необработанных RAW-данных (без коррекции и фильтрации):
    0 - только обработанное значение, 
    1 - всегда оба значения, 
    2 - только когда есть обработка */
#define CONFIG_SENSOR_RAW_ENABLE 1
// Разрешить публикацию форматированных данных в виде строки
#define CONFIG_SENSOR_STRING_ENABLE 0
// Разрешить публикацию отметки времени чтения данных с сенсора
#define CONFIG_SENSOR_TIMESTAMP_ENABLE 1
// Разрешить публикацию форматированных данных в виде "значение + время"
#define CONFIG_SENSOR_TIMESTRING_ENABLE 1
// // Разрешить вычисление и публикацию точки росы
// #define CONFIG_SENSOR_DEWPOINT_ENABLE 0
// Разрешить публикацию смешанного значения, например "температура + влажность"
#define CONFIG_SENSOR_DISPLAY_ENABLED 0
// Разрешить публикацию абсолютного минимума и максимума
#define CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE 1
// Разрешить публикацию ежедневного минимума и максимума
#define CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE 1
// Разрешить публикацию еженедельного минимума и максимума
#define CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE 1
// Публиковать экстеремумы только при их изменении
#define CONFIG_SENSOR_EXTREMUMS_OPTIMIZED 1

// ------------------------------------------------------------------------
//                          ИК пульт 
// ------------------------------------------------------------------------
// Интервал чтения данных с пульта в миллисекундах
// #define CONFIG_RX_TASK_CYCLE 300 // уточнить
// /* Использовать статическое выделение памяти под задачу и очередь. 
//      Должен быть включен параметр CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION! */
#define CONFIG_RX_STATIC_ALLOCATION 0 // 1 не получилось
// Размер стека для задачи
#define CONFIG_RX_TASK_STACK_SIZE 2*1024
// // RU: Размер очереди для задачи чтения данных с пульта
// #define CONFIG_ALARM_QUEUE_SIZE 32

#define IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define IR_TX_GPIO_NUM       16
#define IR_RX_GPIO_NUM       35
#define IR_NEC_DECODE_MARGIN 200     // Tolerance for parsing RMT symbols into bit stream

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0  9000
#define NEC_LEADING_CODE_DURATION_1  4500
#define NEC_PAYLOAD_ZERO_DURATION_0  560
#define NEC_PAYLOAD_ZERO_DURATION_1  560
#define NEC_PAYLOAD_ONE_DURATION_0   560
#define NEC_PAYLOAD_ONE_DURATION_1   1690
#define NEC_REPEAT_CODE_DURATION_0   9000
#define NEC_REPEAT_CODE_DURATION_1   2250


// ------------------------------------------------------------------------
//                           RU - Сигнализация 
// ------------------------------------------------------------------------
// EN: Use static memory allocation for the fire alarm task
// RU: Использовать статическое выделение памяти для задачи охранно-пожарной сигнализации
#define CONFIG_ALARM_STATIC_ALLOCATION 1

// EN: Stack size for the fire alarm task
// RU: Размер стека для задачи охранно-пожарной сигнализации
#define CONFIG_ALARM_STACK_SIZE 4098 * 2

// EN: Queue size for the fire alarm task
// RU: Размер очереди для задачи охранно-пожарной сигнализации
#define CONFIG_ALARM_QUEUE_SIZE 32

// RU: Топик устройства для ОПС
// #define CONFIG_ALARM_MQTT_DEVICE_TOPIC "home"

/* Публиковать состояние сенсоров ОПС в локальных топиках 
   для передачи на другие устройства */
#define CONFIG_ALARM_LOCAL_PUBLISH true

/* Схема топиков ОПС: 0 - %location%/config/security/mode; 
                      1 - %location%/%device%/config/security/mode */
#define CONFIG_ALARM_MQTT_DEVICE_MODE 0

/* Схема топиков ОПС: 0 - %location%/security/events/%zone%; 
                      1 - %location%/%device%/security/events/%zone% */
#define CONFIG_ALARM_MQTT_DEVICE_EVENTS 0

/* Схема топиков ОПС: 0 - %location%/security/status/%device%; 
                      1 - %location%/%device%/security/status */
#define CONFIG_ALARM_MQTT_DEVICE_STATUS 0

/* При отключении тревоги с пульта сразу же снять с охраны, 
   иначе отключить тревогу без снятия с охраны */
#define CONFIG_ALARM_TOGETHER_DISABLE_SIREN_AND_ALARM 1

// ------------------------------------------------------------------------
//                               WiFi сети 
// ------------------------------------------------------------------------
/* 
 *  Режим для одной сети
 * --------------------
 * Раскомментируйте CONFIG_WIFI_SSID и CONFIG_WIFI_PASS, чтобы отключить 
 * автоматическое переключение между wifi-сетями
 * */
#define CONFIG_WIFI_SSID "HUAWEI-a6mB"
#define CONFIG_WIFI_PASS "rmbDHxYK"

/** 
 * Режим для нескольких сетей
 * --------------------
 * Вы можете определенить от одной до пяти сетей. При невозможности 
 * подключиться к одной из сетей, ESP попытается поключиться к следующей. 
 * Номер успешного подключения запоминается и используется в дальнейшем
 *  (до следущего сбоя). 
 * Это позволяет переносить устройство из одного здания в другое, не 
 * перепрошивая и перенастраивая его. 
 * Просто заранее определите все возможные сети.
 * */
// #define CONFIG_WIFI_1_SSID "WIFI1"
// #define CONFIG_WIFI_1_PASS "000000000"
// #define CONFIG_WIFI_2_SSID "WIFI2"
// #define CONFIG_WIFI_2_PASS "111111111"
// #define CONFIG_WIFI_3_SSID "WIFI3"
// #define CONFIG_WIFI_3_PASS "222222222"
// #define CONFIG_WIFI_4_SSID "WIFI4"
// #define CONFIG_WIFI_4_PASS "333333333"
// #define CONFIG_WIFI_5_SSID "WIFI5"
// #define CONFIG_WIFI_5_PASS "444444444"

/* Параметры WiFi подключения. Если закомментировать эти строки, будут 
    использованы параметры по умолчанию ESP-IDF */
// #define CONFIG_WIFI_STORAGE   WIFI_STORAGE_RAM
// #define CONFIG_WIFI_BANDWIDTH WIFI_BW_HT20

/* Перезапустить устройство, если нет подключения к WiFi более указанного
    времени в минутах. 
   Закомментируйте строку, если не нужно перезапускать устройство при длительном 
   отсутствии подключения к сети */
#define CONFIG_WIFI_TIMER_RESTART_DEVICE 60*24

/* Разрешить периодическую проверку доступности сети интернет с помошью пинга. 
    Иногда доступ в сеть может пропасть, но подключение к WiFi при этом работает. 
    В этом случае устройство приостановит все сетевые процессы. */
#define CONFIG_PINGER_ENABLE 1

/* Отключить иникацию сетевых ошибок (wifi, inetnet, openmon, tg...), так как 
    устройство не всегда подключено к сети */
#define CONFIG_OFFLINE_MODE 0

#endif // _COMMON_CONFIG_H_
