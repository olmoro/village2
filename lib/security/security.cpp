#include "security.h"
#include <stdbool.h>
#include "project_config.h"
#include "common_config.h"
#include "def_alarm.h"
#include "def_consts.h"
#include "mTypes.h"
#include "reGpio.h"
#include "reLed.h"
#include "reEvents.h"
#include "reParams.h"
#include "rLog.h"

#include "rmt.h"   

static const char* logTAG = "ALARM";

// ------------------------------------------------------------------------
//                          Проводные входы 
// ------------------------------------------------------------------------

// Объекты reGPIO для обработки прерываний по проводным входым ОПС
static reGPIO gpioAlarm1(CONFIG_GPIO_ALARM_ZONE_1, CONFIG_GPIO_ALARM_LEVEL, false, true, CONFIG_BUTTON_DEBOUNCE_TIME_US, nullptr);
static reGPIO gpioAlarm2(CONFIG_GPIO_ALARM_ZONE_2, CONFIG_GPIO_ALARM_LEVEL, false, true, CONFIG_BUTTON_DEBOUNCE_TIME_US, nullptr);
static reGPIO gpioAlarm3(CONFIG_GPIO_ALARM_ZONE_3, CONFIG_GPIO_ALARM_LEVEL, false, true, CONFIG_BUTTON_DEBOUNCE_TIME_US, nullptr);
static reGPIO gpioAlarm4(CONFIG_GPIO_ALARM_ZONE_4, CONFIG_GPIO_ALARM_LEVEL, false, true, CONFIG_BUTTON_DEBOUNCE_TIME_US, nullptr);
#if defined (CONFIG_GPIO_ALARM_ZONE_5) && (CONFIG_GPIO_ALARM_ZONE_5 > -1)
  //static reGPIO gpioAlarm5(CONFIG_GPIO_ALARM_ZONE_5, CONFIG_GPIO_ALARM_LEVEL, false, true, CONFIG_BUTTON_DEBOUNCE_TIME_US, nullptr);
#endif // CONFIG_GPIO_ALARM_ZONE_5
// ------------------------------------------------------------------------
//                             Инициализация 
// ------------------------------------------------------------------------

void alarmInitDevices()
{
  rlog_i(logTAG, "Initialization of AFS devices");

  // Создаем светодиоды, сирену и флешер
  ledQueue_t ledAlarm = nullptr;
  #if defined(CONFIG_GPIO_ALARM_LED) && (CONFIG_GPIO_ALARM_LED > -1)
    ledAlarm = ledTaskCreate(CONFIG_GPIO_ALARM_LED, true, true, "led_alarm", CONFIG_LED_TASK_STACK_SIZE, nullptr);
    ledTaskSend(ledAlarm, lmOff, 0, 0, 0);
  #endif // CONFIG_GPIO_ALARM_LED

  ledQueue_t ledMode = nullptr;
  #if defined(CONFIG_GPIO_MODE_LED) && (CONFIG_GPIO_MODE_LED > -1)
    ledMode = ledTaskCreate(CONFIG_GPIO_MODE_LED, true, true, "led_mode", CONFIG_LED_TASK_STACK_SIZE, nullptr);
    ledTaskSend(ledMode, lmOff, 0, 0, 0);
  #endif // CONFIG_GPIO_MODE_LED

  ledQueue_t siren = nullptr;
  #if defined(CONFIG_GPIO_ALARM_SIREN) && (CONFIG_GPIO_ALARM_SIREN > -1)
    siren = ledTaskCreate(CONFIG_GPIO_ALARM_SIREN, true, false, "siren", CONFIG_LED_TASK_STACK_SIZE, nullptr);
    ledTaskSend(siren, lmOff, 0, 0, 0);
  #endif // CONFIG_GPIO_ALARM_SIREN

  ledQueue_t flasher = nullptr;
  #if defined(CONFIG_GPIO_ALARM_FLASH) && (CONFIG_GPIO_ALARM_FLASH > -1)
    flasher = ledTaskCreate(CONFIG_GPIO_ALARM_FLASH, true, true, "flasher", CONFIG_LED_TASK_STACK_SIZE, nullptr);
    ledTaskSend(flasher, lmBlinkOn, 1, 100, 5000);
  #endif // CONFIG_GPIO_ALARM_FLASH
  
  // Запускаем задачу             -mode-   -alarm-   -pult-
  alarmTaskCreate(siren, flasher, ledMode, ledAlarm, ledMode, nullptr);

  // // Запускаем приемник RX 433 MHz
  // #if defined (CONFIG_GPIO_RX433) && (CONFIG_GPIO_RX433 > -1)
  //   rx433_Init(CONFIG_GPIO_RX433, alarmTaskQueue());
  //   rx433_Enable();
  // #endif // CONFIG_GPIO_RX433

  // Запускаем приемник ИК
  #if defined (CONFIG_GPIO_RXIR) && (CONFIG_GPIO_RXIR > -1)
    /*  */
    rxIR_Init(CONFIG_GPIO_RXIR, alarmTaskQueue());
    rxIR_Enable();
  #endif // CONFIG_GPIO_RX_IR
}

void alarmInitSensors()
{
  rlog_i(logTAG, "Initialization of AFS zones");
  // ------------------------------------------------------------------------
  // Настраиваем зоны охраны
  // ------------------------------------------------------------------------

  // Двери (периметр) :: создаем зону охраны
  alarmZoneHandle_t azDoors = alarmZoneAdd(
    "Двери",           // Понятное название зоны
    "doors",           // MQTT-топик зоны
    nullptr            // Функция управления реле, при необходимости
  );
  // Настраиваем реакции для данной зоны в разных режимах
  alarmResponsesSet(
    azDoors,           // Ссылка на зону охраны
    ASM_DISABLED,      // Настраиваем реакции для режима ASM_DISABLED - "охрана отключена"
    ASRS_REGISTER,     // Реакция на события тревоги: только регистрация (фактически это приводит к публикации его на MQTT)
    ASRS_REGISTER      // Реакция на отмену тревоги: только регистрация (фактически это приводит к публикации его на MQTT)
  );
  alarmResponsesSet(
    azDoors,           // Ссылка на зону охраны
    ASM_ARMED,         // Настраиваем реакции для режима ASM_ARMED - "полная охрана"
    ASRS_ALARM_SIREN,  // Реакция на события тревоги: включить сирену и отправить уведомления
    ASRS_REGISTER      // Реакция на отмену тревоги: только регистрация (фактически это приводит к публикации его на MQTT)
  );
  alarmResponsesSet(
    azDoors,           // Ссылка на зону охраны
    ASM_PERIMETER,     // Настраиваем реакции для режима ASM_PERIMETER - "только периметр (дома)" 
    ASRS_ALARM_SIREN,  // Реакция на события тревоги: включить сирену и отправить уведомления
    ASRS_REGISTER      // Реакция на отмену тревоги: только регистрация (фактически это приводит к публикации его на MQTT)
  );
  alarmResponsesSet(
    azDoors,           // Ссылка на зону охраны
    ASM_OUTBUILDINGS,  // Настраиваем реакции для режима ASM_OUTBUILDINGS - "внешние помещения" 
    ASRS_ALARM_NOTIFY, // Реакция на события тревоги: тихая тревога - отправить уведомления, но сирену не включать
    ASRS_REGISTER      // Реакция на отмену тревоги: только регистрация (фактически это приводит к публикации его на MQTT)
  );

  // Датчики дыма и пламени - тревога 24*7
  alarmZoneHandle_t azFire = alarmZoneAdd(
    "Пожар", 
    "fire", 
    nullptr);
  alarmResponsesSet(
    azFire, 
    ASM_DISABLED, 
    ASRS_ALARM_SILENT, 
    ASRS_ALARM_NOTIFY);
  alarmResponsesSet(
    azFire, 
    ASM_ARMED, 
    ASRS_ALARM_SIREN, 
    ASRS_ALARM_NOTIFY);
  alarmResponsesSet(
    azFire, 
    ASM_PERIMETER, 
    ASRS_ALARM_SIREN, 
    ASRS_ALARM_NOTIFY);
  alarmResponsesSet(
    azFire, 
    ASM_OUTBUILDINGS, 
    ASRS_ALARM_SIREN, 
    ASRS_ALARM_NOTIFY);


  // Контроль сетевого напряжения на X6
  #if defined(CONFIG_GPIO_ALARM_ZONE_3) && (CONFIG_GPIO_ALARM_ZONE_3 > -1)
    alarmZoneHandle_t azPower1 = alarmZoneAdd(
      "Контроль питания", 
      "power1", 
      nullptr);
    alarmResponsesSet(
      azPower1, 
      ASM_DISABLED, 
      ASRS_REGISTER, 
      ASRS_REGISTER);
    alarmResponsesSet(
      azPower1, 
      ASM_ARMED, 
      ASRS_REGISTER, 
      ASRS_REGISTER);
    alarmResponsesSet(
      azPower1, 
      ASM_PERIMETER, 
      ASRS_REGISTER, 
      ASRS_REGISTER);
    alarmResponsesSet(
      azPower1, 
      ASM_OUTBUILDINGS, 
      ASRS_REGISTER, 
      ASRS_REGISTER);
  #endif  // CONFIG_GPIO_ALARM_ZONE_3

  // Контроль сетевого напряжения на X7
  #if defined(CONFIG_GPIO_ALARM_ZONE_4) && (CONFIG_GPIO_ALARM_ZONE_4 > -1)
    alarmZoneHandle_t azPower2 = alarmZoneAdd(
      "Контроль питания", 
      "power2", 
      nullptr);
    alarmResponsesSet(
      azPower2, 
      ASM_DISABLED, 
      ASRS_REGISTER, 
      ASRS_REGISTER);
    alarmResponsesSet(
      azPower2, 
      ASM_ARMED, 
      ASRS_REGISTER, 
      ASRS_REGISTER);
    alarmResponsesSet(
      azPower2, 
      ASM_PERIMETER, 
      ASRS_REGISTER, 
      ASRS_REGISTER);
    alarmResponsesSet(
      azPower2, 
      ASM_OUTBUILDINGS, 
      ASRS_REGISTER, 
      ASRS_REGISTER);
  #endif // CONFIG_GPIO_ALARM_ZONE_4

  // Тревожные кнопки
  alarmZoneHandle_t azButtons = alarmZoneAdd("Тревожные кнопки", "buttons", nullptr);
  alarmResponsesSet(azButtons, ASM_DISABLED, ASRS_ALARM_SIREN, ASRS_ALARM_NOTIFY);
  alarmResponsesSet(azButtons, ASM_ARMED, ASRS_ALARM_SIREN, ASRS_ALARM_NOTIFY);
  alarmResponsesSet(azButtons, ASM_PERIMETER, ASRS_ALARM_SIREN, ASRS_ALARM_NOTIFY);
  alarmResponsesSet(azButtons, ASM_OUTBUILDINGS, ASRS_ALARM_SIREN, ASRS_ALARM_NOTIFY);



    // Контроль ИК
  // ИК пульт управления                old:   433 MHz пульты управления
  alarmZoneHandle_t azRemoteControls = alarmZoneAdd("Пульт управления", "controls", nullptr);
  alarmResponsesSet(azRemoteControls, ASM_DISABLED, ASRS_CONTROL, ASRS_CONTROL);
  alarmResponsesSet(azRemoteControls, ASM_ARMED, ASRS_CONTROL, ASRS_CONTROL);
  alarmResponsesSet(azRemoteControls, ASM_PERIMETER, ASRS_CONTROL, ASRS_CONTROL);
  alarmResponsesSet(azRemoteControls, ASM_OUTBUILDINGS, ASRS_CONTROL, ASRS_CONTROL);

  rlog_i(logTAG, "Initialization of AFS sensors");

  // ------------------------------------------------------------------------
  // Проводные входы для встроенных GPIO
  // ------------------------------------------------------------------------
  // Проводная зона 1: входная дверь
  gpioAlarm1.initGPIO();
  alarmSensorHandle_t asWired1 = alarmSensorAdd(
    AST_WIRED,                                      // Тип датчика: проводные датчики
    "Входная дверь",                                // Понятное имя датчика
    "door",                                         // Топик датчика
    CONFIG_ALARM_LOCAL_PUBLISH,                     // Использовать локальные топики для передачи сигналов на другие устройства, в примере = TRUE (0x01)
    CONFIG_GPIO_ALARM_ZONE_1                        // Номер вывода или адрес датчика
  );
  if (asWired1) {
    alarmEventSet(asWired1, azDoors, 0, ASE_ALARM, 
      1, CONFIG_ALARM_EVENT_MESSAGE_DOOROPEN,       // Сообщение при сигнале тревоги: "🚨 Открыта дверь"
      0, NULL,                                      // Сообщение при отмене тревоги: отсутствует
      1,                                            // Порог срабатывания (нужен только для беспроводных датчиков, для остальных = 1)
      0,                                            // Время автосброса тревоги по таймеру, 0 = отключено
      60,                                           // Период публикации на MQTT брокере
      false);                                       // Тревога без подтверждения с других датчиков
  };

  // Проводная зона 2: PIR сенсор в прихожей
  gpioAlarm2.initGPIO();
  alarmSensorHandle_t asGasLeak = alarmSensorAdd(AST_WIRED, "Газ", "gas", CONFIG_ALARM_LOCAL_PUBLISH, CONFIG_GPIO_ALARM_ZONE_3);
  if (asGasLeak) {
    alarmEventSet(asGasLeak, azFire, 0, ASE_ALARM, 
      1, CONFIG_ALARM_EVENT_MESSAGE_GAS,            // Сообщение при сигнале тревоги: "🚨 Обнаружена утечка газа"
      0, CONFIG_ALARM_EVENT_MESSAGE_CLEAR,          // Сообщение при отмене тревоги: "🟢 Авария устранена"
      1,                                            // Порог срабатывания (нужен только для беспроводных датчиков, для остальных = 1)
      0,                                            // Время автосброса тревоги по таймеру, 0 = отключено
      60,                                           // Период публикации на MQTT брокере
      false);                                       // Тревога без подтверждения с других датчиков
  };

  // Проводная зона 3: контроль питания 220В на первом
  #if defined(CONFIG_GPIO_ALARM_ZONE_3) && (CONFIG_GPIO_ALARM_ZONE_3 > -1)
    gpioAlarm3.initGPIO();
    alarmSensorHandle_t asPowerMain1 = alarmSensorAdd(AST_WIRED, "Питание 220В", "main_power", CONFIG_ALARM_LOCAL_PUBLISH, CONFIG_GPIO_ALARM_ZONE_3);
    if (asPowerMain1) {
      alarmEventSet(asPowerMain1, azPower1, 0, ASE_POWER, 
        1, CONFIG_ALARM_EVENT_MESSAGE_POWER_MAIN_OFF, // Сообщение при сигнале тревоги: "🔴 Питание первого блока отключено"
        0, CONFIG_ALARM_EVENT_MESSAGE_POWER_MAIN_ON,  // Сообщение при отмене тревоги: "💡 Питание первого блока восстановлено"
        1,                                            // Порог срабатывания (нужен только для беспроводных датчиков, для остальных = 1)
        0,                                            // Время автосброса тревоги по таймеру, 0 = отключено
        0,                                            // Без повторной публикации состояния
        false);                                       // Тревога без подтверждения с других датчиков
    };
  #endif // CONFIG_GPIO_ALARM_ZONE_3

  // Проводная зона 4: контроль питания 220В на втором
  #if defined(CONFIG_GPIO_ALARM_ZONE_4) && (CONFIG_GPIO_ALARM_ZONE_4 > -1)
    gpioAlarm4.initGPIO();
    alarmSensorHandle_t asPowerMain2 = alarmSensorAdd(AST_WIRED, "Питание 220В", "main_power", CONFIG_ALARM_LOCAL_PUBLISH, CONFIG_GPIO_ALARM_ZONE_4);
    if (asPowerMain2) {
      alarmEventSet(asPowerMain2, azPower2, 0, ASE_POWER, 
        1, CONFIG_ALARM_EVENT_MESSAGE_POWER_MAIN_OFF, // Сообщение при сигнале тревоги: "🔴 Питание второго блока отключено"
        0, CONFIG_ALARM_EVENT_MESSAGE_POWER_MAIN_ON,  // Сообщение при отмене тревоги: "💡 Питание второго блока восстановлено"
        1,                                            // Порог срабатывания (нужен только для беспроводных датчиков, для остальных = 1)
        0,                                            // Время автосброса тревоги по таймеру, 0 = отключено
        0,                                            // Без повторной публикации состояния
        false);                                       // Тревога без подтверждения с других датчиков
    };
  #endif // CONFIG_GPIO_ALARM_ZONE_4

  // ... 

  // Беспроводные датчики и пульт 433 МГц в системе отсутствуют

  // // -----------------------------------------------------------------------------------
  // // пульт управления 433 МГц
  // // -----------------------------------------------------------------------------------
  // alarmSensorHandle_t asRC_R2 = alarmSensorAdd(     
  //   AST_RX433_20A4C,                                // Тип датчика: беспроводной
  //   "Пульт",                                        // Название пульта
  //   "rc",                                           // Топик пульта
  //   false,                                          // Локальные топики не используются
  //   0x0004F9CB                                      // Адрес пульта
  // );
  // if (asRC_R2) {
  //   alarmEventSet(asRC_R2, azRemoteControls, 0,     // Зона "пультов"
  //     ASE_CTRL_OFF,                                 // Команда отключения режима охраны
  //     0x01, NULL,                                   // Код команды 0x01, без сообщений
  //     ALARM_VALUE_NONE, NULL,                       // Кода отмены нет, без сообщений
  //     2,                                            // Должно придти как минимум 2 кодовых посылки для переключения
  //     3*1000,                                       // Время автосброса: 3 секунды
  //     0,                                            // Без повторной публикации состояния
  //     false);                                       // Не требуется подтвеждение с других датчиков
  //   alarmEventSet(asRC_R2, azRemoteControls, 1,     // Зона "пультов"
  //     ASE_CTRL_ON,                                  // Команда включения режима охраны
  //     0x08, NULL,                                   // Код команды 0x08, без сообщений
  //     ALARM_VALUE_NONE, NULL,                       // Кода отмены нет, без сообщений
  //     2,                                            // Должно придти как минимум 2 кодовых посылки для переключения
  //     3*1000,                                       // Время автосброса: 3 секунды
  //     0,                                            // Без повторной публикации состояния
  //     false);                                       // Не требуется подтвеждение с других датчиков
  //   alarmEventSet(asRC_R2, azRemoteControls, 2,     // Зона "пультов"
  //     ASE_CTRL_PERIMETER,                           // Команда включения режима "периметр"
  //     0x04, NULL,                                   // Код команды 0x04, без сообщений
  //     ALARM_VALUE_NONE, NULL,                       // Кода отмены нет, без сообщений
  //     2,                                            // Должно придти как минимум 2 кодовых посылки для переключения
  //     3*1000,                                       // Время автосброса: 3 секунды
  //     0,                                            // Без повторной публикации состояния
  //     false);                                       // Не требуется подтвеждение с других датчиков
  //   alarmEventSet(asRC_R2, azButtons, 3,            // Зона "тревожные кнопки"
  //     ASE_ALARM,                                    // Команда "тревога"
  //     0x02, NULL,                                   // Код команды 0x02, сообщение "🔴 Нажата тревожная кнопка"
  //     ALARM_VALUE_NONE, NULL,                       // Кода отмены нет, без сообщений
  //     2,                                            // Должно придти как минимум 2 кодовых посылки для переключения
  //     3*1000,                                       // Время автосброса: 3 секунды
  //     0,                                            // Без повторной публикации состояния
  //     false);                                       // Не требуется подтвеждение с других датчиков
  // };

  // -----------------------------------------------------------------------------------
  // пульт управления ИК, 21 кнопка
  // -----------------------------------------------------------------------------------
  alarmSensorHandle_t asRC_IR = alarmSensorAdd(     
    AST_RXIR_16A_16C,         // Тип датчика: беспроводной    //AST_RX433_20A4C,
    "Пульт",                  // Название пульта
    "rc",                     // Топик пульта
    false,                    // Локальные топики не используются
    0x0000FF00                // Адрес пульта                 //0x0004F9CB
  );
  if (asRC_IR) {
    alarmEventSet(asRC_IR, azRemoteControls, 0,     // Зона "пультов"
      ASE_CTRL_OFF,           // Команда отключения режима охраны
      (uint32_t)0xEA15, NULL, // Код команды VOL-, без сообщений  // Код команды 0x01, без сообщений
      ALARM_VALUE_NONE, NULL, // Кода отмены нет, без сообщений
      2,                      // Должно придти как минимум 2 кодовых посылки для переключения
      3*1000,                 // Время автосброса: 3 секунды
      0,                      // Без повторной публикации состояния
      false);                 // Не требуется подтвеждение с других датчиков
    alarmEventSet(asRC_IR, azRemoteControls, 1,     // Зона "пультов"
      ASE_CTRL_ON,            // Команда включения режима охраны
      (uint32_t)0xF609, NULL, // Код команды VOL+, без сообщений    // Код команды 0x08, без сообщений
      ALARM_VALUE_NONE, NULL, // Кода отмены нет, без сообщений
      2,                      // Должно придти как минимум 2 кодовых посылки для переключения
      3*1000,                 // Время автосброса: 3 секунды
      0,                      // Без повторной публикации состояния
      false);                 // Не требуется подтвеждение с других датчиков



  };


  rlog_i(logTAG, "Initialization of AFS completed");
}

// ------------------------------------------------------------------------
//                             Внешние датчики 
//                          в системе отсутствуют
// ------------------------------------------------------------------------
// static bool _extToiletPir  = false;
// static bool _extKitchenPir = false;

// #define EXT_DATA_QOS                  2
// #define EXT_DATA_FRIENDLY             "Состояние"

// #define EXT_DATA_TOILET_PIR_ID         0xFF000001
// #define EXT_DATA_TOILET_PIR_KEY       "toilet_pir"
// #define EXT_DATA_TOILET_PIR_TOPIC     "security/home/toilet/pir"     // local/security/home/toilet/pir/status
// #define EXT_DATA_TOILET_PIR_FRIENDLY  "Санузел"

// #define EXT_DATA_KITCHEN_PIR_ID        0xFF000002
// #define EXT_DATA_KITCHEN_PIR_KEY      "kitchen_pir"
// #define EXT_DATA_KITCHEN_PIR_TOPIC    "security/home/kitchen/pir"     // local/security/home/kitchen/pir/status
// #define EXT_DATA_KITCHEN_PIR_FRIENDLY "Кухня"

// static void alarmExternalSensorsEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
// {
//   if (*(uint32_t*)event_data == (uint32_t)&_extToiletPir) {
//     if ((event_id == RE_PARAMS_CHANGED) || (event_id == RE_PARAMS_EQUALS)) {
//       vTaskDelay(1);
//       alarmPostQueueExtId(IDS_MQTT, EXT_DATA_TOILET_PIR_ID, _extToiletPir);
//     };
//   } else if (*(uint32_t*)event_data == (uint32_t)&_extKitchenPir) {
//     if ((event_id == RE_PARAMS_CHANGED) || (event_id == RE_PARAMS_EQUALS)) {
//       vTaskDelay(1);
//       alarmPostQueueExtId(IDS_MQTT, EXT_DATA_KITCHEN_PIR_ID, _extKitchenPir);
//     };
//   };
// }

// static void alarmExternalSensorsInit()
// {
//   paramsGroupHandle_t extDataToilet = paramsRegisterGroup(nullptr, 
//     EXT_DATA_TOILET_PIR_KEY, EXT_DATA_TOILET_PIR_TOPIC, EXT_DATA_TOILET_PIR_FRIENDLY);
//   if (extDataToilet) {
//     paramsRegisterValue(OPT_KIND_LOCDATA_ONLINE, OPT_TYPE_U8, nullptr, extDataToilet, 
//       CONFIG_ALARM_MQTT_EVENTS_STATUS, EXT_DATA_FRIENDLY, EXT_DATA_QOS, &_extToiletPir);
//   };

//   paramsGroupHandle_t extDataKitchen = paramsRegisterGroup(nullptr, 
//     EXT_DATA_KITCHEN_PIR_KEY, EXT_DATA_KITCHEN_PIR_TOPIC, EXT_DATA_KITCHEN_PIR_FRIENDLY);
//   if (extDataKitchen) {
//     paramsRegisterValue(OPT_KIND_LOCDATA_ONLINE, OPT_TYPE_U8, nullptr, extDataKitchen, 
//       CONFIG_ALARM_MQTT_EVENTS_STATUS, EXT_DATA_FRIENDLY, EXT_DATA_QOS, &_extKitchenPir);
//   };

//   eventHandlerRegister(RE_PARAMS_EVENTS, RE_PARAMS_CHANGED, alarmExternalSensorsEventHandler, nullptr);
//   eventHandlerRegister(RE_PARAMS_EVENTS, RE_PARAMS_EQUALS, alarmExternalSensorsEventHandler, nullptr);
// }


void alarmStart()
{
  alarmInitDevices();
  //alarmExternalSensorsInit();
  alarmInitSensors();
}
