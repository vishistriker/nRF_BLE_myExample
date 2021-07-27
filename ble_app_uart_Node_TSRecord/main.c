
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_fstorage_sd.h"
#include "nrf_fstorage.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"




#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define FIRMWARE_VERSION                "1_2"

//uncomment if battery monitor IC is used
//#define MAX1708_ENABLE                1
//#define SLEEP_CMD_ENABLE              1
#ifdef MAX1708_ENABLE
#include "nrf_drv_twi.h"
#include "MAX17048.h"
#endif

#define SERIAL_DEBUG_ENABLE             1
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define DEVICE_VERSION                  1
#define DEVICE_NAME                     "TSRecorder"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                1600                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 1000 ms). */

#define APP_ADV_DURATION                0                                        /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    100                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define CONFIGURATION_PAGE_ADDR          0x36000UL
#define CONFIGURATION_PAGE_END_ADDR      0x39FFFUL
#define RECORDED_TIMESTAMP_PAGE_ADDR     0x37000UL
#define RECORDED_TEMPERATURE_PAGE_ADDR   0x38000UL
#define NO_OF_FLASH_PAGE                 4

#define LED_PIN                         19
#define LED_PIN_1                         14
#define LED_PIN_2                         17
#define LED_PIN_3                         18
#define LED_PIN_4                         23
#define LED_PIN_5                         27
#define BUTTON_PIN                      20
#define MAX17048_ADDR                   0x36

#define CONFIG_CMD                      "0x02/"
#define CONFIG_CMD_ACK                  "0x02 OK"
#define CONFIG_CMD_RESPONSE             "0x52"

#define SET_TIME_CMD                 "0x01/"
#define SET_TIME_RESPONSE            "0x51"
#define SET_TIME_CMD_ACK             "0x01 OK"

#define READ_BAT_CMD                   "0x05/"
#define READ_BAT_CMD_ACK               "0x05 OK"

#define SLEEP_CMD                   "0x04/"
#define SLEEP_CMD_ACK               "0x04 OK"
#define SLEEP_CMD_RESPONSE           "0x54"

#define READ_TIMESTAMPS_CMD                   "0x03/"
#define READ_TIMESTAMPS_CMD_ACK               "0x03 OK"
#define READ_TIMESTAMPS_CMD_RESPONSE                    "0x53"

#define SET_PSWD_CMD                   "0x06/"
#define SET_PSWD_CMD_ACK               "0x06 OK"
#define SET_PSWD_CMD_RESPONSE_SUCCESS    "0x56,1"
#define SET_PSWD_CMD_RESPONSE_FAILED     "0x56,0"

#define GET_ACCESS_CMD                   "0x07/"
#define GET_ACCESS_CMD_ACK               "0x07 OK"
#define GET_ACCESS_CMD_RESPONSE_SUCCESS           "0x57,1"
#define GET_ACCESS_CMD_RESPONSE_FAILED            "0x57,0"

#define RECORD_TEMPERATURE_INTERVAL         7200 //15 min
#define LED_OFF_CNT                         480 //  60sec
#define WAKEUPMODE_CNT                    24
#define LED_INTERVAL                      240
#define TOTAL_TIMESTAMP                   250
#define STARTUP_LED_PATTERN               3 // 3 blinks
#define BATTERY_LOW_PERCENT               5

#define MASTER_PASSWORD                  "2021"
#define DEFAULT_USER_PASSWORD             "0000"

#define RAM_MEMORY_TEST_ADDRESS     (0x20005000UL)  /**< Address in RAM where test word (RAM_MEMORY_TEST_WORD) is written before System OFF and checked after System RESET.*/
#define RAM_MEMORY_TEST_WORD        (0xFEEDBEEFUL)  /**< Test word that is written to RAM address RAM_MEMORY_TEST_ADDRESS. */
#define RESET_MEMORY_TEST_BYTE      (0x0DUL)        /**< Known sequence written to a special register to check if this wake up is from System OFF. */
#define MAX_TEST_ITERATIONS         (1)             /**< Maximum number of iterations this example will run. */
#define SUCCESS_OUTPUT_VALUE        (0xAB)          /**< If RAM retention is tested for MAX_TEST_ITERATIONS, this value will be given as output.*/
#define SECS_PER_MIN                (60UL)
#define SECS_PER_HOUR               (3600UL)
#define COMP0_INT_INTERVAL_IN_SEC    120UL

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

uint8_t guUARTCmdBuffer[244];
uint16_t guWakeupModeCnt = 0;
bool gbCmdReceived=0, gbWakeup = 0;
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS 270  //!< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com.
#define ADC_RES_10BIT                  1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   6    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
//#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
//    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE)*3600) / ADC_RES_10BIT))

nrf_saadc_value_t guADCRaw_buf[2];                   //!< Buffer used for storing ADC value.
uint16_t          guBatteryVoltagelevelmillis,guBatteryInPercent= 0; //!< Current battery level.
uint16_t guLEDToggleCnt,guLEDCounter, gbProcessLED, guLEDDelay, gbConnected=0;
uint32_t * volatile p_ram_test = (uint32_t *)RAM_MEMORY_TEST_ADDRESS;
uint32_t guClockSeconds = 0, tickCnt=0;
uint8_t RtcEventGenerated = 0;
int giTemperature = 0,ButtonPressedCounter = 0,deviceLEDOffCnt = 0,RecordTemperatureIntervalCnt=0;
bool gbisUARTBusy = 0,gbButtonReleased=0,gbbootsuccess =0,gbAccessGiven = 0;
static void advertising_start(void);
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
void InitTimestampStorage();
typedef struct
{
  int32_t iTxPower;
  uint32_t DevVersion;
  uint8_t cDeviceName[20];
  uint8_t uDevPassword[5];
}DeviceConfig_t;

typedef struct
{
  uint32_t Timestamp[TOTAL_TIMESTAMP];
  int TimestampCount;
}Timestamp_t;

Timestamp_t gxRecordOfflineTimestamp;

typedef struct
{
  uint32_t Timestamp[TOTAL_TIMESTAMP];
  int Temperature[TOTAL_TIMESTAMP];
  int TimestampCount;
}TimestampTemperature_t;

TimestampTemperature_t gxRecordOfflineTemperature;

#ifdef MAX1708_ENABLE
MAX17048_t gxMAX17048;
#endif

DeviceConfig_t gxCurrentDeviceConfig,gxSavedDeviceConfig;
void vLEDOn(uint32_t pin_number);
void vLEDOff(uint32_t pin_number);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = CONFIGURATION_PAGE_ADDR,
    .end_addr   = CONFIGURATION_PAGE_END_ADDR,
};

/**@brief Set RTC time by providing UTC time.
 */
uint32_t vSetTime(uint32_t UTCTime)
{
   uint32_t tuClockSeconds = 0;
   *p_ram_test = 0;  
   tuClockSeconds = UTCTime + (5 * SECS_PER_HOUR) + (30 * SECS_PER_MIN) ;
   *p_ram_test = tuClockSeconds;
#if defined SERIAL_DEBUG_ENABLE
   NRF_LOG_INFO("Write,UI: %d,TRam: %d",tuClockSeconds,*p_ram_test);
   NRF_LOG_PROCESS();
#endif
   nrf_drv_rtc_counter_clear(&rtc);
   uint32_t err_code = nrf_drv_rtc_cc_set(&rtc,0,COMP0_INT_INTERVAL_IN_SEC * 8,true);
   APP_ERROR_CHECK(err_code);
}

/**@brief get Time in seconds.
 */
uint32_t uReadTime()
{
  uint32_t tuClockSeconds = 0;
  tuClockSeconds = *p_ram_test;
  tuClockSeconds += round((double)nrf_drv_rtc_counter_get(&rtc)/8.0);
#if defined SERIAL_DEBUG_ENABLE
  NRF_LOG_INFO("Read,UI: %d,TRam: %d",tuClockSeconds,*p_ram_test);
  NRF_LOG_PROCESS();
#endif
  nrf_drv_rtc_counter_clear(&rtc);
  uint32_t err_code = nrf_drv_rtc_cc_set(&rtc,0,COMP0_INT_INTERVAL_IN_SEC * 8,true);
  APP_ERROR_CHECK(err_code);
  *p_ram_test = tuClockSeconds;

  return tuClockSeconds;
}

  
/**@brief read ram retention register.
 */
void vCheckRamRetention()
{
  if ((NRF_POWER->GPREGRET >> 4) == RESET_MEMORY_TEST_BYTE)
    {
        NRF_POWER->GPREGRET = 0;
        guClockSeconds = *p_ram_test;
    }
}

/**@brief Init Ram retention.
 */
void vInitRamRetention()
{
    // Write the known sequence + loop_count to the GPREGRET register.
    NRF_POWER->GPREGRET = ((RESET_MEMORY_TEST_BYTE << 4));

    #if defined(NRF52832_XXAA) || defined(NRF51422) || defined(NRF51822)
    NRF_POWER->RAMONB |= (POWER_RAMONB_OFFRAM2_RAM2On << POWER_RAMONB_OFFRAM2_Pos)
                    | POWER_RAMONB_OFFRAM3_RAM3On << POWER_RAMONB_OFFRAM3_Pos
                    | POWER_RAMONB_ONRAM2_RAM2On << POWER_RAMONB_ONRAM2_Pos
                    | POWER_RAMONB_ONRAM3_RAM3On << POWER_RAMONB_ONRAM3_Pos;
    #endif
}



static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{

    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        *p_ram_test += nrf_drv_rtc_counter_get(&rtc)/8;
        nrf_drv_rtc_counter_clear(&rtc);
        uint32_t err_code = nrf_drv_rtc_cc_set(&rtc,0,COMP0_INT_INTERVAL_IN_SEC * 8,true);
        APP_ERROR_CHECK(err_code);
        RtcEventGenerated = 1;
        #if defined SERIAL_DEBUG_ENABLE
          NRF_LOG_INFO("Time:%d",*p_ram_test);
          NRF_LOG_PROCESS();
        #endif
    }
    else if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        guLEDCounter++;
        guLEDToggleCnt++;
        deviceLEDOffCnt++;
        RecordTemperatureIntervalCnt++;
        if(guLEDCounter >= guLEDDelay && gbProcessLED == 1)
        {
          gbProcessLED = 0;
          vLEDOff(LED_PIN);
        }
      
    }
}

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrf_drv_rtc_cc_set(&rtc,0,COMP0_INT_INTERVAL_IN_SEC * 8,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

/**@brief Function handling events from 'nrf_drv_saadc.c'.
 *
 * @param[in] p_evt SAADC event.
 */
static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_evt)
{
    if (p_evt->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result,adc_result1;

        adc_result = p_evt->data.done.p_buffer[0];
        adc_result1 = p_evt->data.done.p_buffer[1];

        guBatteryVoltagelevelmillis = ADC_RESULT_IN_MILLI_VOLTS(adc_result) ;//+ DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        //RealBatteryVoltagemillis = BAT_SLOPE * (float)(adc_result - REF_RAW_VAL) + (float)REF_VOLT;
        giTemperature = roundf((float)(3.3 * adc_result1 * 1000.0)/(float)(1024*10));
#if SERIAL_DEBUG_ENABLE
//        NRF_LOG_INFO("Bat = %d",guBatteryVoltagelevelmillis);
//        NRF_LOG_PROCESS();
//
//        NRF_LOG_INFO("LM35RAW = %d,%d",adc_result1,giTemperature);
//        NRF_LOG_PROCESS();
#endif
        double tfSlope = 1.0 * (100 - 0) / (2767 - 2433);
        guBatteryInPercent = 0 + tfSlope * (float)(guBatteryVoltagelevelmillis - 2433);
        if(guBatteryInPercent > 100)
          guBatteryInPercent = 100;
    }
}


void vBatteryInit(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);

    APP_ERROR_CHECK(err_code);

     nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);


    nrf_saadc_channel_config_t config1 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    err_code = nrf_drv_saadc_channel_init(1, &config1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&guADCRaw_buf[0], 2);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}


void vGetBatteryVoltage()
{
    if (!nrf_drv_saadc_is_busy())
    {
        ret_code_t err_code = nrf_drv_saadc_buffer_convert(&guADCRaw_buf[0], 2);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for updating the VBATT field of TLM*/
static void update_vbatt(void)
{
    vGetBatteryVoltage(); // Get new battery voltage
    
}


void vLEDOn(uint32_t pin_number)
{
  nrf_gpio_pin_write(pin_number, 1);
}

void vLEDOff(uint32_t pin_number)
{
  nrf_gpio_pin_write(pin_number, 0);
}

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
#if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("ERROR");
        NRF_LOG_PROCESS();
#endif
//        NRF_LOG_FLUSH();
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
//            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
//                         p_evt->len, p_evt->addr);
//                         NRF_LOG_FLUSH();
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
//            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
//                         p_evt->len, p_evt->addr);
//                         NRF_LOG_FLUSH();
        } break;

        default:
            break;
    }
}

static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
#if SERIAL_DEBUG_ENABLE
    NRF_LOG_INFO("========| flash info |========");
//    NRF_LOG_FLUSH();
NRF_LOG_PROCESS();
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
//    NRF_LOG_FLUSH();
NRF_LOG_PROCESS();
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
//    NRF_LOG_FLUSH();
NRF_LOG_PROCESS();
    NRF_LOG_INFO("==============================");
//    NRF_LOG_FLUSH();
NRF_LOG_PROCESS();
#endif
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        (void) sd_app_evt_wait();
    }
}


void vInitFstorage()
{
    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_sd;

   ret_code_t rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    print_flash_info(&fstorage);

    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    (void) nrf5_flash_end_addr_get();
}

void vWriteFlash(nrf_fstorage_t const * p_fs, uint32_t dest, void const * p_src, uint32_t len)
{
//    NRF_LOG_INFO("Writing");
//    NRF_LOG_PROCESS();
//    NRF_LOG_FLUSH();
    ret_code_t rc = nrf_fstorage_write(p_fs, dest, p_src, len, NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(p_fs);
//    NRF_LOG_INFO("Done.");
//    NRF_LOG_PROCESS();
//    NRF_LOG_FLUSH();
}

void vReadFlash(nrf_fstorage_t const * p_fs, uint32_t p_src, void * dest, uint32_t len)
{
//    NRF_LOG_INFO("Reading");
//    NRF_LOG_PROCESS();
//    NRF_LOG_FLUSH();
    ret_code_t rc = nrf_fstorage_read(p_fs, p_src, dest, len);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(p_fs);
//    NRF_LOG_INFO("Done.");
//    NRF_LOG_PROCESS();
//    NRF_LOG_FLUSH();
}

void vEraseFlash(nrf_fstorage_t const * p_fs, uint32_t pageAddress, uint32_t len, void * p_context)
{
//    NRF_LOG_INFO("Erasing");
//    NRF_LOG_PROCESS();
//    NRF_LOG_FLUSH();
    ret_code_t rc = nrf_fstorage_erase(p_fs, pageAddress, len, p_context);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(p_fs);
//    NRF_LOG_INFO("Done.");
//    NRF_LOG_PROCESS();
//    NRF_LOG_FLUSH();
}



/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)  &gxCurrentDeviceConfig.cDeviceName[0],
                                          strlen(gxCurrentDeviceConfig.cDeviceName));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        memset(guUARTCmdBuffer,0,sizeof(guUARTCmdBuffer));
        if(p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-1] == '\n' && p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-2] == '\r')
          strncpy(guUARTCmdBuffer,p_evt->params.rx_data.p_data,p_evt->params.rx_data.length-2);
        else if(p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-1] == '\n' || p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-1] == '\r')
          strncpy(guUARTCmdBuffer,p_evt->params.rx_data.p_data,p_evt->params.rx_data.length-1);
 #if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("Data : %s",guUARTCmdBuffer);
        NRF_LOG_PROCESS();
 #endif
        gbCmdReceived = 1;
    }

}
/**@snippet [Handling the data received over BLE] */


void vSendDataToBLEUART(uint8_t   * tuDataBuff)
{
   uint32_t err_code;
   do
    {
        uint16_t length = (uint16_t)strlen(tuDataBuff);
        err_code = ble_nus_data_send(&m_nus, tuDataBuff,&length , m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);
    gbisUARTBusy = 0;
}

void vProcessBLEUARTCmds(uint8_t * tuCmdBuff)
{
#if SERIAL_DEBUG_ENABLE
  NRF_LOG_INFO("Process CMD");
  NRF_LOG_PROCESS();
#endif
   if(!strncmp(GET_ACCESS_CMD,tuCmdBuff,5))
  {
     vSendDataToBLEUART(GET_ACCESS_CMD_ACK);
      int length = strlen(tuCmdBuff) - 5;
  #if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("length: %d",length);
        NRF_LOG_PROCESS();
  #endif
        if(length == 4)
        {
          if(!strncmp(&tuCmdBuff[5],gxCurrentDeviceConfig.uDevPassword,strlen(gxCurrentDeviceConfig.uDevPassword)) || !strncmp(&tuCmdBuff[5],MASTER_PASSWORD,strlen(MASTER_PASSWORD)))
          {
            gbAccessGiven = 1;
             vSendDataToBLEUART(GET_ACCESS_CMD_RESPONSE_SUCCESS);
          }
          else
          {
            gbAccessGiven = 0;
             vSendDataToBLEUART(GET_ACCESS_CMD_RESPONSE_FAILED);
          }
        }
        else
        {
           gbAccessGiven = 0;
            vSendDataToBLEUART(GET_ACCESS_CMD_RESPONSE_FAILED);
        }
        
  }
  else if(gbAccessGiven == 1)
  {
    if(!strncmp(CONFIG_CMD,tuCmdBuff,5))
    {
      int8_t TxPower;
      vReadFlash(&fstorage, CONFIGURATION_PAGE_ADDR, &gxSavedDeviceConfig, sizeof(gxSavedDeviceConfig));
      gxCurrentDeviceConfig.DevVersion = gxSavedDeviceConfig.DevVersion;
      gxCurrentDeviceConfig.iTxPower = gxSavedDeviceConfig.iTxPower;
      strncpy(gxCurrentDeviceConfig.uDevPassword,gxSavedDeviceConfig.uDevPassword,sizeof(gxCurrentDeviceConfig.uDevPassword)-1);
  //    gxCurrentDeviceConfig.cDeviceName = gxSavedDeviceConfig.cDeviceName;
      strncpy(gxCurrentDeviceConfig.cDeviceName,gxSavedDeviceConfig.cDeviceName,sizeof(gxCurrentDeviceConfig.cDeviceName)-1);
      vSendDataToBLEUART(CONFIG_CMD_ACK);
   #if SERIAL_DEBUG_ENABLE
      NRF_LOG_INFO("Config CMD");
      NRF_LOG_PROCESS();
   #endif
   

      int Index = 0,CommaCnt = 0;
       for(int Ind=0;Ind<strlen(tuCmdBuff);Ind++)
      {
        if(tuCmdBuff[Ind] == ',')
        {
          Index = Ind;
          CommaCnt++;
          //break;
        }
        if(CommaCnt == 1)
        {
          int length = Index - 5;
          memset(&gxCurrentDeviceConfig.cDeviceName,0,sizeof(gxCurrentDeviceConfig.cDeviceName));
          strncpy(gxCurrentDeviceConfig.cDeviceName,&tuCmdBuff[5],length);
           TxPower = atoi(&tuCmdBuff[Index+1]);
            gxCurrentDeviceConfig.iTxPower = (int32_t)TxPower;
          break;
        }
       }

      vEraseFlash(&fstorage, CONFIGURATION_PAGE_ADDR,1, NULL);
      vWriteFlash(&fstorage, CONFIGURATION_PAGE_ADDR, &gxCurrentDeviceConfig, sizeof(gxCurrentDeviceConfig));
      vSendDataToBLEUART(CONFIG_CMD_RESPONSE);
      nrf_delay_ms(300);
      vLEDOff(LED_PIN);
      NVIC_SystemReset();

  //    //nrf_gpio_cfg_input(18, NRF_GPIO_PIN_PULLUP);
  //    nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  //    NRF_POWER->SYSTEMOFF = 0x1;
  //     __DSB();
  //     __NOP();
  //    sd_ble_gap_adv_stop(0);
  //    nrf_delay_ms(200);
  //    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, 0, TxPower);
  //    advertising_start();
    }
    else if(!strncmp(SET_TIME_CMD,tuCmdBuff,5))
    {
      uint32_t UTCTimeValue = 0;
      vSendDataToBLEUART(SET_TIME_CMD_ACK);
      UTCTimeValue = (uint32_t) atoi(&tuCmdBuff[5]);
  #if SERIAL_DEBUG_ENABLE
      NRF_LOG_INFO("Time set CMD");
      NRF_LOG_PROCESS();
      NRF_LOG_INFO("UTC Time: %d",UTCTimeValue);
      NRF_LOG_PROCESS();
  #endif
      vSetTime(UTCTimeValue);
      vSendDataToBLEUART(SET_TIME_RESPONSE);
    }
    else if(!strncmp(READ_BAT_CMD,tuCmdBuff,5))
    {


        vSendDataToBLEUART(READ_BAT_CMD_ACK);

        //baterry monitor
//        vBatteryInit();
//        nrf_delay_ms(100);
        for(int Ind=0;Ind < 5;Ind++)
        {
          update_vbatt();
          nrf_delay_ms(5);
        }
//        nrf_delay_ms(50);
//        nrfx_saadc_channel_uninit(0);
//        nrfx_saadc_uninit();

         NRF_LOG_INFO("Batt: %d percent",guBatteryInPercent);
         NRF_LOG_PROCESS();



  #if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("Read Bat CMD");
        NRF_LOG_PROCESS();
  #endif
        uint8_t TempBuffer[50];
//        sprintf(TempBuffer,"TS:%d, Bat:%d%%, DevVer:%s",uReadTime(),guBatteryInPercent,FIRMWARE_VERSION);
        sprintf(TempBuffer,"0x55,%d",guBatteryInPercent);
        vSendDataToBLEUART(TempBuffer);
    }
     else if(!strncmp(READ_TIMESTAMPS_CMD,tuCmdBuff,5))
    {
        vSendDataToBLEUART(READ_TIMESTAMPS_CMD_ACK);
  #if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("Read timestamp");
        NRF_LOG_PROCESS();
  #endif

        //send timestamp
        Timestamp_t txRecordOfflineTimestamp;
        vReadFlash(&fstorage, RECORDED_TIMESTAMP_PAGE_ADDR, &txRecordOfflineTimestamp, sizeof(txRecordOfflineTimestamp));
         uint8_t TempBuffer[50];
        for(int index=0;index <txRecordOfflineTimestamp.TimestampCount ;index++)
        {
          memset(TempBuffer,0,sizeof(TempBuffer));       
          sprintf(TempBuffer,"#1,[%d]\r\n",txRecordOfflineTimestamp.Timestamp[index]);
          vSendDataToBLEUART(TempBuffer);
        }
//
//        uint8_t TempBuffer1[50];
//        sprintf(TempBuffer1,"%s,%d",READ_TIMESTAMPS_CMD_RESPONSE,txRecordOfflineTimestamp.TimestampCount);
//        vSendDataToBLEUART(TempBuffer1);

        // send temperature
        TimestampTemperature_t txRecordOfflineTemperature;
        vReadFlash(&fstorage, RECORDED_TEMPERATURE_PAGE_ADDR, &txRecordOfflineTemperature, sizeof(txRecordOfflineTemperature));
         uint8_t TempBuffer1[50];
        for(int index=0;index <txRecordOfflineTemperature.TimestampCount ;index++)
        {
          memset(TempBuffer1,0,sizeof(TempBuffer1));       
          sprintf(TempBuffer1,"#2,[%d,%d]\r\n",txRecordOfflineTemperature.Timestamp[index],txRecordOfflineTemperature.Temperature[index]);
          vSendDataToBLEUART(TempBuffer1);
        }

        uint8_t TempBuffer2[50];
        sprintf(TempBuffer2,"%s,%d,%d",READ_TIMESTAMPS_CMD_RESPONSE,txRecordOfflineTimestamp.TimestampCount,txRecordOfflineTemperature.TimestampCount);
        vSendDataToBLEUART(TempBuffer2);

        if(txRecordOfflineTimestamp.TimestampCount != 0 || txRecordOfflineTemperature.TimestampCount != 0)
        {
          InitTimestampStorage();
        }
    }
     else if(!strncmp(SET_PSWD_CMD,tuCmdBuff,5))
    {
       vSendDataToBLEUART(SET_PSWD_CMD_ACK);

        vReadFlash(&fstorage, CONFIGURATION_PAGE_ADDR, &gxSavedDeviceConfig, sizeof(gxSavedDeviceConfig));
        gxCurrentDeviceConfig.DevVersion = gxSavedDeviceConfig.DevVersion;
        gxCurrentDeviceConfig.iTxPower = gxSavedDeviceConfig.iTxPower;
        strncpy(gxCurrentDeviceConfig.uDevPassword,gxSavedDeviceConfig.uDevPassword,sizeof(gxCurrentDeviceConfig.uDevPassword)-1);
        strncpy(gxCurrentDeviceConfig.cDeviceName,gxSavedDeviceConfig.cDeviceName,sizeof(gxCurrentDeviceConfig.cDeviceName)-1);
      
        int length = strlen(tuCmdBuff) - 5;
  #if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("length: %d",length);
        NRF_LOG_PROCESS();
  #endif
        if(length == 4)
        {
          memset(&gxCurrentDeviceConfig.uDevPassword,0,sizeof(gxCurrentDeviceConfig.uDevPassword));
          strncpy(gxCurrentDeviceConfig.uDevPassword,&tuCmdBuff[5],length);

          vEraseFlash(&fstorage, CONFIGURATION_PAGE_ADDR,1, NULL);
          vWriteFlash(&fstorage, CONFIGURATION_PAGE_ADDR, &gxCurrentDeviceConfig, sizeof(gxCurrentDeviceConfig));

          vSendDataToBLEUART(SET_PSWD_CMD_RESPONSE_SUCCESS);
        }
        else{
          vSendDataToBLEUART(SET_PSWD_CMD_RESPONSE_FAILED);
        }
      
    }
  #ifdef SLEEP_CMD_ENABLE
    else if(!strncmp(SLEEP_CMD,tuCmdBuff,5))
    {
        vSendDataToBLEUART(SLEEP_CMD_ACK);
  #if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("Sleep CMD");
        NRF_LOG_PROCESS();
  #endif
        vSendDataToBLEUART(SLEEP_CMD_RESPONSE);
      nrf_delay_ms(300);
      vLEDOff(LED_PIN);
      nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
      NRF_POWER->SYSTEMOFF = 0x1;
       __DSB();
       __NOP();
    }
  #endif
  }


}

void InitTimestampStorage()
{
   memset(&gxRecordOfflineTimestamp,0,sizeof(gxRecordOfflineTimestamp));
   memset(&gxRecordOfflineTemperature,0,sizeof(gxRecordOfflineTemperature));
   vEraseFlash(&fstorage, RECORDED_TIMESTAMP_PAGE_ADDR,2, NULL);
   vWriteFlash(&fstorage, RECORDED_TIMESTAMP_PAGE_ADDR, &gxRecordOfflineTimestamp, sizeof(gxRecordOfflineTimestamp));
   vWriteFlash(&fstorage, RECORDED_TEMPERATURE_PAGE_ADDR, &gxRecordOfflineTemperature, sizeof(gxRecordOfflineTemperature));

}

void vInitFlashConfig()
{
  vReadFlash(&fstorage, CONFIGURATION_PAGE_ADDR, &gxSavedDeviceConfig, sizeof(gxSavedDeviceConfig));
  gxCurrentDeviceConfig.DevVersion = gxSavedDeviceConfig.DevVersion;
  gxCurrentDeviceConfig.iTxPower = gxSavedDeviceConfig.iTxPower;
  strncpy(gxCurrentDeviceConfig.uDevPassword,gxSavedDeviceConfig.uDevPassword,sizeof(gxCurrentDeviceConfig.uDevPassword)-1);
//  gxCurrentDeviceConfig.cDeviceName = gxSavedDeviceConfig.cDeviceName;
  strncpy(gxCurrentDeviceConfig.cDeviceName,gxSavedDeviceConfig.cDeviceName,sizeof(gxCurrentDeviceConfig.cDeviceName)-1);

  if(gxCurrentDeviceConfig.DevVersion != DEVICE_VERSION)
  {
    gxCurrentDeviceConfig.DevVersion = DEVICE_VERSION;
    gxCurrentDeviceConfig.iTxPower = 0;
    strncpy(gxCurrentDeviceConfig.uDevPassword,DEFAULT_USER_PASSWORD,sizeof(DEFAULT_USER_PASSWORD)-1);
//    gxCurrentDeviceConfig.cDeviceName = ;
    strncpy(gxCurrentDeviceConfig.cDeviceName,DEVICE_NAME,sizeof(DEVICE_NAME));
    vEraseFlash(&fstorage, CONFIGURATION_PAGE_ADDR,1, NULL);
    vWriteFlash(&fstorage, CONFIGURATION_PAGE_ADDR, &gxCurrentDeviceConfig, sizeof(gxCurrentDeviceConfig));
    InitTimestampStorage();
  }

  vReadFlash(&fstorage, RECORDED_TIMESTAMP_PAGE_ADDR, &gxRecordOfflineTimestamp, sizeof(gxRecordOfflineTimestamp));
  vReadFlash(&fstorage, RECORDED_TEMPERATURE_PAGE_ADDR, &gxRecordOfflineTemperature, sizeof(gxRecordOfflineTemperature));
  #ifdef SERIAL_DEBUG_ENABLE
  NRF_LOG_INFO("%d, %d, %s,%s, %d, %d",gxCurrentDeviceConfig.iTxPower,gxCurrentDeviceConfig.DevVersion,gxCurrentDeviceConfig.cDeviceName,gxCurrentDeviceConfig.uDevPassword,gxRecordOfflineTimestamp.TimestampCount,gxRecordOfflineTemperature.TimestampCount);
  NRF_LOG_PROCESS();
#endif
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            advertising_start();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
#if SERIAL_DEBUG_ENABLE
            NRF_LOG_INFO("Connected");
#endif
             vLEDOn(LED_PIN);
             gbConnected = 1;
             gbAccessGiven = 0;
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
#if SERIAL_DEBUG_ENABLE
            NRF_LOG_INFO("Disconnected");
#endif
             vLEDOff(LED_PIN);
             gbConnected = 0;
             gbAccessGiven = 0;
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        gbConnected= 0;
        gbAccessGiven = 0;
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        gbConnected=0;
        gbAccessGiven = 0;
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            gbConnected = 0;
            gbAccessGiven = 0;
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            gbConnected = 0;
            gbAccessGiven = 0;
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            //sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
//    ble_advdata_manuf_data_t adv_manuf_data;
    uint8_t buffer[10];

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

//    sprintf(buffer,"%d,%d",gxCurrentDeviceConfig.giNodeX,gxCurrentDeviceConfig.giNodeY);
//    adv_manuf_data.data.p_data        = buffer;
//    adv_manuf_data.data.size          = strlen(buffer);
//    adv_manuf_data.company_identifier = 0x0059; //Nordic's company ID
//    init.advdata.p_manuf_specific_data     = &adv_manuf_data;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

void vInitLED(uint32_t pin_number,uint32_t pin_number1,uint32_t pin_number2,uint32_t pin_number3,uint32_t pin_number4,uint32_t pin_number5)
{
    nrf_gpio_cfg_output(pin_number);
    nrf_gpio_cfg_output(pin_number1);
    nrf_gpio_cfg_output(pin_number2);
    nrf_gpio_cfg_output(pin_number3);
    nrf_gpio_cfg_output(pin_number4);
    nrf_gpio_cfg_output(pin_number5);
}

static void button_evt_handler(uint8_t pin_no, uint8_t button_action)
{
    if (button_action == APP_BUTTON_PUSH && pin_no == BUTTON_PIN && gbbootsuccess == 1)
    {
      NRF_LOG_INFO("Btn Sleeping");
//      NRF_LOG_PROCESS();
      vLEDOn(LED_PIN);
      nrf_delay_ms(1000);
      vLEDOff(LED_PIN);
//      //Device goto sleep
//      nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
//      NRF_POWER->SYSTEMOFF = 0x1;
//       __DSB();
//       __NOP();
     gbButtonReleased = 1;
     deviceLEDOffCnt = 0;
    }
}

static void button_init(void)
{
    ret_code_t              err_code;
    const uint8_t           buttons_cnt  = 1;
    static app_button_cfg_t buttons_cfgs =
    {
        .pin_no         = BUTTON_PIN,
        .active_state   = APP_BUTTON_ACTIVE_HIGH,
        .pull_cfg       = NRF_GPIO_PIN_PULLDOWN,
        .button_handler = button_evt_handler
    };

    err_code = app_button_init(&buttons_cfgs, buttons_cnt, APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

}

/**@brief Powerup Indication LED pattern.
 */
void vPowerupLEDPattern()
{
      vLEDOn(LED_PIN);
      nrf_delay_ms(500);
    for(int Ind=0;Ind<STARTUP_LED_PATTERN;Ind++)
    {
      vLEDOn(LED_PIN);
      nrf_delay_ms(150);
      vLEDOff(LED_PIN);
      nrf_delay_ms(150);
    }
}


void vLowBatteryLEDPattern()
{
//  for(int Ind=0;Ind<10;Ind++)
//    {
      vLEDOn(LED_PIN);
      nrf_delay_ms(1000);
      vLEDOff(LED_PIN);
      nrf_delay_ms(500);
//    }
}

int iGetTemperature()
{
  long sum=0;
  int temperature = 0;
  for(int Ind=0;Ind < 10;Ind++)
  {
    update_vbatt();
    nrf_delay_ms(5);
    sum += giTemperature;
//#if SERIAL_DEBUG_ENABLE
//    NRF_LOG_INFO("Temp:sum %d,gtemp:%d",sum,giTemperature);
//    NRF_LOG_PROCESS();
//#endif
  }
  temperature = roundf((float)sum/10);
  #if SERIAL_DEBUG_ENABLE
    NRF_LOG_INFO("Temp: %d,gtemp:%d",temperature,giTemperature);
    NRF_LOG_PROCESS();
#endif
  return temperature;
}


/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;

    //Init RamRetention for RTC clcok.retains last timestamp value
    vCheckRamRetention();
    vInitRamRetention();
    lfclk_config();
    rtc_config();
    // Initialize.
    //uart_init();
#if SERIAL_DEBUG_ENABLE
    log_init(); 
    NRF_LOG_INFO("0x%08X",NRF_POWER->RESETREAS);
    NRF_LOG_PROCESS();
#endif
    

    timers_init();
    button_init();
    //buttons_leds_init(&erase_bonds);
    vInitLED(LED_PIN,LED_PIN_1,LED_PIN_2,LED_PIN_3,LED_PIN_4,LED_PIN_5);

      if(NRF_POWER->RESETREAS == 0x00010000)     // GPIO Wake up
    {
      gbButtonReleased = 1;
     deviceLEDOffCnt = 0;
     NRF_POWER->RESETREAS = 0xffffffff; //clear reset reason
    }
    else
    {
      NRF_POWER->RESETREAS = 0xffffffff; //clear reset reason
    }

#ifdef SLEEP_CMD_ENABLE
    if(NRF_POWER->RESETREAS == 0x00010000)     // GPIO Wake up
    {
#if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("Wakeup");
        NRF_LOG_PROCESS();
#endif
      guWakeupModeCnt=0;
      while(nrf_gpio_pin_read(BUTTON_PIN))
      {
        nrf_delay_ms(125);
        if(guWakeupModeCnt < 160)
        {
          guWakeupModeCnt++;
        }
        if(guWakeupModeCnt == WAKEUPMODE_CNT)
        {
          vLEDOn(LED_PIN);
//          nrf_delay_ms(125);
//          vLEDOff(LED_PIN);
        }
      }
      vLEDOff(LED_PIN);

      if(guWakeupModeCnt == 160)
      {
        guWakeupModeCnt = 0;
      }
     
      if(guWakeupModeCnt >= WAKEUPMODE_CNT && nrf_gpio_pin_read(BUTTON_PIN) == 0 )
      {
        guWakeupModeCnt = 0;
        gbWakeup = 1;
      }

      if(gbWakeup != 1)
      {
#if SERIAL_DEBUG_ENABLE
        NRF_LOG_INFO("Sleeping");
        NRF_LOG_PROCESS();
#endif
//        vLEDOn(LED_PIN);
//        nrf_delay_ms(150);
//        vLEDOff(LED_PIN);
//        nrf_delay_ms(150);
        nrf_delay_ms(300);    
        //Device goto sleep
        nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
        NRF_POWER->SYSTEMOFF = 0x1;
         __DSB();
         __NOP();
      }
      NRF_POWER->RESETREAS = 0xffffffff;
    }
    else
    {
        NRF_POWER->RESETREAS = 0xffffffff;
    }
#endif  //sleep enable

    vInitFstorage();
    vInitFlashConfig();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    vBatteryInit();


    // Start execution.
    //printf("\r\nUART started.\r\n");
#if SERIAL_DEBUG_ENABLE
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    NRF_LOG_PROCESS();
#endif
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle,(int8_t)gxCurrentDeviceConfig.iTxPower);
    //sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, 0,4);
    advertising_start();

    if(gbButtonReleased != 1)
    {
      vPowerupLEDPattern();
    }
 // check at powerup if Battery percent is less than or equal to low percent indication. then blink LED for indication
#ifdef MAX1708_ENABLE
      if(percent <= BATTERY_LOW_PERCENT)
      {          
          vLowBatteryLEDPattern();
      }
#endif

    gbbootsuccess = 1;

// Enter main loop.
    for (;;)
    {
//          if(guLEDToggleCnt >= LED_INTERVAL && gbConnected != 1)
//          {
//            guLEDToggleCnt = 0;
//            guLEDCounter = 0;
//            gbProcessLED = 1;
//            guLEDDelay = 1;
//            vLEDOn(LED_PIN);
//          }

         if(gbCmdReceived == 1)
          {
            gbCmdReceived = 0;
            deviceLEDOffCnt = 0;
            vProcessBLEUARTCmds(guUARTCmdBuffer);
          }

          //RTC event generated to update RTC clock seconds
          if(RtcEventGenerated == 1 && gbConnected != 1)
          {


            
              RtcEventGenerated =0;
              //baterry monitor
//              vBatteryInit();
//              nrf_delay_ms(100);
              for(int Ind=0;Ind < 5;Ind++)
              {
                update_vbatt();
                nrf_delay_ms(5);
                
              }
//              nrf_delay_ms(50);
//              nrfx_saadc_channel_uninit(0);
//              nrfx_saadc_uninit();

               NRF_LOG_INFO("Batt: %d percent",guBatteryInPercent);
               NRF_LOG_PROCESS();

              //also check for low battery percent and indicate if so.
              if(guBatteryInPercent <= BATTERY_LOW_PERCENT)
              {

                for(int loop=0;loop<5;loop++)
                {
                   vLowBatteryLEDPattern();
                }
              }
              //end battery monitor
          }

/*         if(gbButtonReleased == 1 && gbConnected == 1 && gbAccessGiven == 1)
        {
          gbButtonReleased = 0;
          NRF_LOG_INFO("Btn released");
          NRF_LOG_PROCESS();
          uint8_t TempBuffer[50];
          sprintf(TempBuffer,"#1,[%d]\r\n",uReadTime());
          vSendDataToBLEUART(TempBuffer);
          ButtonPressedCounter++;
          switch(ButtonPressedCounter)
          {
            case 1:
            vLEDOn(LED_PIN_1);
            vLEDOff(LED_PIN_2);
            vLEDOff(LED_PIN_3);
            vLEDOff(LED_PIN_4);
            vLEDOff(LED_PIN_5);
            break;
            case 2:
            vLEDOn(LED_PIN_2);
            break;
            case 3:
            vLEDOn(LED_PIN_3);
            break;
            case 4:
            vLEDOn(LED_PIN_4);
            break;
            case 5:
            vLEDOn(LED_PIN_5);
            ButtonPressedCounter = 0;
            break;
            default:

            break;
          }
        }
        else if(gbButtonReleased == 1)
        {
          NRF_LOG_INFO("Btn released");
          NRF_LOG_PROCESS();

          gbButtonReleased = 0;
          uint32_t tempTimestamp = 0;
          tempTimestamp = uReadTime();
          if(gxRecordOfflineTimestamp.TimestampCount < TOTAL_TIMESTAMP)
          {
            gxRecordOfflineTimestamp.Timestamp[gxRecordOfflineTimestamp.TimestampCount] =  tempTimestamp;
            gxRecordOfflineTimestamp.TimestampCount++;
          }
          else
          {
            for(int index=0; index < (TOTAL_TIMESTAMP - 1); index++)
            {
              gxRecordOfflineTimestamp.Timestamp[index] = gxRecordOfflineTimestamp.Timestamp[index+1];
            }
            gxRecordOfflineTimestamp.Timestamp[(TOTAL_TIMESTAMP - 1)] =  tempTimestamp;
            
          }
          vEraseFlash(&fstorage, RECORDED_TIMESTAMP_PAGE_ADDR,1, NULL);
          vWriteFlash(&fstorage, RECORDED_TIMESTAMP_PAGE_ADDR, &gxRecordOfflineTimestamp, sizeof(gxRecordOfflineTimestamp));

          //if(gbConnected != 1)// do not executet this if phone connected to device
          {
//            guLEDCounter = 0;
//            gbProcessLED = 1;
//            guLEDDelay = 2;
              ButtonPressedCounter++;
              switch(ButtonPressedCounter)
              {
                case 1:
                vLEDOn(LED_PIN_1);
                vLEDOff(LED_PIN_2);
                vLEDOff(LED_PIN_3);
                vLEDOff(LED_PIN_4);
                vLEDOff(LED_PIN_5);
                break;
                case 2:
                vLEDOn(LED_PIN_2);
                break;
                case 3:
                vLEDOn(LED_PIN_3);
                break;
                case 4:
                vLEDOn(LED_PIN_4);
                break;
                case 5:
                vLEDOn(LED_PIN_5);
                ButtonPressedCounter = 0;
                break;
                default:

                break;
              }
          }
        }

        if(deviceLEDOffCnt >= LED_OFF_CNT)
        {
          deviceLEDOffCnt = 0;
          ButtonPressedCounter = 0;
          vLEDOff(LED_PIN_1);
          vLEDOff(LED_PIN_2);
          vLEDOff(LED_PIN_3);
          vLEDOff(LED_PIN_4);
          vLEDOff(LED_PIN_5);
        }
 */
/*
        if((RecordTemperatureIntervalCnt >= RECORD_TEMPERATURE_INTERVAL) && gbConnected == 1 && gbAccessGiven == 1)
        {
          RecordTemperatureIntervalCnt = 0;
           uint8_t TempBuffer[50];
          sprintf(TempBuffer,"#2,[%d,%d]\r\n",uReadTime(),iGetTemperature());
          vSendDataToBLEUART(TempBuffer);
        }
        else if(RecordTemperatureIntervalCnt >= RECORD_TEMPERATURE_INTERVAL)
        {
          RecordTemperatureIntervalCnt = 0;
          uint32_t tempTimestamp = 0;
          tempTimestamp = uReadTime();
          int temperature = iGetTemperature();
          if(gxRecordOfflineTemperature.TimestampCount < TOTAL_TIMESTAMP)
          {
            gxRecordOfflineTemperature.Timestamp[gxRecordOfflineTemperature.TimestampCount] =  tempTimestamp;
            gxRecordOfflineTemperature.Temperature[gxRecordOfflineTemperature.TimestampCount] = temperature;
            gxRecordOfflineTemperature.TimestampCount++;
          }
          else
          {
            for(int index=0; index < (TOTAL_TIMESTAMP - 1); index++)
            {
              gxRecordOfflineTemperature.Timestamp[index] = gxRecordOfflineTemperature.Timestamp[index+1];
              gxRecordOfflineTemperature.Temperature[index] = gxRecordOfflineTemperature.Temperature[index+1];
            }
            gxRecordOfflineTemperature.Timestamp[(TOTAL_TIMESTAMP - 1)] =  tempTimestamp;
            gxRecordOfflineTemperature.Temperature[(TOTAL_TIMESTAMP - 1)] =  temperature;
            
          }
          vEraseFlash(&fstorage, RECORDED_TEMPERATURE_PAGE_ADDR,1, NULL);
          vWriteFlash(&fstorage, RECORDED_TEMPERATURE_PAGE_ADDR, &gxRecordOfflineTemperature, sizeof(gxRecordOfflineTemperature));

        }
*/
        idle_state_handle();
    }
}


/**
 * @}
 */
