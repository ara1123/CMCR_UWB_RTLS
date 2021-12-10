/*
File    : main.c
Purpose : Nordic nRF52840-DK build main entry point for simple exmaples.

*/

#include <stdio.h>
#include <stdlib.h>
#include <sdk_config.h>
#include <boards.h>
#include <port.h>
#include <deca_spi.h>
#include <examples_defines.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <shared_defines.h>
#include <example_selection.h>

/* UART STUFF */
//#include "app_uart.h"
//#include "app_error.h"
//#include "nrf_delay.h"
//#include "nrf.h"
//#include "nrf_drv_uart.h"

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

//void uart_error_handle(app_uart_evt_t * p_event);
/* END UART STUFF*/

#define UNIT_TEST 0
#define BLINK_FRAME_SN_IDX 1
#define FRAME_LENGTH    (sizeof(tx_msg)+FCS_LEN) //The real length that is going to be transmitted
#define TX_DELAY_MS 500
#define APP_NAME "ANTHONY TESTING"

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 */
static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E'};

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

extern example_ptr example_pointer;
extern int unit_test_main(void);
extern void build_examples(void);
extern dwt_txconfig_t txconfig_options;
/*! ------------------------------------------------------------------------------------------------------------------
* @fn test_run_info()
*
* @brief  This function is simply a printf() call for a string. It is implemented differently on other platforms,
*         but on the nRF52840-DK, a printf() call is .
*
* @param data - Message data, this data should be NULL string.
*
* output parameters
*
* no return value
*/
void test_run_info(unsigned char *data)
{
    printf("%s\r\n", data);
}

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,  /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/
int main(void) {
    /* USER CODE BEGIN 1 */
    uint8_t chip_select = 0;
    uint32_t err_code;
    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals (if attached). */
    build_examples();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);
    
    ///* Define UART Params */
    //const app_uart_comm_params_t comm_params =
    //  {
    //      RX_PIN_NUMBER,
    //      TX_PIN_NUMBER,
    //      RTS_PIN_NUMBER,
    //      CTS_PIN_NUMBER,
    //      NRF_UART_HWFC_DISABLED,
    //      false,
    //      NRF_UART_BAUDRATE_115200
    //  };

    //APP_UART_FIFO_INIT(&comm_params,
    //                   UART_RX_BUF_SIZE,
    //                   UART_TX_BUF_SIZE,
    //                   uart_error_handle,
    //                   APP_IRQ_PRIORITY_LOWEST,
    //                   err_code);

    //APP_ERROR_CHECK(err_code);
    //test_run_info((unsigned char *)"UART Config w/o a problem");

    /* Initialise the SPI for nRF52840-DK */
    nrf52840_dk_spi_init();

    /* Configuring interrupt*/
    dw_irq_init();

    /* Small pause before startup */
    nrf_delay_ms(2);
    
    //while (1)
    //{
      
    //  test_run_info((unsigned char *)"HELLO TO ANTHONY %d"); 
    //  nrf_delay_ms(2000);

    //};

    if(UNIT_TEST)
    {
        unit_test_main();
    }
    else
    {
        //Run the selected example as selected in example_selection.h
        example_pointer();
    }
    /* USER CODE END 2 */
    
  
    //chip_select = 1;
    //change_chip(chip_select);
    //read_dev_id_chip2();

    //chip_select = 0;
    //change_chip(chip_select);
    //read_dev_id();

//    example_pointer();
//    test_func();
//      simple_rx();




    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */
}

//void uart_error_handle(app_uart_evt_t * p_event)
//{
//    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_communication);
//    }
//    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_code);
//    }
//}

/*************************** End of file ****************************/
