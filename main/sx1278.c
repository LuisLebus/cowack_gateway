/**
 * @file sx1278.c
 * @author
 * @date
 * @brief
 */

//=============================================================================
// [Inclusions] ===============================================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "sx1278.h"


//=============================================================================

//=============================================================================
// [Private Defines] ==========================================================
#define SX1278_TAG          "[SX1278]"

#define SX1278_RX_DONE_BIT  BIT0
#define SX1278_TX_DONE_BIT  BIT1

//=============================================================================

//=============================================================================
// [Local Typedef] ============================================================

//=============================================================================

//=============================================================================
// [External Data Definitions] ================================================

// Const ---------------------------------------------
//----------------------------------------------------

// Vars ----------------------------------------------
//----------------------------------------------------

// Task Handlers -------------------------------------
//----------------------------------------------------

// Queue Handlers ------------------------------------
//----------------------------------------------------

// Event Group Handlers ------------------------------
//----------------------------------------------------

// Semaphore Handlers --------------------------------
//----------------------------------------------------

//=============================================================================

//=============================================================================
// [Local Data Declarations] ==================================================

// Const ---------------------------------------------
//----------------------------------------------------

// Vars ----------------------------------------------
static spi_device_handle_t sx1278_spi_handle = NULL;
static bool sx1278_enable_rx = false;

//----------------------------------------------------

// Task Handlers -------------------------------------
//----------------------------------------------------

// Queue Handlers ------------------------------------
static QueueHandle_t sx1278_rx_queue = NULL;

//----------------------------------------------------

// Event Group Handlers ------------------------------
static EventGroupHandle_t sx1278_ev_group = NULL;

//----------------------------------------------------

// Semaphore Handlers --------------------------------
//----------------------------------------------------

//=============================================================================

//=============================================================================
// [Local Function Declarations] ==============================================

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static uint8_t sx1278_set_bit(uint8_t value, uint8_t bit);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static uint8_t sx1278_clear_bit(uint8_t value, uint8_t bit);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static bool sx1278_get_bit(uint8_t value, uint8_t bit);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static uint8_t sx1278_write_bit(uint8_t value, uint8_t bit, bool bit_value);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static uint8_t sx1278_write_bits(uint8_t value, uint8_t value_in, uint8_t start, uint8_t len);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
//static uint8_t sx1278_read_bits(uint8_t value_in, uint8_t start, uint8_t len);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_spi_write(uint8_t address, uint8_t val);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static uint8_t sx1278_spi_read(uint8_t address);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_set_payload_crc_on(bool val);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_set_payload_max_length(uint8_t val);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_set_implicit_header_mode_on(bool val);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_set_io_pin(uint8_t io, uint8_t val);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static bool sx1278_get_flag(uint8_t flag);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_clear_flag(uint8_t flag);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_set_state(uint8_t state);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_start_lora_mode(void);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void IRAM_ATTR sx1278_gpio_rx_done_isr_handler(void * arg);

//=============================================================================

//=============================================================================
// [FreeRTOS Task Definitions] ================================================

static void sx1278_rx_task(void * pvParameter)
{
    uint8_t startadd = 0;
    uint8_t i = 0;
    uint8_t n = 0;
    uint8_t data[SX1278_BUFF_SIZE] = {0};

    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1)
    {
    	xEventGroupWaitBits(sx1278_ev_group, SX1278_RX_DONE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    	startadd = sx1278_spi_read(SX1278_REG_LR_FIFORXCURRENTADDR);
    	sx1278_spi_write(SX1278_REG_LR_FIFOADDRPTR, startadd);

    	n = sx1278_spi_read(SX1278_REG_LR_NBRXBYTES);

    	if ((n != SX1278_BUFF_SIZE) || (true == sx1278_get_flag(SX1278_PAYLOAD_CRC_ERROR)))
    	{
    		for (i = 0; i < n; i++)
    		{
    			sx1278_spi_read(SX1278_REG_LR_FIFO);
    		}
    	}
    	else
    	{
    		for (i = 0; i < n; i++)
    		{
    			data[i] = sx1278_spi_read(SX1278_REG_LR_FIFO);
    		}

    		xQueueSend(sx1278_rx_queue, data, 0);
    	}

    	sx1278_clear_flag(SX1278_PAYLOAD_CRC_ERROR);
    	sx1278_clear_flag(SX1278_RX_DONE);
    }
}
// End

//=============================================================================

//=============================================================================
// [Local Function Definitions] ===============================================

uint8_t sx1278_set_bit(uint8_t value, uint8_t bit)
{
    return value | (1 << bit);
}
// End

uint8_t sx1278_clear_bit(uint8_t value, uint8_t bit)
{
    return value & ~(1 << bit);
}
// End

bool sx1278_get_bit(uint8_t value, uint8_t bit)
{
    return (bool)(value & (1 << bit));
}
// End

uint8_t sx1278_write_bit(uint8_t value, uint8_t bit, bool bit_value)
{
    if (true == bit_value)
    {
        value = sx1278_set_bit(value, bit);
    }
    else
    {
        value = sx1278_clear_bit(value, bit);
    }

    return value;
}
// End

uint8_t sx1278_write_bits(uint8_t value, uint8_t value_in, uint8_t start, uint8_t len)
{
    uint8_t i;

    for(i=0; i<len; i++)
    {
        value = sx1278_write_bit(value, i + start, sx1278_get_bit(value_in, i));
    }

    return value;
}
// End

//uint8_t sx1278_read_bits(uint8_t value_in, uint8_t start, uint8_t len)
//{
//    uint8_t i;
//    uint8_t value = 0;
//
//    for (i = 0; i < len; i++)
//    {
//        value = sx1278_write_bit(value, i, sx1278_get_bit(value_in, i + start));
//    }
//
//    return value;
//}
// End

static void sx1278_spi_write(uint8_t address, uint8_t val)
{
    spi_transaction_t t = {0};
    uint8_t dataTx[2] = {0};

    dataTx[0] = address | 0b10000000;
    dataTx[1] = val;

    t.length = 16;
    t.tx_buffer = dataTx;

    spi_device_polling_transmit(sx1278_spi_handle, &t);
}
// End

static uint8_t sx1278_spi_read(uint8_t address)
{
    spi_transaction_t t = {0};
    uint8_t dataTx[2] = {0};
    uint8_t dataRx[2] = {0};

    dataTx[0] = address;

    t.length = 16;
    t.tx_buffer = dataTx;
    t.rx_buffer = dataRx;

    spi_device_transmit(sx1278_spi_handle, &t);

    return dataRx[1];
}
// End

static void sx1278_set_payload_crc_on(bool val)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_MODEMCONFIG2);
    sx1278_write_bit(b, 2, val);
    sx1278_spi_write(SX1278_REG_LR_MODEMCONFIG2, b);
}
// End

static void sx1278_set_payload_max_length(uint8_t val)
{
    sx1278_spi_write(SX1278_REG_LR_PAYLOADMAXLENGTH, val);
}
// End

static void sx1278_set_implicit_header_mode_on(bool val)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_MODEMCONFIG1);
    sx1278_write_bit(b, 0, val);
    sx1278_spi_write(SX1278_REG_LR_MODEMCONFIG1, b);
}
// End

static void sx1278_set_io_pin(uint8_t io, uint8_t val)
{
    uint8_t b0 = sx1278_spi_read(SX1278_REG_LR_DIOMAPPING1);
    uint8_t b1 = sx1278_spi_read(SX1278_REG_LR_DIOMAPPING2);

    switch(io)
    {
        case 0:
            b0 = sx1278_write_bits(b0, val, 6, 2);
        break;

        case 1:
            b0 = sx1278_write_bits(b0, val, 4, 2);
        break;

        case 2:
            b0 = sx1278_write_bits(b0, val, 2, 2);
        break;

        case 3:
            b0 = sx1278_write_bits(b0, val, 0, 2);
        break;

        case 4:
            b1 = sx1278_write_bits(b1, val, 6, 2);
        break;

        case 5:
            b1 = sx1278_write_bits(b1, val, 4, 2);
        break;

        default:
            // Nothing to do.
        break;
    }

    sx1278_spi_write(SX1278_REG_LR_DIOMAPPING1, b0);
    sx1278_spi_write(SX1278_REG_LR_DIOMAPPING2, b1);
}
// End

static bool sx1278_get_flag(uint8_t flag)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_IRQFLAGS);
    return sx1278_get_bit(b, flag);
}
// End

static void sx1278_clear_flag(uint8_t flag)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_IRQFLAGS);
    b = sx1278_set_bit(b, flag);
    sx1278_spi_write(SX1278_REG_LR_IRQFLAGS, b);
}
// End

static void sx1278_set_state(uint8_t state)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_OPMODE);
    b = sx1278_write_bits(b, state, 0, 3);
    sx1278_spi_write(SX1278_REG_LR_OPMODE, b);
    ets_delay_us(5000);
}
// End

static void sx1278_start_lora_mode(void)
{
    sx1278_set_state(SX1278_SLEEP);
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_OPMODE);
    b = sx1278_set_bit(b, 7);
    sx1278_spi_write(SX1278_REG_LR_OPMODE, b);

    sx1278_set_state(SX1278_STDBY);

    sx1278_clear_flag(SX1278_RX_DONE);
    sx1278_clear_flag(SX1278_TX_DONE);
}
// End

static void IRAM_ATTR sx1278_gpio_rx_done_isr_handler(void * arg)
{
    xEventGroupSetBitsFromISR(sx1278_ev_group, SX1278_RX_DONE_BIT, NULL);
}
// End

//=============================================================================

//=============================================================================
// [External Function Definition] =============================================

void sx1278_init(void)
{
    spi_bus_config_t spi_bus_config = {
            .miso_io_num = SX1278_GPIO_MISO,
            .mosi_io_num = SX1278_GPIO_MOSI,
            .sclk_io_num = SX1278_GPIO_SCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
    };

    spi_device_interface_config_t spi_device_interface_config = {
            .clock_speed_hz = 100000,               //Clock out at 100 KHz
            .mode = 0,                              //SPI mode 0
            .spics_io_num = SX1278_GPIO_NSS,  		//CS pin
            .queue_size = 7,                        //We want to be able to queue 7 transactions at a time
            .pre_cb = NULL,                         //Specify pre-transfer callback to handle D/C line
    };


    gpio_pad_select_gpio(SX1278_GPIO_DIO1);
	gpio_set_direction(SX1278_GPIO_DIO1, GPIO_MODE_INPUT);

	gpio_pad_select_gpio(SX1278_GPIO_RESET);
	gpio_set_direction(SX1278_GPIO_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(SX1278_GPIO_RESET, true);


    spi_bus_initialize(SPI2_HOST, &spi_bus_config, 0);
    spi_bus_add_device(SPI2_HOST, &spi_device_interface_config, &sx1278_spi_handle);


    sx1278_start_lora_mode();
    sx1278_spi_write(SX1278_REG_LR_OCP, 0x2F);  //Current trim : 120mA

    sx1278_set_payload_crc_on(true);
    sx1278_set_implicit_header_mode_on(true);
    sx1278_set_io_pin(0, 0);
    sx1278_set_payload_max_length(SX1278_BUFF_SIZE);

    sx1278_ev_group = xEventGroupCreate();
    sx1278_rx_queue = xQueueCreate(10, SX1278_BUFF_SIZE);
}
// End

void sx1278_write(uint8_t * data)
{
    uint8_t i = 0;

    sx1278_set_state(SX1278_STDBY);

    sx1278_spi_write(SX1278_REG_LR_PAYLOADLENGTH, SX1278_BUFF_SIZE);

    uint8_t base_addr = sx1278_spi_read(SX1278_REG_LR_FIFOTXBASEADDR);
    sx1278_spi_write(SX1278_REG_LR_FIFOADDRPTR, base_addr);

    for (i = 0; i < SX1278_BUFF_SIZE; i++)
    {
        sx1278_spi_write(SX1278_REG_LR_FIFO, data[i]);
    }

    sx1278_set_state(SX1278_TX);

    for (i = 0; i < 30; i++)
    {
        vTaskDelay(100 / portTICK_RATE_MS);

        if( true == sx1278_get_flag(SX1278_TX_DONE) )
        {
        	ESP_LOGI(SX1278_TAG, "SX1278_TX_DONE");
            break;
        }
    }

    sx1278_set_state(SX1278_STDBY);
    sx1278_clear_flag(SX1278_TX_DONE);
    sx1278_clear_flag(SX1278_RX_DONE);

    if (true == sx1278_enable_rx)
    {
    	sx1278_set_state(SX1278_RXCONT);
    }
}
// End

bool sx1278_read(uint8_t * data)
{
    bool ret_val = false;

    if( xQueueReceive(sx1278_rx_queue, data, portMAX_DELAY) == pdTRUE )
    {
        ret_val = true;
    }

    return ret_val;
}
// End

void sx1278_set_frequency(float frequency)
{
    uint32_t freq = (uint32_t)( (frequency * 1000000) / 61.03515625 );

    sx1278_spi_write(SX1278_REG_LR_FRFMSB, (uint8_t)(freq >> 16));
    sx1278_spi_write(SX1278_REG_LR_FRFMID, (uint8_t)(freq >> 8));
    sx1278_spi_write(SX1278_REG_LR_FRFLSB, (uint8_t)(freq >> 0));
}
// End

void sx1278_set_boost_on(bool boost_on)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_PACONFIG);
    b = sx1278_write_bit(b, 7, boost_on);
    sx1278_spi_write(SX1278_REG_LR_PACONFIG, b);
}
// End

void sx1278_set_power(uint8_t power)
{
    uint8_t b1 = sx1278_spi_read(SX1278_REG_LR_PACONFIG);
    uint8_t b2 = sx1278_spi_read(SX1278_REG_LR_PADAC);

    switch(power)
    {
        case 1:
            b1 = sx1278_write_bits(b1, 5, 0, 4);
            b2 = sx1278_write_bits(b2, 4, 0, 3);
        break;

        case 2:
            b1 = sx1278_write_bits(b1, 8, 0, 4);
            b2 = sx1278_write_bits(b2, 4, 0, 3);
        break;

        case 3:
            b1 = sx1278_write_bits(b1, 11, 0, 4);
            b2 = sx1278_write_bits(b2, 4, 0, 3);
        break;

        case 4:
            b1 = sx1278_write_bits(b1, 15, 0, 4);
            b2 = sx1278_write_bits(b2, 4, 0, 3);
        break;

        case 5:
            b1 = sx1278_write_bits(b1, 15, 0, 4);
            b2 = sx1278_write_bits(b2, 7, 0, 3);
        break;

        default:
            // Nothing to do.
        break;
    }

    sx1278_spi_write(SX1278_REG_LR_PADAC, b2);
    sx1278_spi_write(SX1278_REG_LR_PACONFIG, b1);
}
// End

void sx1278_set_spreading_factor(uint8_t spreading_factor)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_MODEMCONFIG2);

    if(spreading_factor < 6)
        spreading_factor = 6;

    if(spreading_factor > 12)
        spreading_factor = 12;

    b = sx1278_write_bits(b, spreading_factor, 4, 4);
    sx1278_spi_write(SX1278_REG_LR_MODEMCONFIG2, b);
}
// End

void sx1278_set_bandwidth(uint8_t bandwidth)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_MODEMCONFIG1);

    if(bandwidth > 9)
        bandwidth = 9;

    b = sx1278_write_bits(b, bandwidth, 4, 4);
    sx1278_spi_write(SX1278_REG_LR_MODEMCONFIG1, b);
}
// End

void sx1278_set_coding_rate(uint8_t coding_rate)
{
    uint8_t b = sx1278_spi_read(SX1278_REG_LR_MODEMCONFIG1);

    if(coding_rate < 1)
        coding_rate = 1;

    if(coding_rate > 4)
        coding_rate = 4;

    b = sx1278_write_bits(b, coding_rate, 1, 3);
    sx1278_spi_write(SX1278_REG_LR_MODEMCONFIG1, b);
}
// End

void sx1278_set_rx_enable(void)
{
	sx1278_enable_rx = true;

	gpio_pad_select_gpio(SX1278_GPIO_DIO0);
	gpio_set_direction(SX1278_GPIO_DIO0, GPIO_MODE_INPUT);
	gpio_set_intr_type(SX1278_GPIO_DIO0, GPIO_INTR_POSEDGE);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(SX1278_GPIO_DIO0, sx1278_gpio_rx_done_isr_handler, NULL);

	sx1278_set_state(SX1278_RXCONT);

	xTaskCreate(sx1278_rx_task, "sx1278_rx_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, NULL);
}
// End

//=============================================================================

//====================== [End Document] =======================================
