/**
 * @file lora.c
 * @author
 * @date
 * @brief
 */

//=============================================================================
// [Inclusions] ===============================================================
#include "lora.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "mqtt_client.h"
#include "esp_log.h"
#include "mbedtls/aes.h"

#include "nvs_application.h"
#include "mqtt_application.h"
#include "nvs_keys.h"
#include "sx1278.h"

#include "cJSON.h"

//=============================================================================

//=============================================================================
// [Private Defines] ==========================================================
#define LORA_TAG        		"[LORA]"

#define LORA_DEFAULT_FREQUENCY	915.0
#define LORA_DEFAULT_BOOST_ON  	true
#define LORA_DEFAULT_POWER     	4

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
static uint8_t destination_addr[6] = {0x7C, 0x9E, 0xBD, 0x49, 0x50, 0x80};
static uint8_t sourse_addr[6] = {0};

static unsigned char iv[] = {
        0xCD, 0x32, 0x87, 0x1D, 0x15, 0xDD, 0xAA, 0x55,
        0xB3, 0xA7, 0xAC, 0x0B, 0x3E, 0x90, 0x06, 0x0F
};

mbedtls_aes_context aes = {0};

// key length 32 bytes for 256 bit encrypting
unsigned char key[] = {
        0xFF, 0x03, 0xCD, 0x31, 0x84, 0x05, 0xA6, 0x55,
        0xC8, 0x87, 0x32, 0x72, 0x4C, 0x17, 0xB4, 0xBA,
        0xEA, 0x2B, 0x78, 0x26, 0x99, 0x68, 0x39, 0x12,
        0x57, 0x3E, 0xBF, 0x0B, 0x2D, 0x2A, 0xE8, 0x0F
};

//----------------------------------------------------

// Task Handlers -------------------------------------
//----------------------------------------------------

// Queue Handlers ------------------------------------
//----------------------------------------------------

// Event Group Handlers ------------------------------
//----------------------------------------------------

// Semaphore Handlers --------------------------------
static SemaphoreHandle_t lora_mutex = {0};

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
static void lora_rx_task(void * pvParameter);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_encrypt_data(uint8_t * data_decrypted, uint8_t * data_encrypted);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void sx1278_decrypt_data(uint8_t * data_decrypted, uint8_t * data_encrypted);

//=============================================================================

//=============================================================================
// [FreeRTOS Task Definitions] ================================================

static void lora_rx_task(void * pvParameter)
{
    uint8_t sx1278_data_encrypted[SX1278_BUFF_SIZE] = {0};
    uint8_t sx1278_data_decrypted[SX1278_BUFF_SIZE] = {0};
    lora_packet_t lora_packet = {0};

    char topic[64] = {0};
	char * payload = NULL;
	cJSON * root = NULL;

	 vTaskDelay(pdMS_TO_TICKS(15000));

    while(1)
    {
        if (true == sx1278_read(sx1278_data_encrypted))
        {
            sx1278_decrypt_data(sx1278_data_decrypted, sx1278_data_encrypted);

            memcpy(&lora_packet, sx1278_data_decrypted, sizeof(lora_packet_t));

            ESP_LOGI(LORA_TAG, "================================================");
            ESP_LOGI(LORA_TAG, "company_id: %x", lora_packet.company_id);
            ESP_LOGI(LORA_TAG, "destination_addr: %02x:%02x:%02x:%02x:%02x:%02x",
            		lora_packet.destination_addr[5], lora_packet.destination_addr[4],
					lora_packet.destination_addr[3], lora_packet.destination_addr[2],
					lora_packet.destination_addr[1], lora_packet.destination_addr[0]);
            ESP_LOGI(LORA_TAG, "sourse_addr: %02x:%02x:%02x:%02x:%02x:%02x",
            		lora_packet.sourse_addr[5], lora_packet.sourse_addr[4],
					lora_packet.sourse_addr[3], lora_packet.sourse_addr[2],
					lora_packet.sourse_addr[1], lora_packet.sourse_addr[0]);
            ESP_LOGI(LORA_TAG, "dev_name: %u", lora_packet.dev_name);
            ESP_LOGI(LORA_TAG, "dev_type: %u", lora_packet.dev_type);
            ESP_LOGI(LORA_TAG, "msg_type: %u", lora_packet.msg_type);
            ESP_LOGI(LORA_TAG, "latitude: %f", lora_packet.latitude);
            ESP_LOGI(LORA_TAG, "longitude: %f", lora_packet.longitude);
            ESP_LOGI(LORA_TAG, "speed: %u", lora_packet.speed);
            ESP_LOGI(LORA_TAG, "course: %u", lora_packet.course);
            ESP_LOGI(LORA_TAG, "sats: %u", lora_packet.sats);
            ESP_LOGI(LORA_TAG, "ta: %i", lora_packet.ta);
            ESP_LOGI(LORA_TAG, "to: %i", lora_packet.to);
            ESP_LOGI(LORA_TAG, "x: %i", lora_packet.x);
            ESP_LOGI(LORA_TAG, "y: %i", lora_packet.y);
            ESP_LOGI(LORA_TAG, "z: %i", lora_packet.z);
            ESP_LOGI(LORA_TAG, "bat: %u", lora_packet.bat);
            ESP_LOGI(LORA_TAG, "================================================");

            if (LORA_COMPANY_ID == lora_packet.company_id)
            {
                if (0 == memcmp(lora_packet.destination_addr, sourse_addr, sizeof(lora_packet.sourse_addr)))
                {
                	//Topic = "data"
                	root = cJSON_CreateObject();

                	cJSON_AddItemToObject(root, "latitude", cJSON_CreateNumber(lora_packet.latitude));
                	cJSON_AddItemToObject(root, "longitude", cJSON_CreateNumber(lora_packet.longitude));
                	cJSON_AddItemToObject(root, "speed", cJSON_CreateNumber(lora_packet.speed));
                	cJSON_AddItemToObject(root, "course", cJSON_CreateNumber(lora_packet.course));
                	cJSON_AddItemToObject(root, "sats", cJSON_CreateNumber(lora_packet.sats));
                	cJSON_AddItemToObject(root, "ta", cJSON_CreateNumber(lora_packet.ta / 10.0));
                	cJSON_AddItemToObject(root, "to", cJSON_CreateNumber(lora_packet.to / 10.0));
                	cJSON_AddItemToObject(root, "x", cJSON_CreateNumber(lora_packet.x / 10.0));
                	cJSON_AddItemToObject(root, "y", cJSON_CreateNumber(lora_packet.y / 10.0));
                	cJSON_AddItemToObject(root, "z", cJSON_CreateNumber(lora_packet.z / 10.0));
                	cJSON_AddItemToObject(root, "bat", cJSON_CreateNumber(lora_packet.bat / 100.0));

                	payload = cJSON_Print(root);
                	cJSON_Delete(root);

                	memset(topic, 0, sizeof(topic));
                	mqtt_make_topic("data", topic);

                	esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 0, 0);

#if 1
                	ESP_LOGI(LORA_TAG, "Topic: %s", topic);
                	ESP_LOGI(LORA_TAG, "Data: %s", payload);
#endif
                    free(payload);

                    ESP_LOGI(LORA_TAG, "MQTT Sent!");
                }
            }
        }
    }
}
// End

//=============================================================================

//=============================================================================
// [Local Function Definitions] ===============================================

static void sx1278_encrypt_data(uint8_t * data_decrypted, uint8_t * data_encrypted)
{
//    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, SX1278_BUFF_SIZE, iv, data_decrypted, data_encrypted);
	memcpy(data_encrypted, data_decrypted, SX1278_BUFF_SIZE);
}
// End

static void sx1278_decrypt_data(uint8_t * data_decrypted, uint8_t * data_encrypted)
{
//    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, SX1278_BUFF_SIZE, iv, data_encrypted, data_decrypted);
	memcpy(data_decrypted, data_encrypted, SX1278_BUFF_SIZE);
}
// End

//=============================================================================

//=============================================================================
// [External Function Definition] =============================================

void lora_init(bool enable_rx)
{
    sx1278_init();

    sx1278_set_frequency(LORA_DEFAULT_FREQUENCY);
    sx1278_set_boost_on(LORA_DEFAULT_BOOST_ON);
    sx1278_set_power(LORA_DEFAULT_POWER);

    if (true == enable_rx)
	{
    	sx1278_set_rx_enable();
	}

    lora_mutex = xSemaphoreCreateMutex();
    lora_exit_critical();

    esp_read_mac(sourse_addr, ESP_MAC_WIFI_STA);
    ESP_LOGI(LORA_TAG, "sourse_addr: %02x:%02x:%02x:%02x:%02x:%02x",
    		sourse_addr[5], sourse_addr[4],
			sourse_addr[3], sourse_addr[2],
			sourse_addr[1], sourse_addr[0]);

    if (true == enable_rx)
    {
        xTaskCreate(lora_rx_task, "lora_rx_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, NULL);
    }
}
// End

void lora_write(lora_packet_t * lora_packet)
{
    uint8_t sx1278_data_encrypted[SX1278_BUFF_SIZE] = {0};
    uint8_t sx1278_data_decrypted[SX1278_BUFF_SIZE] = {0};

    lora_packet->company_id = LORA_COMPANY_ID;
    memcpy(lora_packet->destination_addr, destination_addr, sizeof(destination_addr));
    memcpy(lora_packet->sourse_addr, sourse_addr, sizeof(sourse_addr));
    lora_packet->dev_name = LORA_DEV_NAME_C700;
    lora_packet->dev_type = LORA_DEV_TYPE_SENSORS;

    ESP_LOGI(LORA_TAG, "================================================");
    ESP_LOGI(LORA_TAG, "company_id: %x", lora_packet->company_id);
    ESP_LOGI(LORA_TAG, "destination_addr: %02x:%02x:%02x:%02x:%02x:%02x",
    		lora_packet->destination_addr[5], lora_packet->destination_addr[4],
			lora_packet->destination_addr[3], lora_packet->destination_addr[2],
			lora_packet->destination_addr[1], lora_packet->destination_addr[0]);
    ESP_LOGI(LORA_TAG, "sourse_addr: %02x:%02x:%02x:%02x:%02x:%02x",
    		lora_packet->sourse_addr[5], lora_packet->sourse_addr[4],
			lora_packet->sourse_addr[3], lora_packet->sourse_addr[2],
			lora_packet->sourse_addr[1], lora_packet->sourse_addr[0]);
    ESP_LOGI(LORA_TAG, "dev_name: %u", lora_packet->dev_name);
    ESP_LOGI(LORA_TAG, "dev_type: %u", lora_packet->dev_type);
	ESP_LOGI(LORA_TAG, "msg_type: %u", lora_packet->msg_type);
	ESP_LOGI(LORA_TAG, "latitude: %f", lora_packet->latitude);
	ESP_LOGI(LORA_TAG, "longitude: %f", lora_packet->longitude);
	ESP_LOGI(LORA_TAG, "speed: %u", lora_packet->speed);
	ESP_LOGI(LORA_TAG, "course: %u", lora_packet->course);
	ESP_LOGI(LORA_TAG, "sats: %u", lora_packet->sats);
	ESP_LOGI(LORA_TAG, "ta: %i", lora_packet->ta);
	ESP_LOGI(LORA_TAG, "to: %i", lora_packet->to);
	ESP_LOGI(LORA_TAG, "x: %i", lora_packet->x);
	ESP_LOGI(LORA_TAG, "y: %i", lora_packet->y);
	ESP_LOGI(LORA_TAG, "z: %i", lora_packet->z);
	ESP_LOGI(LORA_TAG, "bat: %u", lora_packet->bat);
	ESP_LOGI(LORA_TAG, "================================================");

    memcpy(sx1278_data_decrypted, lora_packet, sizeof(lora_packet_t));

    sx1278_encrypt_data(sx1278_data_decrypted, sx1278_data_encrypted);

    sx1278_write(sx1278_data_encrypted);
}
// End

void lora_enter_critical(void)
{
    xSemaphoreTake(lora_mutex, portMAX_DELAY);
}
// End

void lora_exit_critical(void)
{
    xSemaphoreGive(lora_mutex);
}
// End

//=============================================================================

//====================== [End Document] =======================================
