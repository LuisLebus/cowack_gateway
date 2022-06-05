/**
 * @file mqtt_application.h
 * @author
 * @date
 * @brief
 */

#ifndef _MQTT_APPLICATION_H_
#define _MQTT_APPLICATION_H_

//=============================================================================
// [Inclusions] ===============================================================
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "mqtt_client.h"

//=============================================================================

//=============================================================================
// [External Defines] =========================================================

//=============================================================================

//=============================================================================
// [External Typedef] =========================================================

typedef struct
{
    bool connected;
    bool subscribed;
    bool mutex_err;
} mqtt_state_t;


typedef struct
{
    bool server;
    char host[16];
    char user[32];
    char pass[32];
    uint32_t port;
    uint16_t delay;
    uint16_t logs_delay;
} mqtt_config_t;

//=============================================================================

//=============================================================================
// [External Data Definitions] ================================================

// Const ---------------------------------------------
//----------------------------------------------------

// Vars ----------------------------------------------
extern esp_mqtt_client_handle_t mqtt_client;

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
// [External Function Declarations] ===========================================

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_init(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_make_topic(const char * label, char * dest);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
mqtt_config_t mqtt_get_config(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
mqtt_state_t mqtt_get_state(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_set_host(char * host);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_set_user(char * user);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_set_pass(char * pass);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_set_port(uint32_t port);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_set_delay(uint16_t delay);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_set_logs_delay(uint16_t logs_delay);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void mqtt_set_server(bool server);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
extern void mqtt_send_config(void);

//=============================================================================

#endif /* _MQTT_APPLICATION_H_ */
