/**
 * @file nets_application.h
 * @author
 * @date
 * @brief
 */

#ifndef _NETS_APPLICATION_H_
#define _NETS_APPLICATION_H_

//=============================================================================
// [Inclusions] ===============================================================

#include "esp_netif.h"
#include "esp_wifi_types.h"
#include "esp_netif_types.h"

//=============================================================================

//=============================================================================
// [External Defines] =========================================================

#define DEFAULT_SCAN_LIST_SIZE	16

//=============================================================================

//=============================================================================
// [External Typedef] =========================================================

typedef struct
{
    bool connected;
} wifi_state_t;

typedef struct
{
    bool static_ip;
    char ip[16];
    char netmask[16];
    char gateway[16];
    char ssid[32];
    char pass[32];
} wifi_configs_t;

typedef struct
{
	uint16_t ap_count;
	wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
} wifi_scan_result_t;

typedef struct
{
    bool connected;
} eth_state_t;

typedef struct
{
    bool static_ip;
    char ip[16];
    char netmask[16];
    char gateway[16];
} eth_config_t;

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
// [External Function Declarations] ===========================================

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void nets_init(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void wifi_init(void);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
extern bool wifi_is_ssid_setting(void);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
wifi_state_t wifi_get_state(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
wifi_configs_t wifi_get_config(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
wifi_scan_result_t wifi_get_scan_nets(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void wifi_set_static_ip(bool static_ip);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void wifi_set_ip(char * ip);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void wifi_set_netmask(char * netmask);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void wifi_set_gateway(char * gateway);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void wifi_set_ssid(char * ssid);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void wifi_set_pass(char * pass);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
ip_event_got_ip_t wifi_get_IPinfo(void);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
wifi_ap_record_t wifi_get_APinfo(void);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void eth_init(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
eth_state_t eth_get_state(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
eth_config_t eth_get_config(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void eth_set_static_ip(bool static_ip);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void eth_set_ip(char * ip);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void eth_set_netmask(char * netmask);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void eth_set_gateway(char * gateway);

//=============================================================================

#endif /* _NETS_APPLICATION_H_ */
