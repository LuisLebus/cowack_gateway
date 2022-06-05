/**
 * @file nets_application.c
 * @author
 * @date
 * @brief
 */

//=============================================================================
// [Inclusions] ===============================================================
#include <string.h>

#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "enc28j60.h"

#include <lwip/sockets.h>

#include "nets_application.h"
#include "nvs_application.h"
#include "nvs_keys.h"
#include "device.h"

//=============================================================================

//=============================================================================
// [Private Defines] ==========================================================
#define NETS_TAG        "[NETS]"

#define WIFI_AP_PASS    ""

#define ETH_ENC28J60_SPI_HOST       SPI3_HOST
#define ETH_ENC28J60_SCLK_GPIO      GPIO_NUM_14
#define ETH_ENC28J60_MOSI_GPIO      GPIO_NUM_25
#define ETH_ENC28J60_MISO_GPIO      GPIO_NUM_32
#define ETH_ENC28J60_CS_GPIO        GPIO_NUM_26
#define ETH_ENC28J60_INT_GPIO       GPIO_NUM_33
#define ETH_ENC28J60_SPI_CLOCK_MHZ  6

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

// WiFi scan vars
wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE] = {0};
uint16_t ap_count = 0;

static wifi_configs_t wifi_config = {
		.ssid = "",
		.pass = ""
};
static wifi_state_t wifi_state = {0};
static eth_config_t eth_config = {0};
static eth_state_t eth_state = {0};

//----------------------------------------------------

// TypeDefs -----------------------------------------

static ip_event_got_ip_t ip_event_got_ip = {0};

//---------------------------------------------------

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
// [Local Function Declarations] ==============================================

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void wifi_read_config(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void eth_read_config(void);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void eth_event_handler(void * arg, esp_event_base_t event_base, int32_t event_id, void * event_data);


/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
static void wifi_event_handler(void * arg, esp_event_base_t event_base, int32_t event_id, void * event_data);

//=============================================================================

//=============================================================================
// [FreeRTOS Task Definitions] ================================================

//=============================================================================

//=============================================================================
// [Local Function Definitions] ===============================================

static void wifi_read_config(void)
{
    nvs_read_bool(NVS_KEY_WIFI_IP_TYPE, &wifi_config.static_ip);
    nvs_read_string(NVS_KEY_WIFI_IP, wifi_config.ip);
    nvs_read_string(NVS_KEY_WIFI_NETMASK, wifi_config.netmask);
    nvs_read_string(NVS_KEY_WIFI_GATEWAY, wifi_config.gateway);
    nvs_read_string(NVS_KEY_WIFI_SSID, wifi_config.ssid);
    nvs_read_string(NVS_KEY_WIFI_PASS, wifi_config.pass);
}
// End static void wifi_read_config(void)


static void eth_read_config(void)
{
    nvs_read_bool(NVS_KEY_ETH_IP_TYPE, &eth_config.static_ip);
    nvs_read_string(NVS_KEY_ETH_IP, eth_config.ip);
    nvs_read_string(NVS_KEY_ETH_NETMASK, eth_config.netmask);
    nvs_read_string(NVS_KEY_ETH_GATEWAY, eth_config.gateway);
}
// End static void eth_read_config(void)


static void eth_event_handler(void * arg, esp_event_base_t event_base, int32_t event_id, void * event_data)
{
    // ip_event_got_ip_t ip_event_got_ip = {0};

    if (event_base == ETH_EVENT)
    {
        switch (event_id)
        {
            case ETHERNET_EVENT_CONNECTED:
                ESP_LOGI(NETS_TAG, "ETHERNET_EVENT_CONNECTED");
            break;

            case ETHERNET_EVENT_DISCONNECTED:
                ESP_LOGI(NETS_TAG, "ETHERNET_EVENT_DISCONNECTED");
                eth_state.connected = false;

                if (true == eth_config.static_ip)
                {
                    strcpy(eth_config.ip, "");
                    strcpy(eth_config.netmask, "");
                    strcpy(eth_config.gateway, "");
                }
            break;

            case ETHERNET_EVENT_START:
                ESP_LOGI(NETS_TAG, "ETHERNET_EVENT_START");
            break;

            case ETHERNET_EVENT_STOP:
                ESP_LOGI(NETS_TAG, "ETHERNET_EVENT_STOP");
            break;

            default:
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
            case IP_EVENT_ETH_GOT_IP:
                ESP_LOGI(NETS_TAG, "IP_EVENT_ETH_GOT_IP");
                eth_state.connected = true;

                if (true == eth_config.static_ip)
                {
                    ip_event_got_ip = *(ip_event_got_ip_t *)event_data;

                    strcpy(eth_config.ip, ip4addr_ntoa( (ip4_addr_t*)&ip_event_got_ip.ip_info.ip) );
                    strcpy(eth_config.netmask, ip4addr_ntoa( (ip4_addr_t*)&ip_event_got_ip.ip_info.netmask) );
                    strcpy(eth_config.gateway, ip4addr_ntoa( (ip4_addr_t*)&ip_event_got_ip.ip_info.gw) );
                }
            break;

            default:
            break;
        }
    }
    else
    {
        //Nothing to do.
    }
}
// End static void eth_event_handler(void * arg, esp_event_base_t event_base, int32_t event_id, void * event_data)


static void wifi_event_handler(void * arg, esp_event_base_t event_base, int32_t event_id, void * event_data)
{
    ip_event_got_ip_t ip_event_got_ip = {0};
	uint16_t number = DEFAULT_SCAN_LIST_SIZE;

	if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_STA_START");
				// Scan wifi nets
				esp_wifi_scan_start(NULL, false);
			break;

            case WIFI_EVENT_STA_STOP:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_STA_STOP");
            break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_STA_CONNECTED");
            break;

            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_STA_DISCONNECTED");
                wifi_state.connected = false;

                if (true == wifi_config.static_ip)
                {
                    strcpy(wifi_config.ip, "");
                    strcpy(wifi_config.netmask, "");
                    strcpy(wifi_config.gateway, "");
                }

                vTaskDelay(pdMS_TO_TICKS(5000));
                esp_wifi_connect();
            break;

            case WIFI_EVENT_AP_START:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_AP_START");
            break;

            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_AP_STOP");
            break;

            case WIFI_EVENT_AP_STACONNECTED:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_AP_STACONNECTED");
            break;

            case WIFI_EVENT_AP_STADISCONNECTED:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_AP_STADISCONNECTED");
            break;

			case WIFI_EVENT_SCAN_DONE:
                ESP_LOGI(NETS_TAG, "WIFI_EVENT_SCAN_DONE");
				ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
				ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
				ESP_LOGI(NETS_TAG, "Total APs scanned = %u", ap_count);

                esp_wifi_connect();
				break;

            default:
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch(event_id)
        {
            case IP_EVENT_STA_GOT_IP:
                ESP_LOGI(NETS_TAG, "IP_EVENT_STA_GOT_IP");
                wifi_state.connected = true;

                if (wifi_config.static_ip == false)
                {
                    ip_event_got_ip = *(ip_event_got_ip_t*)event_data;

                    strcpy(wifi_config.ip, ip4addr_ntoa( (ip4_addr_t*)&ip_event_got_ip.ip_info.ip) );
                    strcpy(wifi_config.netmask, ip4addr_ntoa( (ip4_addr_t*)&ip_event_got_ip.ip_info.netmask) );
                    strcpy(wifi_config.gateway, ip4addr_ntoa( (ip4_addr_t*)&ip_event_got_ip.ip_info.gw) );
                }
            break;

            default:
            break;
        }
    }
    else
    {
        //Nothing to do.
    }
}
// End static void wifi_event_handler(void * arg, esp_event_base_t event_base, int32_t event_id, void * event_data)

//=============================================================================

//=============================================================================
// [External Function Definition] =============================================

void nets_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
}
// End void nets_init(void)


void wifi_init(void)
{
    wifi_read_config();

    esp_netif_t * sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    if (true == wifi_config.static_ip)
    {
        esp_netif_dhcpc_stop(sta_netif);

        esp_netif_ip_info_t ip_info = {0};
        inet_pton(AF_INET, wifi_config.ip, &ip_info.ip);
        inet_pton(AF_INET, wifi_config.gateway, &ip_info.gw);
        inet_pton(AF_INET, wifi_config.netmask, &ip_info.netmask);
        esp_netif_set_ip_info(sta_netif, &ip_info);

        esp_netif_dns_info_t dns_info = {0};
        inet_pton(AF_INET, "8.8.8.8", &dns_info.ip);
        esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns_info);
    }

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_config);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_sta_config = {0};
    strncpy((char *)wifi_sta_config.sta.ssid, wifi_config.ssid, sizeof(wifi_sta_config.sta.ssid));
    strncpy((char *)wifi_sta_config.sta.password, wifi_config.pass, sizeof(wifi_sta_config.sta.password));

    device_config_t device_config = device_get_config();

    if ((device_config.model != NULL) && (device_config.sn != NULL))
    {
        wifi_config_t wifi_ap_config = {0};
        strcpy((char *)wifi_ap_config.ap.ssid, device_config.model);
        strcat((char *)wifi_ap_config.ap.ssid, "-");
        strcat((char *)wifi_ap_config.ap.ssid, device_config.sn);
        strncpy((char *)wifi_ap_config.ap.password, WIFI_AP_PASS, sizeof(wifi_ap_config.ap.password));
        wifi_ap_config.ap.max_connection = 1;
        wifi_ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

        esp_wifi_set_mode(WIFI_MODE_APSTA);
        esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_ap_config);
    }
    else
    {
        esp_wifi_set_mode(WIFI_MODE_STA);
    }

    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config);
    esp_wifi_start();
}
// End void wifi_init(void)


extern bool wifi_is_ssid_setting(void)
{
	bool result = nvs_read_string(NVS_KEY_WIFI_SSID, wifi_config.ssid);

	return result;
}
// End extern bool wifi_is_ssid_setting(void)


wifi_configs_t wifi_get_config(void)
{
    return wifi_config;
}
// End wifi_configs_t wifi_get_config(void)


wifi_scan_result_t wifi_get_scan_nets(void)
{
	wifi_scan_result_t scan_result = {0};

	scan_result.ap_count = ap_count;

	for (uint8_t i = 0; i < ap_count; i++)
	{
		memcpy(scan_result.ap_info[i].ssid, ap_info[i].ssid, sizeof(ap_info[i].ssid));
		scan_result.ap_info[i].rssi = ap_info[i].rssi;
	}

	return scan_result;

}
// End wifi_scan_result_t wifi_get_scan_nets(void)


wifi_state_t wifi_get_state(void)
{
    return wifi_state;
}
// End wifi_state_t wifi_get_state(void)


void wifi_set_static_ip(bool static_ip)
{
    if (nvs_write_bool(NVS_KEY_WIFI_IP_TYPE, static_ip))
    {
        wifi_config.static_ip = static_ip;
    }
}
// End void wifi_set_stattic_ip(bool static_ip)


void wifi_set_ip(char * ip)
{
    if (nvs_write_string(NVS_KEY_WIFI_IP, ip))
    {
        strncpy(wifi_config.ip, ip, sizeof(wifi_config.ip));
    }
}
// End void wifi_set_ip(char * ip)


void wifi_set_netmask(char * netmask)
{
    if (nvs_write_string(NVS_KEY_WIFI_NETMASK, netmask))
    {
        strncpy(wifi_config.netmask, netmask, sizeof(wifi_config.netmask));
    }
}
// End void wifi_set_netmask(char * netmask)


void wifi_set_gateway(char * gateway)
{
    if (nvs_write_string(NVS_KEY_WIFI_GATEWAY, gateway))
    {
        strncpy(wifi_config.gateway, gateway, sizeof(wifi_config.gateway));
    }
}
// End void wifi_set_gateway(char * gateway)


void wifi_set_ssid(char * ssid)
{
    if (nvs_write_string(NVS_KEY_WIFI_SSID, ssid))
    {
        strncpy(wifi_config.ssid, ssid, sizeof(wifi_config.ssid));
    }
}
// End void wifi_set_ssid(char * ssid)


void wifi_set_pass(char * pass)
{
    if (nvs_write_string(NVS_KEY_WIFI_PASS, pass))
    {
        strncpy(wifi_config.pass, pass, sizeof(wifi_config.pass));
    }
}
// End void wifi_set_pass(char * pass)


ip_event_got_ip_t wifi_get_IPinfo(void){

    /* Local data declaration */

    /* Local data initialization */

    /* Return AP info */
    return ip_event_got_ip;
}
// End extern wifi_ap_record_t wifi_get_APinfo(void)


wifi_ap_record_t wifi_get_APinfo(void){

    /* Local data declaration */
    wifi_ap_record_t ap_info;

    /* Local data initialization */
    esp_wifi_sta_get_ap_info(&ap_info);

    /* Return AP info */
    return ap_info;
}
// End extern wifi_ap_record_t wifi_get_APinfo(void)

void eth_init(void)
{
    eth_read_config();

    gpio_install_isr_service(0);

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t* eth_netif = esp_netif_new(&netif_cfg);

    // Set default handlers to process TCP/IP stuffs
    esp_eth_set_default_handlers(eth_netif);

    if (true == eth_config.static_ip)
    {
        esp_netif_dhcpc_stop(eth_netif);

        esp_netif_ip_info_t ip_info = {0};
        inet_pton(AF_INET, eth_config.ip, &ip_info.ip);
        inet_pton(AF_INET, eth_config.gateway, &ip_info.gw);
        inet_pton(AF_INET, eth_config.netmask, &ip_info.netmask);
        esp_netif_set_ip_info(eth_netif, &ip_info);

        esp_netif_dns_info_t dns_info = {0};
        inet_pton(AF_INET, "8.8.8.8", &dns_info.ip);
        esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_MAIN, &dns_info);
        esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_BACKUP, &dns_info);
        esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_FALLBACK, &dns_info);
    }

    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_event_handler, NULL);

    spi_bus_config_t buscfg = {
        .miso_io_num = ETH_ENC28J60_MISO_GPIO,
        .mosi_io_num = ETH_ENC28J60_MOSI_GPIO,
        .sclk_io_num = ETH_ENC28J60_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(ETH_ENC28J60_SPI_HOST, &buscfg, 1);

    /* ENC28J60 ethernet driver is based on spi driver */
    spi_device_interface_config_t devcfg = {
        .command_bits = 3,
        .address_bits = 5,
        .mode = 0,
        .clock_speed_hz = ETH_ENC28J60_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = ETH_ENC28J60_CS_GPIO,
        .queue_size = 20
    };

    spi_device_handle_t spi_handle = NULL;
    spi_bus_add_device(ETH_ENC28J60_SPI_HOST, &devcfg, &spi_handle);

    eth_enc28j60_config_t enc28j60_config = ETH_ENC28J60_DEFAULT_CONFIG(spi_handle);
    enc28j60_config.int_gpio_num = ETH_ENC28J60_INT_GPIO;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.smi_mdc_gpio_num = -1;       // ENC28J60 doesn't have SMI interface
    mac_config.smi_mdio_gpio_num = -1;
    esp_eth_mac_t * mac = esp_eth_mac_new_enc28j60(&enc28j60_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.autonego_timeout_ms = 0;     // ENC28J60 doesn't support auto-negotiation
    phy_config.reset_gpio_num = -1;         // ENC28J60 doesn't have a pin to reset internal PHY
    esp_eth_phy_t * phy = esp_eth_phy_new_enc28j60(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    esp_eth_driver_install(&eth_config, &eth_handle);

    uint8_t mac_addr[6] = {0};
    esp_read_mac(mac_addr, ESP_MAC_ETH);
    mac->set_addr(mac, (uint8_t[]) {
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]
    });

    /* attach Ethernet driver to TCP/IP stack */
    esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle));
    /* start Ethernet driver state machine */
    esp_eth_start(eth_handle);
}
// End void eth_init(void)


eth_config_t eth_get_config(void)
{
    return eth_config;
}
// End eth_config_t eth_get_config(void)


eth_state_t eth_get_state(void)
{
    return eth_state;
}
// End eth_state_t eth_get_state(void)


void eth_set_static_ip(bool static_ip)
{
    if (nvs_write_bool(NVS_KEY_ETH_IP_TYPE, static_ip))
    {
        eth_config.static_ip = static_ip;
    }
}
// End void eth_set_static_ip(bool static_ip)


void eth_set_ip(char * ip)
{
    if (nvs_write_string(NVS_KEY_ETH_IP, ip))
    {
        strncpy(eth_config.ip, ip, sizeof(eth_config.ip));
    }
}
// End void eth_set_ip(char * ip)


void eth_set_netmask(char * netmask)
{
    if (nvs_write_string(NVS_KEY_ETH_NETMASK, netmask))
    {
        strncpy(eth_config.netmask, netmask, sizeof(eth_config.netmask));
    }
}
// End void eth_set_netmask(char * netmask)


void eth_set_gateway(char * gateway)
{
    if (nvs_write_string(NVS_KEY_ETH_GATEWAY, gateway))
    {
        strncpy(eth_config.gateway, gateway, sizeof(eth_config.gateway));
    }
}
// End void eth_set_gateway(char * gateway)

//=============================================================================

//====================== [End Document] =======================================
