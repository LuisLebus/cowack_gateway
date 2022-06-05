/**
 * @file lora.h
 * @author
 * @date
 * @brief
 */

#ifndef _LORA_H_
#define _LORA_H_

//=============================================================================
// [Inclusions] ===============================================================
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

//=============================================================================

//=============================================================================
// [External Defines] =========================================================
#define LORA_COMPANY_ID         0xB23FF1E1

#define LORA_DEV_TYPE_SENSORS   0x00
#define LORA_DEV_TYPE_GATEWAY   0x01

#define LORA_DEV_NAME_C700      0x00

#define LORA_MSG_DATA           0x00
#define LORA_MSG_EVENT_MIC      0x01
#define LORA_MSG_EVENT_ACCEL    0x02

//=============================================================================

//=============================================================================
// [External Typedef] =========================================================
typedef struct __attribute__ ((packed))
{
    uint32_t company_id;
    uint8_t destination_addr[6];
    uint8_t sourse_addr[6];
    uint8_t dev_name;
    uint8_t dev_type;
    uint8_t msg_type;
    union
    {
        uint8_t msg[24];
        struct
        {
            float_t latitude;
            float_t longitude;
            uint8_t sats;
            uint8_t speed;
            uint16_t course;
            int16_t x;              ///< X acceleration in unspecified units
            int16_t y;              ///< Y acceleration in unspecified units
            int16_t z;              ///< Z acceleration in unspecified units
            int16_t ta;             ///< Temperature amb
            int16_t to;             ///< Temperature obj
            uint16_t bat;           ///< Battery
        };
    };
} lora_packet_t;

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
void lora_init(bool enable_rx);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void lora_write(lora_packet_t * lora_packet);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void lora_enter_critical(void);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void lora_exit_critical(void);


//=============================================================================

#endif /* _LORA_H_ */
