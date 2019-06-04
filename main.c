/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "nrf_delay.h"

#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"

#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "app_usbd_core.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"


unsigned char test[] = "Ich bin Ramon Loher \n";
/*===============================================================================================================
*								BLUETOOTH LOW ENERGY CENTRAL DEFINITIONS
================================================================================================================*/

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  0	                                    /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

APP_TIMER_DEF(m_blink_ble);

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
    .type = NUS_SERVICE_UUID_TYPE,
    .uuid = BLE_UUID_NUS_SERVICE
};

static bool connectedF = false;
/*===============================================================================================================
*								  END OF BLUETOOTH LOW ENERGY CENTRAL DEFINITIONS
================================================================================================================*/





/*===============================================================================================================
*										BEGIN OF USBD CDC ACM DEFINITIONS
================================================================================================================*/

#define BTN_CDC_DATA_SEND       0
#define BTN_CDC_NOTIFY_SEND     1
#define BTN_CDC_DATA_KEY_RELEASE        (bsp_event_t)(BSP_EVENT_KEY_LAST + 1)

#define LED_BLINK_INTERVAL 800
bool USBD_STARTED_FLAG = false;
/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

//creating a CDC class instance
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

uint8_t m_cdc_data_array[BLE_NUS_MAX_DATA_LEN];
/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
							APP_USBD_CDC_COMM_PROTOCOL_VENDOR
);

APP_TIMER_DEF(m_blink_cdc);


/*===============================================================================================================
*										END OF USBD CDC ACM DEFINITIONS
================================================================================================================*/


//declaration
void blink_handler(void * p_context);

/*===============================================================================================================
*
*								BEGIN OF BLUETOOTH LOW ENERGY CENTRAL FUNCTIONS
*
================================================================================================================*/

/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = app_timer_start(m_blink_ble,
                                          APP_TIMER_TICKS(LED_BLINK_INTERVAL),
                                          (void *) LEDG2);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
    SEGGER_RTT_WriteString(0,"scan_start() \n");
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         	 bsp_board_led_off(LEDG2);
              SEGGER_RTT_WriteString(0,"NRF_BLE_SCAN_EVT_CONNECTING_ERROR \n");
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
        	 app_timer_stop(m_blink_ble);
        	 bsp_board_led_on(LEDG2);
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
              SEGGER_RTT_printf(0,"Connecting to target %02x%02x%02x%02x%02x%02x \n",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         }
         SEGGER_RTT_WriteString(0,"NRF_BLE_SCAN_EVT_CONNECTED \n");
         break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
        	 bsp_board_led_off(LEDG2);
             scan_start();
             SEGGER_RTT_WriteString(0,"NRF_BLE_SCAN_EVT_SCAN_TIMEOUT \n");
         } break;

         default:
             break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);

    SEGGER_RTT_WriteString(0,"scan_init() \n");
}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val = NRF_SUCCESS;
    SEGGER_RTT_WriteString(0,"ble_nus_chars_received() \n");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);


	for (uint32_t i = 0; i < data_len; i++)
	{
//		do
//		{
//			ret_val = app_usbd_cdc_acm_write(&m_app_cdc_acm, p_data, data_len);
//			if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
//			{
//				SEGGER_RTT_printf(0,"app_uart_put failed for index 0x%04x.\n", i);
//				APP_ERROR_CHECK(ret_val);
//			}
//		} while (ret_val == NRF_ERROR_BUSY);
	}

	if (ECHOBACK_BLE_UART_DATA)// ECHO ===> sends received data back to transmitter
	{
		// Send data back to the peripheral.
		do
		{
			ret_val = ble_nus_c_string_send(&m_ble_nus_c, test, sizeof(test));
			if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
			{
				SEGGER_RTT_printf(0,"Failed sending NUS message. Error 0x%x. \n", ret_val);
				APP_ERROR_CHECK(ret_val);
			}
		} while (ret_val == NRF_ERROR_BUSY);
	}

    SEGGER_RTT_WriteString(0,"ble_nus_chars_received_uart_print() \n");
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;
    SEGGER_RTT_WriteString(0,"ble_nus_c_evt_handler() \n");
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
        	SEGGER_RTT_WriteString(0,"Discovery complete. \n");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"Connected to device with Nordic UART Service.\n");
            SEGGER_RTT_WriteString(0,"BLE_NUS_C_EVT_DISCOVERY_COMPLETE \n");
            break;

        case BLE_NUS_C_EVT_NUS_RX_EVT:
            SEGGER_RTT_WriteString(0,"BLE_NUS_C_EVT_NUS_RX_EVT \n");
            ble_nus_chars_received(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            scan_start();
            SEGGER_RTT_WriteString(0,"BLE_NUS_C_EVT_DISCONNECTED \n");
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    SEGGER_RTT_WriteString(0,"ble_evt_handler() \n");
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            connectedF = true;
            bsp_board_led_on(LEDG2);
            SEGGER_RTT_WriteString(0,"BLE_GAP_EVT_CONNECTED \n");
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        	bsp_board_led_off(LEDG2);
        	SEGGER_RTT_printf(0,"Disconnected. conn_handle: 0x%x, reason: 0x%x \n",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
        	connectedF = false;
            SEGGER_RTT_WriteString(0,"BLE_GAP_EVT_DISCONNECTED \n");
            break;

        case BLE_GAP_EVT_ADV_REPORT:
                {


                }
            break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
            	SEGGER_RTT_WriteString(0,"Connection Request timed out.");
            }
            SEGGER_RTT_WriteString(0,"BLE_GAP_EVT_TIMEOUT \n");
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"BLE_GAP_EVT_SEC_PARAMS_REQUEST \n");
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST \n");
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
            SEGGER_RTT_WriteString(0,"BLE_GAP_EVT_PHY_UPDATE_REQUEST \n");
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"BLE_GATTS_EVT_TIMEOUT \n");
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"BLE_GATTS_EVT_TIMEOUT \n");
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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
    SEGGER_RTT_WriteString(0,"ble_stack_init() \n");
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    SEGGER_RTT_WriteString(0,"gatt_evt_handler() \n");
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
    	SEGGER_RTT_WriteString(0,"ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        SEGGER_RTT_printf(0,"Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }

}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    SEGGER_RTT_WriteString(0,"gatt_init() \n");
}


/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}


/*===============================================================================================================
*
*								END OF BLUETOOTH LOW ENERGY CENTRAL FUNCTIONS
*
================================================================================================================*/



/*===============================================================================================================
*
*									  BEGIN OF USBD CDC ACM FUNCTIONS
*
================================================================================================================*/

static bool m_usb_connected = false;


/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    ret_code_t ret;
    uint8_t rxSize = 0;
    switch (event)
        {
            case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            {
                /*Set up the first transfer*/
                USBD_STARTED_FLAG = true;
                ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                       m_cdc_data_array,
                                                       1);
                UNUSED_VARIABLE(ret);
                ret = app_timer_stop(m_blink_cdc);
                APP_ERROR_CHECK(ret);
            	SEGGER_RTT_WriteString(0,"APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN \n");
                break;
            }

            case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
                USBD_STARTED_FLAG = false;
            	SEGGER_RTT_WriteString(0,"APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE \n");
                if (m_usb_connected)
                {
                    ret_code_t ret = app_timer_start(m_blink_cdc,
                                                     APP_TIMER_TICKS(LED_BLINK_INTERVAL),
                                                     (void *) LEDG1);
                    APP_ERROR_CHECK(ret);
                }
                break;

            case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            	SEGGER_RTT_WriteString(0,"APP_USBD_CDC_ACM_USER_EVT_TX_DONE \n");
                break;

            case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
            {
				rxSize = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
				do
				{
					ret = ble_nus_c_string_send(&m_ble_nus_c, m_cdc_data_array, rxSize);
					if ((ret != NRF_SUCCESS) && (ret != NRF_ERROR_BUSY))
					{
						SEGGER_RTT_printf(0,"Failed sending NUS message. Error 0x%x. \n", ret);
						APP_ERROR_CHECK(ret);
					}
				} while (ret == NRF_ERROR_BUSY);

            	SEGGER_RTT_WriteString(0,"APP_USBD_CDC_ACM_USER_EVT_RX_DONE \n");
            	static uint8_t index = 0;
            	index++;

				break;
            }


            default:
                break;
        }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
	ret_code_t err_code;
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_led_off(LEDG1);
            SEGGER_RTT_WriteString(0,"APP_USBD_EVT_DRV_SUSPEND \n");
            break;

        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LEDG1);
            SEGGER_RTT_WriteString(0,"APP_USBD_EVT_DRV_RESUME \n");
            break;

        case APP_USBD_EVT_STARTED:
            SEGGER_RTT_WriteString(0,"APP_USBD_EVT_STARTED \n");
            break;

        case APP_USBD_EVT_STOPPED:
            bsp_board_led_off(LEDG1);
            app_usbd_disable();
            SEGGER_RTT_WriteString(0,"APP_USBD_EVT_STOPPED \n");
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            SEGGER_RTT_WriteString(0,"APP_USBD_EVT_POWER_DETECTED \n");
            break;

        case APP_USBD_EVT_POWER_REMOVED:
            m_usb_connected = false;
            SEGGER_RTT_WriteString(0,"APP_USBD_EVT_POWER_REMOVED \n");
            err_code = app_timer_stop(m_blink_cdc);
            APP_ERROR_CHECK(err_code);
            bsp_board_led_off(LEDG1);
            app_usbd_stop();
            break;

        case APP_USBD_EVT_POWER_READY:
        	SEGGER_RTT_WriteString(0,"APP_USBD_EVT_POWER_READY\n");
            err_code = app_timer_start(m_blink_cdc,
                                                  APP_TIMER_TICKS(LED_BLINK_INTERVAL),
                                                  (void *) LEDG1);
            APP_ERROR_CHECK(err_code);
            m_usb_connected = true;
            app_usbd_start();
            break;
        default:
            break;
    }
}

/*===============================================================================================================
*
*									  END OF USBD CDC ACM FUNCTIONS
*
================================================================================================================*/



/*===============================================================================================================
*
*									  BEGIN OF OTHER FUNCTIONS
*
================================================================================================================*/

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
    SEGGER_RTT_WriteString(0,"assert_nrf_callback() \n");
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
    SEGGER_RTT_WriteString(0,"db_disc_handler() \n");
}


/*===============================================================================================================
*		Hier war der UART HAndler Implementiert. Die übriggebliebene FUnktion war Teil davon.
================================================================================================================*/
/*
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];

                    ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
                    if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
                    {
                        APP_ERROR_CHECK(ret_val);
                    }
*/
/*===============================================================================================================
*										END OF UART EVENT HANDLER
================================================================================================================*/



/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_WriteString(0,"NRF_PWR_MGMT_EVT_PREPARE_WAKEUP \n");
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);



/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            SEGGER_RTT_WriteString(0,"BSP_EVENT_SLEEP \n");
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            SEGGER_RTT_WriteString(0,"BSP_EVENT_DISCONNECT\n");
            break;

        default:
            break;
    }
}





/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_blink_ble, APP_TIMER_MODE_REPEATED, blink_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_blink_cdc, APP_TIMER_MODE_REPEATED, blink_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
	SEGGER_RTT_Init();
    SEGGER_RTT_WriteString(0,"HELLO RAMON");
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**
 * @brief App timer handler for blinking the LEDs.
 *
 * @param p_context LED to blink.
 */
void blink_handler(void * p_context)
{
    bsp_board_led_invert((uint32_t) p_context);
}
/*============================================================================================================
 *
 * 												END OF OTHER FUNCTIONS
 *
 =============================================================================================================*/

void nvmcErase(void)
{
  if (NRF_UICR->REGOUT0 != UICR_REGOUT0_VOUT_3V3)
  {
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;        //erase enable
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
      NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase << NVMC_ERASEUICR_ERASEUICR_Pos;   //erase UICR
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;  //read only enable
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
  }
}


/**
 * @brief Function for configuration of REGOUT0 register.
 */
void vddInit(void)
{
  if (NRF_UICR->REGOUT0 != UICR_REGOUT0_VOUT_3V3)
  {
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;    //write enable
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
	NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_3V3;                        //configurate REGOUT0
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NVIC_SystemReset();                                               // Reset device
  }
}

// ACHTUNG : bei längeren delays wird der USB nicht mehr von WIndows erkannt !!!!!!!!!
//nrf_delay_ms(100);	// Funktioniert noch
int main(void)
{
    nvmcErase();
    vddInit();
	uint8_t arr[30] = {0xFF, 0x00, 0xFF, 0x52, 0xAD, 0x55, 0x33, 0xFF, 0x00, 0xFF,
					0xFF, 0x00, 0xFF, 0x52, 0xAD, 0x55, 0x33, 0xFF, 0x00, 0xFF,
					0xFF, 0x00, 0xFF, 0x52, 0xAD, 0x55, 0x33, 0xFF, 0x00, 0xFF};


    ret_code_t ret = NRF_SUCCESS;
    uint32_t ret_value = NRF_SUCCESS;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    log_init();

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);

    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    timer_init();
    buttons_leds_init();
    db_discovery_init();
    power_management_init();

    app_usbd_serial_num_generate();

//    The created instance is added to the USBD library
    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);


   	ble_stack_init();
	gatt_init();
	nus_c_init();
	scan_init();

    scan_start();

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    // Enter main loop.
//sdfsdfs
	for (;;)
	{
		while (app_usbd_event_queue_process())
		{
			while(connectedF)
			{
				//idle_state_handle();
			}
		}
	}
}
