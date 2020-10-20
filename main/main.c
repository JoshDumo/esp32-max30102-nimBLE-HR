/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "blehr_sens.h"
#include "max30102.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FRQ 100000
#define I2C_PORT I2C_NUM_0

max30102_config_t max30102 = {};

static const char *tag = "MAX30102_NimBLE_BLE_HeartRate";

static xTimerHandle blehr_tx_timer;

static bool notify_state;

static uint16_t conn_handle;

static const char *device_name = "blehr_max30102_1.0";

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;

/* Variable to simulate heart beats */
static uint8_t heartrate = 90;

/**
 * Utility function to log an array of bytes.
 */
void
print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

/**
 * Addition by @author Joshua D JOHN
 * Function to initialize the I2C port connecting the MAX30102
*/

esp_err_t i2c_master_init(i2c_port_t i2c_port){
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.scl_io_num = I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FRQ;
    i2c_param_config(i2c_port, &conf);
    return i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}


/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void
blehr_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blehr_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void
blehr_tx_hrate_stop(void)
{
    xTimerStop( blehr_tx_timer, 1000 / portTICK_PERIOD_MS );
}

/* Reset heart rate measurement */
static void
blehr_tx_hrate_reset(void)
{
    int rc;

    if (xTimerReset(blehr_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
        rc = 0;
    } else {
        rc = 1;
    }

    assert(rc == 0);

}

/* This function simulates heart beat and notifies it to the client */
static void
blehr_tx_hrate(xTimerHandle ev)
{
    static uint8_t hrm[2];
    int rc;
    struct os_mbuf *om;

    max30102_data_t result = {};
    
    /** @author Joshua D. JOHN
     * Here we replace a simulator of heartrate with the real thing: sensor data read from a 
     * finger placed on the MAX30102 Sensor
    */

    while(notify_state) {
        hrm[0] = 0x06; /* contact of a sensor */
        hrm[1] = heartrate; /* storing dummy data */
        //Update sensor, saving to "result"
        ESP_ERROR_CHECK(max30102_update(&max30102, &result));
        if(result.pulse_detected) {
            printf("BEAT\n");
            printf("BPM: %f | SpO2: %f%%\n", result.heart_bpm, result.spO2);
            heartrate = result.heart_bpm;
            om = ble_hs_mbuf_from_flat(hrm, sizeof(hrm));
            rc = ble_gattc_notify_custom(conn_handle, hrs_hrm_handle, om);
        }
        //Update rate: 100Hz
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    if (!notify_state) {
        blehr_tx_hrate_stop();
        heartrate = 90;
        return;
    }

   
    assert(rc == 0);

    blehr_tx_hrate_reset();
}

static int
blehr_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            blehr_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, hrs_hrm_handle);
        if (event->subscribe.attr_handle == hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_reset();
        } else if (event->subscribe.attr_handle != hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}

static void
blehr_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &blehr_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(blehr_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    /* Begin advertising */
    blehr_advertise();
}

static void
blehr_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void blehr_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void app_main(void)
{
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /** 
     * Adding an actual Sensor, MAX30102
     *  - Initialize the I2C port
     *  - initialize the MAX30102 sensor
     */
    //Init I2C_NUM_0
    ESP_ERROR_CHECK(i2c_master_init(I2C_PORT));
    //Init sensor at I2C_NUM_0
    ESP_ERROR_CHECK(max30102_init( &max30102, I2C_PORT,
                   MAX30102_DEFAULT_OPERATING_MODE,
                   MAX30102_DEFAULT_SAMPLING_RATE,
                   MAX30102_DEFAULT_LED_PULSE_WIDTH,
                   MAX30102_DEFAULT_IR_LED_CURRENT,
                   MAX30102_DEFAULT_START_RED_LED_CURRENT,
                   MAX30102_DEFAULT_MEAN_FILTER_SIZE,
                   MAX30102_DEFAULT_PULSE_BPM_SAMPLE_SIZE,
                   MAX30102_DEFAULT_ADC_RANGE, 
                   MAX30102_DEFAULT_SAMPLE_AVERAGING,
                   MAX30102_DEFAULT_ROLL_OVER,
                   MAX30102_DEFAULT_ALMOST_FULL,
                   false ));

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = blehr_on_sync;
    ble_hs_cfg.reset_cb = blehr_on_reset;

    /* name, period/time,  auto reload, timer ID, callback */
    blehr_tx_timer = xTimerCreate("blehr_tx_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, blehr_tx_hrate);

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(blehr_host_task);

}
