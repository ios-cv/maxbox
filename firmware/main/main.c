#include "stdio.h"
#include "string.h"
#include "driver/gpio.h"
#include "inttypes.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "pthread.h"

#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "nvs_flash.h"
#include "cJSON.h"

#include "rc522.h"
#include "network.h"
#include "vehicle.h"
#include "led.h"
#include "owb.h"

#include <time.h>
#include <sys/time.h>
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

// GPIO

#define RFID_MISO_PIN   5
#define RFID_MOSI_PIN   18
#define RFID_SCK_PIN    19
#define RFID_SDA_PIN    21
#define RFID_NRSTPD_PIN 17
#define RFID_IRQ_PIN    22

#define CAN_TX_PIN      15
#define CAN_RX_PIN      13
#define CAN_SLEEP_PIN   16

#define ONEWIRE_PIN     26

#define API_ENDPOINT_TOUCH          CONFIG_API_ROOT "touch"
#define API_ENDPOINT_TELEMETRY      CONFIG_API_ROOT "telemetry"

#define MAX_OPERATOR_CARDS          32

#define TAG_CHECK_INTERVAL_MS       500
#define TELEMETRY_SEND_INTERVAL_MS  120000

#define TELEMETRY_TIMEOUT_MS        8000
#define TOUCH_TIMEOUT_MS            20000

/* FreeRTOS event group to signal when it's safe to power off*/
EventGroupHandle_t s_status_group;

#define TELEMETRY_SENDING_BIT   BIT0 // currently sending telemetry
#define TELEMETRY_DONE_BIT      BIT1 // telemetry is sent
#define TAG_PROCESSING_BIT      BIT2 // currently processing a tag
#define TAG_DONE_BIT            BIT3 // tag processing is finished
#define FIRMWARE_UPDATING_BIT   BIT4 // firmware update in progress

static const char* TAG = "MaxBox";
static esp_adc_cal_characteristics_t adc1_chars;
rest_request_t touch_req;
rest_request_t telemetry_req;

char firmware_update_url[255] = {0};

struct maxbox {
    vehicle_t vehicle;
    int operator_car_lock;
    char operator_card_list[MAX_OPERATOR_CARDS][9];
};

int etag = -1;

OneWireBus * owb;
owb_rmt_driver_info rmt_driver_info;

typedef struct maxbox* maxbox_handle_t;

static maxbox_handle_t hndl = NULL;

static void io_init(void)
{
    // Power up the MFRC522
    gpio_set_direction(RFID_NRSTPD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RFID_NRSTPD_PIN, 1);

    // TODO: Sleep CAN transmitter until required (to save power) 
    // For the moment we just leave it awake all the time
    gpio_set_direction(CAN_SLEEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN_SLEEP_PIN, 0);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
}

static void flash_init(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    uint8_t base_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(base_mac, ESP_MAC_WIFI_STA));

    ESP_LOGI(TAG, "Loaded NVS. Base MAC address %02x%02x%02x%02x%02x%02x", base_mac[0], base_mac[1], base_mac[2], base_mac[3], base_mac[4], base_mac[5]);

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Reading operator card list from NVS ...");
        nvs_get_i32(my_handle, "etag", &etag);

        size_t required_size = sizeof(hndl->operator_card_list);
        nvs_get_blob(my_handle, "op_card_list", hndl->operator_card_list, &required_size);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Loaded etag: %i", etag);
            int i;
            for (i=0; i<MAX_OPERATOR_CARDS; i++)
            {
                if (strcmp(hndl->operator_card_list[i], "voidvoid") != 0)
                {
                    ESP_LOGI(TAG, "Operator card %i: %s", i+1, hndl->operator_card_list[i]);
                }
            }
        }
        else if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGI(TAG, "No etag value written in flash");
        }
        else
        {
            ESP_LOGE(TAG, "Error (%s) reading flash", esp_err_to_name(err));
        }
    }
}

void json_touch_handler(char* result)
{
    cJSON *result_json = cJSON_Parse(result);
    
    if(cJSON_GetObjectItem(result_json, "action"))
    {
        char *action = cJSON_GetObjectItem(result_json, "action")->valuestring;
        
        if (strcmp(action, "lock") == 0)
        {
            vehicle_lock_doors();
        } 
        else if (strcmp(action, "unlock") == 0)
        {
            vehicle_unlock_doors();
        } 
        else if (strcmp(action, "reject") == 0)
        {
            ESP_LOGI(TAG, "Request rejected");

            led_update(DENY); 
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            led_update(IDLE);

            xEventGroupSetBits(s_status_group, TAG_DONE_BIT);
        } 
    }
    else
    {
        ESP_LOGE(TAG, "Card unlock error");

        led_update(ERROR);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        led_update(IDLE);

        xEventGroupSetBits(s_status_group, TAG_DONE_BIT);
    }

    cJSON_Delete(result_json);

}

void json_telemetry_handler(char* result)
{
    cJSON *result_json = cJSON_Parse(result);

    if(cJSON_GetObjectItem(result_json, "operator_card_list"))
    {
        cJSON *card_list = cJSON_GetObjectItem(result_json, "operator_card_list");
        cJSON *recv_etag = cJSON_GetObjectItem(card_list, "etag");

        if (cJSON_IsNumber(recv_etag))
        {
            if (recv_etag->valuedouble != etag)
            {
                ESP_LOGI(TAG, "New etag is %d", recv_etag->valueint);

                cJSON *card;
                cJSON *op_cards = cJSON_GetObjectItem(card_list, "cards");
                int i = 0;
                cJSON_ArrayForEach(card, op_cards)
                {
                    char *cardid = card->valuestring;
                    ESP_LOGI(TAG, "Added card to operator list with id: %s", cardid);
                    strncpy(hndl->operator_card_list[i], cardid, 9);
                    i++;
                }

                for (; i<MAX_OPERATOR_CARDS; i++)
                {
                    strcpy(hndl->operator_card_list[i], "voidvoid");
                }

                etag = recv_etag->valuedouble;
                
                nvs_handle_t my_handle;

                esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
                } else {

                ESP_LOGI(TAG, "Writing operator card to NVS...");

                nvs_set_i32(my_handle, "etag", etag);

                size_t required_size = sizeof(hndl->operator_card_list);
                nvs_set_blob(my_handle, "op_card_list", hndl->operator_card_list, required_size);
            }
            }
        }
    }

    // Optionally, there may be an action to manually lock or unlock the car remotely
    if(cJSON_GetObjectItem(result_json, "action"))
    {
        char *action = cJSON_GetObjectItem(result_json, "action")->valuestring;
        if (strcmp(action, "lock") == 0)
        {
            vehicle_lock_doors();
        } 
        else if (strcmp(action, "unlock") == 0)
        {
            vehicle_unlock_doors();
        }
    }

    if(cJSON_GetObjectItem(result_json, "firmware_update_url"))
    {
        char *fw_url = cJSON_GetObjectItem(result_json, "firmware_update_url")->valuestring;
        strcpy(firmware_update_url, fw_url);
        ESP_LOGI(TAG, "Firmware update detected, updating from URL %s", firmware_update_url);
        xEventGroupSetBits(s_status_group, FIRMWARE_UPDATING_BIT);      
        xTaskCreate(firmware_update, "firmware_update", 8192, NULL, 5, NULL);
    } 

    cJSON_Delete(result_json);

    if(pthread_mutex_lock(&hndl->vehicle->telemetrymux) == 0) // make sure telemetry isn't being updated as we reset it
    {
        hndl->vehicle->soc_percent = -1;
        hndl->vehicle->odometer_miles = -1;
        hndl->vehicle->doors_locked = -1;
        pthread_mutex_unlock(&hndl->vehicle->telemetrymux);
    }
    
    ESP_LOGI(TAG, "Finished sending telemetry");
    xEventGroupClearBits(s_status_group, TELEMETRY_SENDING_BIT);        
    xEventGroupSetBits(s_status_group, TELEMETRY_DONE_BIT);
}

static void update_ibutton_id(void)
{
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    ESP_LOGI(TAG, "Searching for iButton...");
    owb_search_first(owb, &search_state, &found);
    OneWireBus_ROMCode device_rom_code;

    if(pthread_mutex_lock(&hndl->vehicle->telemetrymux) == 0) // make sure telemetry isn't being updated as we set it
    {
        hndl->vehicle->ibutton_id[0] = '\0';
        esp_err_t err = owb_read_rom(owb, &device_rom_code);

        if (err == ESP_OK) {
            owb_string_from_rom_code(device_rom_code, hndl->vehicle->ibutton_id, sizeof(hndl->vehicle->ibutton_id));
        }

        pthread_mutex_unlock(&hndl->vehicle->telemetrymux);
    }
}

static void tag_handler(uint8_t* sn) // serial number is always 4 bytes long
{
    xEventGroupSetBits(s_status_group, TAG_PROCESSING_BIT);
    led_update(PROCESSING);

    char card_id[9];
    sprintf(card_id, "%02x%02x%02x%02x", sn[0], sn[1], sn[2], sn[3]);

    ESP_LOGI(TAG, "Detected card %s", card_id);

    // // wait for telemetry to return if we're already in the middle of an HTTP request.
    // // for some reason, sending an HTTP request if we're already waiting for a response
    // // can cause confusing behaviour (the JSON response sometimes gets sent to the wrong
    // // callback function) so easier just to wait.

    // if(xEventGroupGetBits(s_status_group) & TELEMETRY_SENDING_BIT)
    // {
    //     ESP_LOGI(TAG, "Waiting for telemetry operation to finish before processing card");
    //     xEventGroupWaitBits(s_status_group,
    //     TELEMETRY_DONE_BIT,
    //     pdFALSE,
    //     pdFALSE,
    //     TELEMETRY_TIMEOUT_MS/portTICK_PERIOD_MS);
    // }

    // first let's check if this is a tag in our operator card list
    int i;
    for (i=0; i<MAX_OPERATOR_CARDS; i++)
    {
        if (strcmp(hndl->operator_card_list[i], card_id) == 0)
        {
            if (hndl->operator_car_lock == 0)
            {
                ESP_LOGI(TAG, "Operator card detected, locking");
                vehicle_lock_doors();
                hndl->operator_car_lock = 1;
            }
            else
            {
                ESP_LOGI(TAG, "Operator card detected, unlocking");
                vehicle_unlock_doors();
                hndl->operator_car_lock = 0;
            }
            return;
        }
    }

    wifi_reconnect();

    ESP_LOGI(TAG, "Not an operator tag, reconnecting to wifi");

    cJSON *root;
    root=cJSON_CreateObject();
    cJSON_AddItemToObject(root, "card_id", cJSON_CreateString(card_id));

    update_ibutton_id();

    cJSON_AddStringToObject(root, "ibutton_id",  hndl->vehicle->ibutton_id);

    char *rendered=cJSON_Print(root);
    strcpy(touch_req.data, rendered);

    cJSON_Delete(root);
    free(rendered);

    touch_req.callback = json_touch_handler;
    touch_req.url = API_ENDPOINT_TOUCH;
    touch_req.alert_on_error = pdTRUE;

    xTaskCreate(&http_auth_rfid, "http_auth_rfid", 8192, &touch_req, 2, NULL);
}

static void update_battery_voltage(void)
{
    uint32_t voltage_raw = adc1_get_raw(ADC1_CHANNEL_0);
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(voltage_raw, &adc1_chars);
    float voltage = (float)voltage_mv / 179;

    if(pthread_mutex_lock(&hndl->vehicle->telemetrymux) == 0) // make sure telemetry isn't being updated as we set it
    {
        hndl->vehicle->aux_battery_voltage = voltage;
        pthread_mutex_unlock(&hndl->vehicle->telemetrymux);
    }
}

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
        ESP_LOGI(TAG, "ADC calibrated");
    } else {
        ESP_LOGE(TAG, "ADC calibration failed");
    }

    return cali_enable;
}

static void init_rfid(void)
{
    const rc522_start_args_t start_args = {
        .miso_io  = 5,
        .mosi_io  = 18,
        .sck_io   = 19,
        .sda_io   = 21,
        .callback = &tag_handler,
        .spi_host_id = VSPI_HOST
    };

    rc522_init(&start_args);
}

static void telemetry_loop(void *args)
{
    while (true) {
        if (xEventGroupGetBits(s_status_group) & TAG_PROCESSING_BIT)
        {
            ESP_LOGI(TAG, "Tag operation in progress, waiting to send telemetry");
            // don't send telemetry if we're already processing a tag
            xEventGroupWaitBits(s_status_group,
            TAG_DONE_BIT,
            pdTRUE,
            pdFALSE,
            TOUCH_TIMEOUT_MS/portTICK_PERIOD_MS);
            continue;
        }

        if (xEventGroupGetBits(s_status_group) & FIRMWARE_UPDATING_BIT)
        {
            ESP_LOGI(TAG, "Firmware update in progress, not sending telemetry");
            vTaskDelay(60000 / portTICK_PERIOD_MS);
            continue;
        }

        uint32_t free_heap_size = esp_get_free_heap_size();
        ESP_LOGI(TAG, "Free heap is %zu", free_heap_size);

        xEventGroupSetBits(s_status_group, TELEMETRY_SENDING_BIT);
        xEventGroupClearBits(s_status_group, TELEMETRY_DONE_BIT);

        led_update(HEARTBEAT);
        ESP_LOGI(TAG, "Reconnecting wifi to send telemetry");
        wifi_reconnect();

        cJSON *root, *tel;
        root=cJSON_CreateObject();
        cJSON_AddItemToObject(root, "telemetry", tel=cJSON_CreateObject());

        if (hndl->vehicle->soc_percent != -1)
        {
            cJSON_AddNumberToObject(tel, "soc_percent", hndl->vehicle->soc_percent);
        }
        if (hndl->vehicle->odometer_miles != -1)
        {
            cJSON_AddNumberToObject(tel, "odometer_miles", hndl->vehicle->odometer_miles);
        }
        if (hndl->vehicle->doors_locked != -1)
        {
            cJSON_AddNumberToObject(tel, "doors_locked", hndl->vehicle->doors_locked);
        }

        update_battery_voltage();
        cJSON_AddNumberToObject(tel, "aux_battery_voltage",  hndl->vehicle->aux_battery_voltage);

        update_ibutton_id();

        cJSON_AddStringToObject(tel, "ibutton_id",  hndl->vehicle->ibutton_id);

        cJSON_AddNumberToObject(tel, "box_uptime_s", esp_timer_get_time()/1000000);

        cJSON_AddNumberToObject(tel, "box_free_heap_bytes", esp_get_free_heap_size());

        char *rendered=cJSON_Print(root);

        strcpy(telemetry_req.data, rendered);

        cJSON_Delete(root);
        free(rendered);

        telemetry_req.callback = json_telemetry_handler;
        telemetry_req.url = API_ENDPOINT_TELEMETRY;
        telemetry_req.alert_on_error = pdFALSE;

        xTaskCreate(http_auth_rfid, "http_auth_rfid", 8192, &telemetry_req, 2, NULL);

        xEventGroupWaitBits(s_status_group,
        TELEMETRY_DONE_BIT,
        pdTRUE,
        pdFALSE,
        TELEMETRY_TIMEOUT_MS/portTICK_PERIOD_MS);

        xEventGroupClearBits(s_status_group, TELEMETRY_SENDING_BIT);

        if(xEventGroupGetBits(s_status_group) & TAG_PROCESSING_BIT)
        {
            ESP_LOGI(TAG,"Not disconnecting wifi - tag handling operation in progress");
        }
        else if(xEventGroupGetBits(s_status_group) & FIRMWARE_UPDATING_BIT)
        {
            ESP_LOGI(TAG,"Not disconnecting wifi - firmware update in progress");
        }
        else
        {
            ESP_LOGI(TAG, "Telemetry sent, disconnecting wifi");
            wifi_disconnect();

        }

        vTaskDelay(TELEMETRY_SEND_INTERVAL_MS / portTICK_PERIOD_MS);

    }
    vTaskDelete(NULL);
}

static void tag_loop(void *args)
{
    while (true) {
        // is there a tag?
        uint8_t* serial_no = rc522_get_tag();

        if(serial_no)
        {
            ESP_LOGI(TAG, "Processing tag");
            xEventGroupClearBits(s_status_group, TAG_DONE_BIT);
            xEventGroupSetBits(s_status_group, TAG_PROCESSING_BIT);
            tag_handler(serial_no);
            xEventGroupWaitBits(s_status_group,
            TAG_DONE_BIT,
            pdTRUE,
            pdFALSE,
            TOUCH_TIMEOUT_MS/portTICK_PERIOD_MS);
            xEventGroupClearBits(s_status_group, TAG_PROCESSING_BIT);
            ESP_LOGI(TAG, "Tag done");

            if(xEventGroupGetBits(s_status_group) & TAG_PROCESSING_BIT)
            {
                ESP_LOGI(TAG,"Not disconnecting wifi - telemetry operation in progress");
            }
            else if(xEventGroupGetBits(s_status_group) & FIRMWARE_UPDATING_BIT)
            {
                ESP_LOGI(TAG,"Not disconnecting wifi - firmware update in progress");
            }
            else {
                ESP_LOGI(TAG, "Tag operation finished, disconnecting wifi");
                wifi_disconnect();
            }
        }

        // are all modem/unlocking/locking/LED tasks finished? can we sleep?
        // check if LED_IDLE && CAN_IDLE && WIFI_DISCONNECTED

        vTaskDelay(TAG_CHECK_INTERVAL_MS / portTICK_PERIOD_MS);

        // esp_sleep_enable_timer_wakeup(TAG_CHECK_INTERVAL_MS);
        // esp_light_sleep_start();
    }
    vTaskDelete(NULL);
}

void ibutton_init(void)
{
     // Create a 1-Wire bus, using the RMT timeslot driver
    owb = owb_rmt_initialize(&rmt_driver_info, ONEWIRE_PIN, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code
}

void app_main(void)
{
    hndl = calloc(1, sizeof(struct maxbox));
    hndl->vehicle = calloc(1, sizeof(struct vehicle));

    hndl->operator_car_lock = 0;

    io_init();
    led_init();
    flash_init();
    init_rfid();
    vehicle_init(hndl->vehicle);
    adc_calibration_init();
    wifi_init_sta();
    ibutton_init();
    led_update(IDLE);

    s_status_group = xEventGroupCreate();

    xTaskCreate(tag_loop, "tag_loop", 4096, NULL, 6, NULL);
    xTaskCreate(telemetry_loop, "telemetry_loop", 4096, NULL, 6, NULL);
    xTaskCreate(led_loop, "led_loop", 4096, NULL, 4, NULL);
    xTaskCreatePinnedToCore(can_receive_task, "can_receive_task", 4096, NULL, 3, NULL, tskNO_AFFINITY);
}
