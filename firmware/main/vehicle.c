#include "driver/twai.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "pthread.h"

#include "esp_log.h"

#include "inttypes.h"
#include "vehicle.h"
#include "led.h"

static const char* TAG = "MaxBox-Vehicle";

vehicle_t vhcl = NULL;

extern EventGroupHandle_t s_status_group;

uint8_t b_lock;

void can_receive_task(void *arg)
{
    while (1) {
        twai_message_t msg;
        twai_receive(&msg, portMAX_DELAY);

        if(pthread_mutex_lock(&vhcl->telemetrymux) == 0) // make sure telemetry isn't being updated as we reset it
        {

            if (msg.identifier == 0x5c5)
            {
                vhcl->odometer_miles = (msg.data[1] << 16) | (msg.data[2] << 8) | (msg.data[3]);
            }
            else if (msg.identifier == 0x55b)
            {
                vhcl->soc_percent = ((msg.data[0] << 2) | (msg.data[1] >> 6)) / 10;
            }
            else if (msg.identifier == 0x60d)
            {
                if (msg.data[2] == 0x18)
                {
                    vhcl->doors_locked = 1;
                }
                else
                {
                    vhcl->doors_locked = 0;
                }
            }
            pthread_mutex_unlock(&vhcl->telemetrymux);
        }
    }
    vTaskDelete(NULL);
}

static void send_can(twai_message_t message)
{
    twai_transmit(&message, pdMS_TO_TICKS(100));
}

static void un_lock(void *xParameters)
{
    const uint8_t *lock = (uint8_t *) xParameters;

    twai_message_t packet1;
    packet1.identifier = 0x756;
    packet1.data_length_code = 8;

    packet1.data[0] = 0x02;
    packet1.data[1] = 0x10;
    packet1.data[2] = 0x81;
    packet1.data[3] = 0xff;
    packet1.data[4] = 0xff;
    packet1.data[5] = 0xff;
    packet1.data[6] = 0xff;
    packet1.data[7] = 0xff;

    twai_message_t packet2;
    packet2.identifier = 0x756;
    packet2.data_length_code = 8;

    packet2.data[0] = 0x02;
    packet2.data[1] = 0x10;
    packet2.data[2] = 0xc0;
    packet2.data[3] = 0xff;
    packet2.data[4] = 0xff;
    packet2.data[5] = 0xff;
    packet2.data[6] = 0xff;
    packet2.data[7] = 0xff;

    twai_message_t packetlock;
    packetlock.identifier = 0x756;
    packetlock.data_length_code = 8;

    packetlock.data[0] = 0x04;
    packetlock.data[1] = 0x30;
    packetlock.data[2] = 0x07;
    packetlock.data[3] = 0x00;
    packetlock.data[4] = 0x01;
    packetlock.data[5] = 0xff;
    packetlock.data[6] = 0xff;
    packetlock.data[7] = 0xff;


    if(*lock)
    {
        ESP_LOGI(TAG, "Transmitting lock CAN packet");
        packetlock.data[4] = 0x01;
    } else {
        ESP_LOGI(TAG, "Transmitting unlock CAN packet");
        packetlock.data[4] = 0x02;
    }

    send_can(packet1);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_can(packet1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_can(packet2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    send_can(packetlock);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    send_can(packet1);

    if(*lock)
    {
        ESP_LOGI(TAG, "Car locked");
        led_update(LOCKING);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        led_update(IDLE);
    } else {
        ESP_LOGI(TAG, "Car unlocked");
        led_update(UNLOCKING);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        led_update(IDLE);
    }

    xEventGroupClearBits(s_status_group, BIT2);
    xEventGroupSetBits(s_status_group, BIT3);

    vTaskDelete(NULL);
}

esp_err_t vehicle_init(vehicle_t vehicle)
{
    vhcl = vehicle;

    if(pthread_mutex_init (&vhcl->telemetrymux, NULL) != 0){
        ESP_LOGE(TAG, "Failed to initialize the telemetry mutex");
    }

    vhcl->doors_locked = -1;
    vhcl->odometer_miles = -1;
    vhcl->aux_battery_voltage = -1;
    vhcl->soc_percent = -1;
    vhcl->ibutton_id[0] = '\0';

    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_15, GPIO_NUM_13, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //Install CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return ESP_FAIL;
    }

    //Start CAN driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void vehicle_lock_doors()
{
    b_lock = 1;
    ESP_LOGI(TAG, "Locking car");
    xTaskCreate(&un_lock, "lock_car", 8192, &b_lock, 3, NULL);
    return;
}

void vehicle_unlock_doors()
{
    b_lock = 0;
    ESP_LOGI(TAG, "Unlocking car");
    xTaskCreate(&un_lock, "unlock_car", 8192, &b_lock, 3, NULL);
    return;
}
