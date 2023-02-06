#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "led.h"

#define LED_R_PIN       33
#define LED_G_PIN       25
#define LED_B_PIN       32

#define LED_STATUS_PIN  23

led_status_t led_status = IDLE;

void led_init(void)
{
    // RGB LED pins should all be pulled low.
    gpio_set_direction(LED_R_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_G_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_B_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_STATUS_PIN, GPIO_MODE_OUTPUT);

    // LED init flash to indicate successful boot
    gpio_set_level(LED_R_PIN, 1);
    gpio_set_level(LED_G_PIN, 0);
    gpio_set_level(LED_B_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LED_R_PIN, 0);
    gpio_set_level(LED_G_PIN, 1);
    gpio_set_level(LED_B_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LED_R_PIN, 0);
    gpio_set_level(LED_G_PIN, 0);
    gpio_set_level(LED_B_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LED_B_PIN, 0);
}

void led_update(led_status_t st)
{
    led_status = st;
}

void led_loop(void *args)
{
    while (true)
    {
        uint64_t ms_elapsed = xTaskGetTickCount()*portTICK_PERIOD_MS;

        if(led_status == IDLE)
        {
            gpio_set_level(LED_R_PIN,0);
            gpio_set_level(LED_G_PIN,0);
            gpio_set_level(LED_B_PIN,0);
            gpio_set_level(LED_STATUS_PIN, 0);

        }
        else if(led_status == HEARTBEAT)
        {
            gpio_set_level(LED_R_PIN,0);
            gpio_set_level(LED_G_PIN,0);
            gpio_set_level(LED_B_PIN,0);
            gpio_set_level(LED_STATUS_PIN, 1);
            led_update(IDLE); // only want this for one LED tick
        }
        else if(led_status == PROCESSING)
        {
            gpio_set_level(LED_R_PIN,1);
            gpio_set_level(LED_G_PIN,1);
            gpio_set_level(LED_B_PIN,1);
        }
        else if(led_status == DENY)
        {
            gpio_set_level(LED_R_PIN,1);
            gpio_set_level(LED_G_PIN,0);
            gpio_set_level(LED_B_PIN,0); 
        }
        else if(led_status == ERROR)
        {
            if (ms_elapsed % 500 >= 250)
            {
            gpio_set_level(LED_R_PIN,1);
            gpio_set_level(LED_G_PIN,0);
            gpio_set_level(LED_B_PIN,0); 
            }
            else
            {
            gpio_set_level(LED_R_PIN,0);
            gpio_set_level(LED_G_PIN,0);
            gpio_set_level(LED_B_PIN,0);    
            }
        }
        else if(led_status == LOCKING)
        {
            gpio_set_level(LED_R_PIN,0);
            gpio_set_level(LED_G_PIN,1);
            gpio_set_level(LED_B_PIN,0);

        }
        else if(led_status == UNLOCKING)
        {
            gpio_set_level(LED_R_PIN,0);
            gpio_set_level(LED_G_PIN,0);
            gpio_set_level(LED_B_PIN,1);
        }
        else if(led_status == FIRMWARE)
        {
            if (ms_elapsed % 1000 >= 500)
            {
            gpio_set_level(LED_R_PIN,1);
            gpio_set_level(LED_G_PIN,1);
            gpio_set_level(LED_B_PIN,0);
            }
            else
            {
            gpio_set_level(LED_R_PIN,0);
            gpio_set_level(LED_G_PIN,1);
            gpio_set_level(LED_B_PIN,1);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        }

        vTaskDelete(NULL);

}