/* Vehicle class: currently interacts with Nissan Leaf / e-NV200
*/
#pragma once

#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

struct vehicle {
    pthread_mutex_t telemetrymux;
	int8_t doors_locked;                   /*<! 1 = doors locked, 0 = doors unlocked */
    int32_t odometer_miles;                /*<! current odometer reading, in miles */
    float aux_battery_voltage;             /*<! standby battery voltage, from ADC */
    float soc_percent;                     /*<! HV state of charge, in percent */
    char ibutton_id[17];                   /*<! ID of iButton currently attached */ 
};

typedef struct vehicle* vehicle_t;

/**
 * @brief FreeRTOS CAN bus receive task
 */
void can_receive_task(void *arg);

/**
 * @brief Initialize vehicle CAN bus communications.
 * @param vehicle Vehicle struct to be updated when CAN bus wakes
 * @return ESP_OK on success
 */
esp_err_t vehicle_init(vehicle_t vehicle);

/**
 * @brief Lock vehicle doors
 */
void vehicle_lock_doors();

/**
 * @brief Unlock vehicle doors
 */
void vehicle_unlock_doors();

#ifdef __cplusplus
}
#endif