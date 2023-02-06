/* WiFi and network communication functions
*/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef void(*rest_callback_t)(char*);

typedef struct {
    char *url;                   /*<! URL to POST to */
    char data[1023];             /*<! JSON data to send */
    rest_callback_t callback;    /*<! callback function */
    bool alert_on_error;         /*<! signal error if request fails */       
} rest_request_t;

void wifi_init_sta(void);
void wifi_disconnect(void);
void wifi_reconnect(void);
void http_auth_rfid(void* rest_request);
void firmware_update(void* url);

#ifdef __cplusplus
}
#endif