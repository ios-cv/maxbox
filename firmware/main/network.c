#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_system.h"
#include "esp_tls.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "cJSON.h"
#include "network.h"
#include "esp_crt_bundle.h"
#include "led.h"

// WiFi
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD

#define ESP_MAXIMUM_RETRY           3
#define MAX_HTTP_RECV_BUFFER        512
#define MAX_HTTP_OUTPUT_BUFFER      2048
#define MAX_WAIT_MS                 5000 // maximum time to wait for wifi connection

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT          BIT0 // we are connected to the AP with an IP
#define WIFI_FAIL_BIT               BIT1 // we failed to connect after the maximum amount of retries
#define WIFI_OPERATION_FINISHED_BIT BIT2 // we are not in the process of (dis)connecting

static int s_retry_num = 0;
static int desired_connection_state = 0;

static const char* TAG = "MaxBox Network";

extern int etag;
extern EventGroupHandle_t s_status_group;

extern char firmware_update_url[255];

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
        s_retry_num = 0;
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY && desired_connection_state == 1) {
            ESP_LOGI(TAG, "WIFI_STA_DISCONNECTED retry");
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            if (s_retry_num >= ESP_MAXIMUM_RETRY)
            {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGI(TAG, "WIFI_STA_DISCONNECTED too many retries");
            }
            else if (desired_connection_state == 0)
            {
                ESP_LOGI(TAG, "Disconnecting from wifi");
            }
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            desired_connection_state = 0;
            esp_wifi_stop();
        }
        ESP_LOGI(TAG,"Not connected to the AP");

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { 
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    desired_connection_state = 1;
    
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
         .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    xEventGroupSetBits(s_wifi_event_group, WIFI_OPERATION_FINISHED_BIT);
    wifi_reconnect();

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

void wifi_reconnect()
{
    xEventGroupWaitBits(s_wifi_event_group,
            WIFI_OPERATION_FINISHED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT)) // we're already connected
    {
        xEventGroupClearBits(s_wifi_event_group, WIFI_OPERATION_FINISHED_BIT);
        desired_connection_state = 1;
        s_retry_num = 0;

        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);

        esp_wifi_start();

        /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
         * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                pdFALSE,
                pdFALSE,
                MAX_WAIT_MS);

        /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
         * happened. */
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Connected to SSID:%s password:%s",
                     ESP_WIFI_SSID, ESP_WIFI_PASS);
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                     ESP_WIFI_SSID, ESP_WIFI_PASS);
        } else {
            ESP_LOGE(TAG, "UNEXPECTED EVENT");
        }
    }
    xEventGroupSetBits(s_wifi_event_group, WIFI_OPERATION_FINISHED_BIT);
}

void wifi_disconnect()
{
    ESP_LOGI(TAG, "Waiting for WiFi operations to complete...");
    xEventGroupWaitBits(s_wifi_event_group,
            WIFI_OPERATION_FINISHED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi operations complete, disconnecting");

    if ((xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT)) // 
    {
        xEventGroupClearBits(s_wifi_event_group, WIFI_OPERATION_FINISHED_BIT);
        desired_connection_state = 0;
        esp_wifi_stop();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_OPERATION_FINISHED_BIT);
    }

}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}

static esp_err_t _http_set_headers(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;

    char mac_addr_string[13];
    char rendered_etag[10];

    uint8_t base_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(base_mac, ESP_MAC_WIFI_STA));

    sprintf(mac_addr_string, "%02x%02x%02x%02x%02x%02x", base_mac[0], base_mac[1], base_mac[2], base_mac[3], base_mac[4], base_mac[5]);
    sprintf(rendered_etag, "%d", etag);

    esp_http_client_set_header(http_client, "Accept", "application/json");
    esp_http_client_set_header(http_client, "Content-Type", "application/json");
    esp_http_client_set_header(http_client, "X-Carshare-Box-ID", mac_addr_string);
    esp_http_client_set_header(http_client, "X-Carshare-Box-Secret", "s3cr3t-go3s-h3r3");
    esp_http_client_set_header(http_client, "X-Carshare-Operator-Card-List-ETag", rendered_etag);
    esp_http_client_set_header(http_client, "X-Carshare-Firmware-Version", "8");
    return err;
}

void firmware_update(void* pxParameters)
{
    led_update(FIRMWARE);

    esp_err_t ota_finish_err = ESP_OK;

    ESP_LOGI(TAG, "Updating firmware from %s", firmware_update_url);

    esp_http_client_config_t config = {
        .url = firmware_update_url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .http_client_init_cb = _http_set_headers,
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        goto ota_end;
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
        goto ota_end;
    }

    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
        // esp_https_ota_perform returns after every read operation which gives user the ability to
        // monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
        // data read so far.
        ESP_LOGD(TAG, "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        // the OTA image was not completely received and user can customise the response to this situation.
        ESP_LOGE(TAG, "Complete data was not received.");
    } else {
        ota_finish_err = esp_https_ota_finish(https_ota_handle);
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
            ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        } else {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
            }
            ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
            goto ota_end;

        }
    }

ota_end:
    esp_https_ota_abort(https_ota_handle);
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
    xEventGroupClearBits(s_status_group, BIT4);
    led_update(IDLE);
    vTaskDelete( NULL );
}

void http_auth_rfid(void *rest_request)
{
	const rest_request_t *request = (rest_request_t *) rest_request;

    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

    esp_http_client_config_t config = {
        .url = request->url,
        .user_agent = "Carshare Box v0.0.0.0.0.1 ;)",
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    ESP_LOGI(TAG, "POST DATA is %s", request->data);
    esp_http_client_set_method(client, HTTP_METHOD_POST);

    _http_set_headers(client);

    esp_http_client_set_post_field(client, request->data, strlen(request->data));
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));

                ESP_LOGI(TAG, "Got data: %s", local_response_buffer);

                request->callback(local_response_buffer);

    } else {
        
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
        if (request->alert_on_error)
        {
            led_update(ERROR);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            led_update(IDLE);
            xEventGroupSetBits(s_status_group, BIT3);
        }
    }
    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "Sent auth request");

    vTaskDelete( NULL );
}
