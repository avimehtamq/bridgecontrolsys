/*  WiFi softAP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include <ultrasonic.h>
#include <esp32/rom/ets_sys.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define GTK_REKEY_INTERVAL 0
#endif

#define opening 2 
#define closing 3 // Extra states for Bridge State 
#define GPIO_PWM0A_OUT 19   // Set GPIO 19 as PWMA
#define GPIO_PWM0B_OUT 21   // Set GPIO 21 as PWMB

static const char *TAG = "MM6-BB-AP"; // Info tag - will be seen in serial monitor
static int ts = 0, // Traffic state *IMPORTANT* note: false = 0, true = 1 | false = nodetect, true = detect
bs = 0, // Bridge state *IMPORTANT* note: false = 0, true = 1, closing = 2, opening = 3 | false = close, true = open, closing = closing, opening = opening
os = 0; // Override state *IMPORTANT* note: false = 0, true = 1 | false = nooverride, true = override
uint32_t d1 = 0, d2 = 0; // Distances of respective sensors (numerical association i.e. s1 to d1)

static const ultrasonic_sensor_t s1 = {
    .trigger_pin = 26,
    .echo_pin = 36,
};
static const ultrasonic_sensor_t s2 = {
    .trigger_pin = 27,
    .echo_pin = 34,
};

static void mcpwm_initialise()
{
    ESP_LOGI(TAG, "Initialising MCPWM GPIO...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    
    ESP_LOGI(TAG, "Initilisating MCPWM Configuration");
    //2. MCPWM configuration
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 500;    // frequency = 500Hz,   
    pwm_config.cmpr_a = 0;    // duty cycle of PWMA = 0
    pwm_config.cmpr_b = 0;    // duty cycle of PWMB = 0
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

void motor_lift(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void motor_lower(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

esp_err_t readSensors() // This method saves the ultrasonic read in the distance floats
{
    if (ultrasonic_measure_cm(&s1, 100, &d1) == ESP_ERR_TIMEOUT && ultrasonic_measure_cm(&s2, 100, &d2) == ESP_ERR_TIMEOUT) {
    return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// Begin state section

void setTrafficState() // Traffic state dependent on what the sensors read from readSensors()
{
    ts = false; // Assuming maritime traffic is slow majority of the time
    if(readSensors() != ESP_ERR_TIMEOUT) {
        if((d1 > 50 && d1 < 80) || (d2 > 50 && d2 < 80)) {
        ts = true;
        }
    } else {
        ESP_LOGI(TAG, "Traffic State remains: %d", ts);
        return;
        }
    ESP_LOGI(TAG, "Set Traffic State to: %d", ts);
}

bool getTrafficState() 
{
    return ts;
}

void overrideTrafficState(int d)
{
    ts = d ? true : false;
}

void setBridgeState(int d)
{
    switch (d) {
        case 0: bs = false; break;
        case 1: bs = true; break;
        case 2: bs = closing; break;
        case 3: bs = opening; break;
        default: ESP_LOGE(TAG, "Unexpected bridge state!");
    }
    ESP_LOGI(TAG, "Set Bridge State to: %d", bs);
}

int getBridgeState() 
{
    return bs;
} 

void setOverrideState(bool d) 
{
    os = d ? true : false;
}

int getOverrideState() 
{
    return os;
}

char* bridgeStateString()
{
char* a = "";

switch (getBridgeState()) {
    case false: a = "0"; break;
    case true: a = "1"; break;
    case closing: a = "2"; break;
    case opening: a = "3"; break;
    default: ESP_LOGE(TAG, "Unexpected bridge state!");
    }
return a;
}

// Creates a task for motor control when requested by handlers
void motor_task()
{
    int target = 0;
    switch (bs) {
        case closing:
        target = false;
        break;
        case opening:
        target = true;
        break;
    }
 
    while (!os && bs != target) {
        if(bs == opening) {
            ESP_LOGI(TAG, "Moving motor forward");
            motor_lift(MCPWM_UNIT_0, MCPWM_TIMER_0, 25.0); 
        } else {
            ESP_LOGI(TAG, "Moving motor backward");
            motor_lower(MCPWM_UNIT_0, MCPWM_TIMER_0, 25.0);
        }
    vTaskDelay(500);
    }
motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
vTaskDelete(NULL);
}        


// These handlers are called upon specific scenarios that can be seen in the state_task method

void nodetect_handler()
{   
    if(getBridgeState() == true) { // OPEN
    setBridgeState(closing); 
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
    setBridgeState(false);
    }
    ESP_LOGI(TAG, "Detecting... Sensors: %dcm, %dcm, Traffic state: %d, Bridge state: %d", d1, d2, getTrafficState(), getBridgeState());
}

void detect_handler()
{
    ESP_LOGI(TAG, "Detected! Sensors: %dcm, %dcm, Traffic state: %d, Bridge state: %d", d1, d2, getTrafficState(), getBridgeState());
    setBridgeState(opening);
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
    setBridgeState(true);
}

// Creates a persistent task to run a state check
void state_task() 
{
while (!os) {
setTrafficState();
    if(getBridgeState() != closing || getBridgeState() != opening) {
        switch (getTrafficState()) {
            case false:
                nodetect_handler(); break;
            case true:
                detect_handler(); break;
            }
        }
    vTaskDelay(pdMS_TO_TICKS(1000)); // check every second
    }
vTaskDelete(NULL);
}

// End state section                 

// Begin Webserver section 
// This section deals with the ESP32 hosted webpage and its commands

static esp_err_t index_handler(httpd_req_t *req)
{
    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
    const size_t html_len = index_html_end - index_html_start;

    httpd_resp_send(req, (const char *)index_html_start, html_len);
    return ESP_OK;
}

static esp_err_t halt_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "CMD:HALT");
    httpd_resp_send(req, "HALT RECEIVED", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t open_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "CMD:OPEN");
    overrideTrafficState(true);
    setOverrideState(true);
    ESP_LOGI(TAG, "TS: %d, OS: %d", getTrafficState(), getOverrideState());
    httpd_resp_send(req, "OPEN RECEIVED", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t close_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "CMD:CLOSE");
    overrideTrafficState(false);
    setOverrideState(true);
    httpd_resp_send(req, "CLOSE RECEIVED", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t restart_auto_handler(httpd_req_t *req)
{
    if(getOverrideState() == false) {
    ESP_LOGI(TAG, "Already automated...");
    httpd_resp_send(req, "Already automated", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
    }
    ESP_LOGI(TAG, "Sending restart command...");
    setOverrideState(false);
    httpd_resp_send(req, "Restarting automation.", HTTPD_RESP_USE_STRLEN);
    xTaskCreate(state_task, "state_task", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static esp_err_t traffic_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Sending %d", getTrafficState());
    httpd_resp_send(req, getTrafficState() ? "1" : "0", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t bridge_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Sending %s", bridgeStateString());
    httpd_resp_send(req, bridgeStateString(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// These are the declarations of the page URIs, which need to be registered

static const httpd_uri_t bcs_index = {
    .uri       = "/",               // the address at which the resource can be found
    .method    = HTTP_GET,          // The HTTP method (HTTP_GET, HTTP_POST, ...)
    .handler   = index_handler,     // The function which process the request
    .user_ctx  = NULL               // Additional user data for context
};

static const httpd_uri_t bcs_halt = {
    .uri       = "/halt",
    .method    = HTTP_GET,
    .handler   = halt_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t bcs_open = {
    .uri       = "/open",
    .method    = HTTP_GET,
    .handler   = open_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t bcs_close = {
    .uri       = "/close",
    .method    = HTTP_GET,
    .handler   = close_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t restart_auto = {
    .uri       = "/restart",
    .method    = HTTP_GET,
    .handler   = restart_auto_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t traffic_state = {
    .uri       = "/traffic_state",
    .method    = HTTP_GET,
    .handler   = traffic_state_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t bridge_state = {
    .uri       = "/bridge_state",
    .method    = HTTP_GET,
    .handler   = bridge_state_handler,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver() {
httpd_handle_t server = NULL;
httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Server started successfully, registering URI handlers...");
        httpd_register_uri_handler(server, &bcs_index);
        httpd_register_uri_handler(server, &bcs_halt);
        httpd_register_uri_handler(server, &bcs_open);
        httpd_register_uri_handler(server, &bcs_close);
        httpd_register_uri_handler(server, &restart_auto);
        httpd_register_uri_handler(server, &traffic_state);
        httpd_register_uri_handler(server, &bridge_state);
        return server;
    }   
    ESP_LOGE(TAG, "Failed to start server");
    return NULL;
}

// End webserver section

// Begin wifi section

// This wifi_event_handler method is provided in the ESP-IDF softAP example
// Creates an event handler - currently makes station connections and disconnections an event
// Events are displayed in the serial monitor

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

// This method is provided in the ESP-IDF softAP example
// Initialises the wifi configuration to default values

void wifi_init_bcs(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
#ifdef CONFIG_ESP_WIFI_BSS_MAX_IDLE_SUPPORT
            .bss_max_idle_cfg = {
                .period = WIFI_AP_DEFAULT_MAX_IDLE_PERIOD,
                .protected_keep_alive = 1,
            },
#endif
            .gtk_rekey_interval = GTK_REKEY_INTERVAL,
        },
    };
    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_bcs finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
}

// End wifi section

void app_main(void)
{
    // WiFi setup
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_bcs();
 
    // Webpage setup
    start_webserver();

    // Component setup
    ESP_ERROR_CHECK(ultrasonic_init(&s1));
    ESP_ERROR_CHECK(ultrasonic_init(&s2));
    mcpwm_initialise();

    // State task setup
    xTaskCreate(state_task, "state_task", 2048, NULL, 5, NULL);
}
