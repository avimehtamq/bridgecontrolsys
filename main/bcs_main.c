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
#include "driver/ledc.h"
#include "soc/mcpwm_periph.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// Definition / Declaration section

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define GTK_REKEY_INTERVAL 0
#endif

#define closing 2
#define opening 3

#define LED_PIN_1 15
#define LED_PIN_2 16
#define LED_PIN_3 17
#define LED_PIN_4 18
#define LED_PIN_5 20
#define LED_PIN_6 22
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 1000 // 1KHz 

#define GPIO_PWM0A_OUT 19   // Set GPIO 19 as PWMA
#define GPIO_PWM0B_OUT 21   // Set GPIO 21 as PWMB

static const char *TAG = "MM6-BB-AP"; // Info tag - will be seen in serial monitor
static int ts = 0; // Traffic state *IMPORTANT* note: false = 0, true = 1 | false = nodetect, true = detect
static int bs = 0; // Bridge state *IMPORTANT* note: false = 0, true = 1, closing = 2, opening = 3 | false = close, true = open, closing = closing, opening = opening
static int ls = 0; // Light state *IMPORTANT* <note> based on MARITIME TRAFFIC: false = 0, true = 1 | false = stop, true = go 
static int os = 0; // Override state *IMPORTANT* note: false = 0, true = 1 | false = nooverride, true = override
uint32_t d1 = 0, d2 = 0; // Distances of respective sensors (numerical association i.e. s1 to d1)
TaskHandle_t state; // Needs declaration for FreeRTOS task purposes

// Begin configuration section
// Sensors
static const ultrasonic_sensor_t s1 = {
    .trigger_pin = 26,
    .echo_pin = 36,
};
static const ultrasonic_sensor_t s2 = {
    .trigger_pin = 27,
    .echo_pin = 34,
};

// Motor
static void mcpwm_initialise()
{
    ESP_LOGI(TAG, "Initialising MCPWM GPIO...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    
    ESP_LOGI(TAG, "Initilisating MCPWM Configuration");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 500;    // frequency = 500Hz,   
    pwm_config.cmpr_a = 0;    // duty cycle of PWMA = 0
    pwm_config.cmpr_b = 0;    // duty cycle of PWMB = 0
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

// Lights
static void light_initialise()
{
    // Configure LEDC timer (one timer for all channels)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
    };
    ledc_timer_config(&ledc_timer);

    // Configure LEDC channels
    ledc_channel_config_t ledc_channel_1 = {
        .gpio_num = LED_PIN_1,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ledc_channel_1);

    ledc_channel_config_t ledc_channel_2 = {
        .gpio_num = LED_PIN_2,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ledc_channel_2);

    ledc_channel_config_t ledc_channel_3 = {
        .gpio_num = LED_PIN_3,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_3,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ledc_channel_3);

    ledc_channel_config_t ledc_channel_4 = {
        .gpio_num = LED_PIN_4,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ledc_channel_4);

    ledc_channel_config_t ledc_channel_5 = {
        .gpio_num = LED_PIN_5,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_5,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ledc_channel_5);

    ledc_channel_config_t ledc_channel_6 = {
        .gpio_num = LED_PIN_6,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_6,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ledc_channel_6);
}

// Begin component function section
esp_err_t readSensors() // This method saves the ultrasonic read in the distance floats
{
    if (ultrasonic_measure_cm(&s1, 100, &d1) == ESP_ERR_TIMEOUT && ultrasonic_measure_cm(&s2, 100, &d2) == ESP_ERR_TIMEOUT) {
    return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// Motor control
void motor_open(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void motor_close(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
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

// Light control
// Note: Road Amber lights are channels 1, 2, Road Red lights are 3, 4, Boat lights are 5, 6
void roadlight_amber()
{
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 100);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, 100);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
}

void roadlight_stop()
{
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, 100);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_4, 100);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_4);
}

void roadlight_go()
{   
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, 0);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);

ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, 0);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3);
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_4, 0);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_4);
}

void waterlight_go() {
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_5, 0);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_5);
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_6, 0);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_6);
}

void waterlight_stop()
{
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_5, 100);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_5);
ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_6, 100);
ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_6);
}

// End functions section

// Begin state section

int getTrafficState() 
{
    return ts;
}

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

void overrideTrafficState(int d)
{
    ts = d ? true : false;
}

int getBridgeState() 
{
    return bs;
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

int getLightState() 
{
    return ls;
} 

void setLightState(int d)
{
    ls = d ? true : false;
    ESP_LOGI(TAG, "Set Light State to: %d", d);
}

int getOverrideState() 
{
    return os;
}

void setOverrideState(int d) 
{
    os = d ? true : false;
    ESP_LOGI(TAG, "Set Override State to: %d", d);
}

char* bridgeStateString()
{
char* a = "";

switch (bs) {
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
        ESP_LOGI(TAG, "Close motor");
        while (bs != target) {
        motor_close(MCPWM_UNIT_0, MCPWM_TIMER_0, 25.0);
        vTaskDelay(50);
        }
        motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        setBridgeState(target);
        vTaskDelete(NULL);
        break;

        case opening:
        target = true;
        ESP_LOGI(TAG, "Open motor");
        while (bs != target) {
        motor_open(MCPWM_UNIT_0, MCPWM_TIMER_0, 25.0);
        vTaskDelay(50);
        }
        motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        setBridgeState(target);
        vTaskDelete(NULL);
        break;

        default:
        ESP_LOGI(TAG, "Ongoing operation.");
        vTaskDelete(NULL);
        break;
    }
}        

// These handlers are called upon specific scenarios that can be seen in the state_task method

void nodetect_handler()
{   
    if(getBridgeState() == true) { // OPEN
    setBridgeState(closing); 
    xTaskCreatePinnedToCore(motor_task, "motor_task", 2048, NULL, 3, NULL, 0);
    }
    ESP_LOGI(TAG, "Detecting... Sensors: %dcm, %dcm, Traffic state: %d, Bridge state: %d", d1, d2, getTrafficState(), getBridgeState());
}

void detect_handler()
{
    if(getBridgeState() == false) {
    ESP_LOGI(TAG, "Detected! Sensors: %dcm, %dcm, Traffic state: %d, Bridge state: %d", d1, d2, getTrafficState(), getBridgeState());
    setBridgeState(opening);
    roadlight_amber();
    roadlight_stop();
    xTaskCreatePinnedToCore(motor_task, "motor_task", 2048, NULL, 3, NULL, 0);
    }
}

// Creates a persistent task to run a state check
void state_task(void *pvParameters) 
{
while (true) {
    while (!os) {
    setTrafficState();
    ESP_LOGI("CORE", "State Task is running on core %d", xPortGetCoreID());
        if(getBridgeState() != closing || getBridgeState() != opening) {
            switch (getTrafficState()) {
                case false:
                    nodetect_handler(); break;
                case true:
                    detect_handler(); break;
             }
         }
        vTaskDelay(90); // check every second (with grace for sensor timeout of 0.1 seconds)
        }
    vTaskSuspend(state);
    }
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
    setOverrideState(true);
    httpd_resp_send(req, "HALT RECEIVED", HTTPD_RESP_USE_STRLEN);
    setLightState(false);
    roadlight_stop();
    waterlight_stop();
    return ESP_OK;
}

static esp_err_t open_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "CMD:OPEN");
    setOverrideState(true);
    setBridgeState(opening);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 2048, NULL, 3, NULL, 0);
    ESP_LOGI(TAG, "TS: %d, OS: %d", getTrafficState(), getOverrideState());
    httpd_resp_send(req, "OPEN RECEIVED", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t close_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "CMD:CLOSE");
    setOverrideState(true);
    setBridgeState(closing);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 2048, NULL, 3, NULL, 0);
    httpd_resp_send(req, "CLOSE RECEIVED", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t restart_auto_handler(httpd_req_t *req)
{
    if(!getOverrideState()) {
    ESP_LOGI(TAG, "Already automated...");
    httpd_resp_send(req, "Already automated", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
    } else {
    ESP_LOGI(TAG, "Sending restart command...");
    setOverrideState(false);
    httpd_resp_send(req, "Restarting automation.", HTTPD_RESP_USE_STRLEN);
    vTaskResume(state);
    return ESP_OK;
    }
}

static esp_err_t traffic_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Sending TS:%d", getTrafficState());
    httpd_resp_send(req, getTrafficState() ? "1" : "0", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t bridge_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Sending BS:%s", bridgeStateString());
    httpd_resp_send(req, bridgeStateString(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t light_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Sending LS:%d", getLightState());
    httpd_resp_send(req, getLightState() ? "1" : "0", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t override_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Sending OS:%d", getOverrideState());
    httpd_resp_send(req, getOverrideState() ? "1" : "0", HTTPD_RESP_USE_STRLEN);
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

static const httpd_uri_t bcs_restart_auto = {
    .uri       = "/restart",
    .method    = HTTP_GET,
    .handler   = restart_auto_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t bcs_traffic_state = {
    .uri       = "/traffic_state",
    .method    = HTTP_GET,
    .handler   = traffic_state_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t bcs_bridge_state = {
    .uri       = "/bridge_state",
    .method    = HTTP_GET,
    .handler   = bridge_state_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t bcs_light_state = {
    .uri       = "/light_state",
    .method    = HTTP_GET,
    .handler   = light_state_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t bcs_override_state = {
    .uri       = "/override_state",
    .method    = HTTP_GET,
    .handler   = override_state_handler,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver() {
httpd_handle_t server = NULL;
httpd_config_t config = HTTPD_DEFAULT_CONFIG();
config.max_uri_handlers = 9;
config.core_id = 1;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Server started successfully, registering URI handlers...");
        httpd_register_uri_handler(server, &bcs_index);
        httpd_register_uri_handler(server, &bcs_halt);
        httpd_register_uri_handler(server, &bcs_open);
        httpd_register_uri_handler(server, &bcs_close);
        httpd_register_uri_handler(server, &bcs_restart_auto);
        httpd_register_uri_handler(server, &bcs_traffic_state);
        httpd_register_uri_handler(server, &bcs_bridge_state);
        httpd_register_uri_handler(server, &bcs_light_state);
        httpd_register_uri_handler(server, &bcs_override_state);
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
 
    // Webserver setup
    start_webserver();

    // Component setup
    ESP_ERROR_CHECK(ultrasonic_init(&s1));
    ESP_ERROR_CHECK(ultrasonic_init(&s2));
    mcpwm_initialise();
    light_initialise();

    // State task setup
    xTaskCreatePinnedToCore(state_task, "state_task", 2048, NULL, 2, &state, 0);
}
