#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "mqtt_client.h"
#include "pico/time.h"
#include "imu.h"
#include "encoder.h"
#include <stdint.h>
#include "pico/types.h"

// ============================================================
// lwIP Poll Helper
// ------------------------------------------------------------
// Keeps MQTT and Wi-Fi background tasks alive.
// ============================================================
void mqtt_loop_poll(void)
{
    sys_check_timeouts();
    cyw43_arch_poll();
}

// ============================================================
// Manual struct definition (needed for Pico SDK 1.5.1 lwIP)
// ============================================================
struct mqtt_connect_client_info {
    const char *client_id;
    const char *client_user;
    const char *client_pass;
    uint16_t    keep_alive;
    const char *will_topic;
    const char *will_msg;
    uint8_t     will_qos;
    uint8_t     will_retain;
    
};

// ============================================================
// Global configuration
// ============================================================
static mqtt_client_t *mqtt_client;
static ip_addr_t broker_ip_addr;
static int broker_port;
static struct mqtt_connect_client_info client_info;

// ============================================================
// Public helper for connection status
// ============================================================
bool mqtt_is_connected(void)
{
    if (mqtt_client == NULL)
        return false;
    return mqtt_client_is_connected(mqtt_client);
}

// ============================================================
// MQTT Connection Callback
// ============================================================
static bool mqtt_first_connection_done = false;
static void mqtt_connection_cb(mqtt_client_t *client,
                               void *arg,
                               mqtt_connection_status_t status)
{
    if (status == MQTT_CONNECT_ACCEPTED) {
        mqtt_first_connection_done = true;
        printf("MQTT connected successfully.\n");
    } else {
        printf("MQTT connection failed (status %d)\n", status);
    }
}

// ============================================================
// MQTT Publish Helper
// ============================================================
void mqtt_publish_message(const char *topic, const char *payload) {
    // Always check Wi-Fi link *first*
    int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

    if (link_status != CYW43_LINK_UP) {
        printf("Wi-Fi disconnected (status=%d) — cannot publish to %s\n", link_status, topic);
        return;
    }

    // Now check if MQTT session is alive
    if (!mqtt_client_is_connected(mqtt_client)) {
        printf("MQTT not connected — skipping publish to %s\n", topic);
        return;
    }

    // Attempt publish
    err_t err = mqtt_publish(mqtt_client, topic, payload,
                             strlen(payload), 0, 0, NULL, NULL);

    if (err == ERR_OK) {
        printf("Published to %s: %s\n", topic, payload);
    } 
    else {
        // Publish failed; recheck link to know whether it’s Wi-Fi or broker
        int recheck = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (recheck != CYW43_LINK_UP) {
            printf("Wi-Fi disconnected (status=%d). Attempting Wi-Fi reconnect...\n", recheck);
            int r = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                                       CYW43_AUTH_WPA2_AES_PSK, 10000);
            if (r == 0)
                printf("Wi-Fi reconnected successfully.\n");
            else
                printf("Wi-Fi reconnection failed (err %d).\n", r);
        } else {
            // Wi-Fi still fine → broker unreachable or busy
            printf("Broker unreachable or busy — Wi-Fi connection is OK.\n");
        }
    }

    mqtt_loop_poll();
}

// ============================================================
// IMU Telemetry Publisher (Raw + Filtered)
// ------------------------------------------------------------
// Uses imu_get_all_data() to publish raw and filtered IMU data
// ============================================================
void mqtt_publish_imu(void)
{
    imu_data_t imu_data;
    imu_get_all_data(&imu_data);

    // ---------- Raw Data (Accelerometer + Heading) ----------
    char raw_payload[160];
    snprintf(raw_payload, sizeof(raw_payload),
             "{\"raw_pitch\": %.2f, \"raw_roll\": %.2f, \"raw_heading\": %.2f}",
             imu_data.raw_pitch, imu_data.raw_roll, imu_data.raw_heading);
    mqtt_publish_message("telemetry/imu/raw", raw_payload);

    // ---------- Filtered Data (Orientation) ----------
    char filt_payload[160];
    snprintf(filt_payload, sizeof(filt_payload),
             "{\"pitch\": %.2f, \"roll\": %.2f, \"heading\": %.2f}",
             imu_data.filt_pitch, imu_data.filt_roll, imu_data.filt_heading);
    mqtt_publish_message("telemetry/imu/filtered", filt_payload);
}

// // ============================================================
// // Encoder Telemetry Publisher (Single Combined Speed)
// // ------------------------------------------------------------
// // Publishes average wheel speed and total distance travelled
// // ============================================================
void mqtt_publish_speed(void) {
    const float wheel_diameter_mm = 65.0f;  // Wheel diameter in millimeters

    // Get the encoder data (speed, distance, etc.)
    EncoderData data = encoder_get_data(wheel_diameter_mm);

    // Create the payload for speed
    char payload[80];
    snprintf(payload, sizeof(payload), "{ \"speed\": %.2f }", data.avg_speed / 1000.0f);  // Convert to m/s
    mqtt_publish_message("telemetry/speed", payload);
}

void mqtt_publish_distance(void) {
    const float wheel_diameter_mm = 65.0f;  // Wheel diameter in millimeters

    // Get the encoder data (speed, distance, etc.)
    EncoderData data = encoder_get_data(wheel_diameter_mm);

    // Create the payload for distance
    char payload[80];
    snprintf(payload, sizeof(payload), "{ \"distance\": %.2f }", data.avg_distance / 1000.0f);  // Convert to meters
    mqtt_publish_message("telemetry/distance", payload);
}

void mqtt_publish_barcode_edge(int level, uint32_t dt_us) {
    if (!mqtt_is_connected()) return;
    char p[96];
    snprintf(p, sizeof(p), "{\"level\":%d,\"dt_us\":%u}", level, dt_us);
    mqtt_publish_message("telemetry/barcode/edge", p);
}

void mqtt_publish_barcode_duty(float black_ms, float white_ms, float duty_pct) {
    if (!mqtt_is_connected()) return;
    char p[128];
    snprintf(p, sizeof(p),
        "{\"black_ms\":%.1f,\"white_ms\":%.1f,\"duty\":%.1f}",
        black_ms, white_ms, duty_pct);
    mqtt_publish_message("telemetry/barcode/duty", p);
}

void mqtt_publish_barcode_snapshot(const char *bits, int count,
                                   const char *guess, float conf) {
    if (!mqtt_is_connected()) return;
    char p[256];
    snprintf(p, sizeof(p),
        "{\"count\":%d,\"bits\":\"%s\",\"guess\":\"%s\",\"conf\":%.2f}",
        count, bits ? bits : "", guess ? guess : "UNKNOWN", conf);
    mqtt_publish_message("telemetry/barcode/snapshot", p);
}


// // ============================================================
// // Line Telemetry Publisher 
// // ------------------------------------------------------------
// // Publishes line events and movement states
// // ============================================================
// void mqtt_publish_line(void)
// {
//     line_state_t state = line_get_state();
//     const char *state_str = line_state_to_string(state);

//     // Derive line event description
//     const char *event_str;
//     if (strcmp(state_str, "TURN_LEFT") == 0)
//         event_str = "Turning Left";
//     else if (strcmp(state_str, "TURN_RIGHT") == 0)
//         event_str = "Turning Right";
//     else if (strcmp(state_str, "OFF_TRACK") == 0)
//         event_str = "Lost Line";
//     else if (strcmp(state_str, "ON_TRACK") == 0)
//         event_str = "Following Line";
//     else
//         event_str = "Unknown";

//     char payload[160];
//     snprintf(payload, sizeof(payload),
//              "{ "
//                "\"state\": \"%s\", "
//                "\"line_event\": \"%s\" "
//              "}",
//              state_str, event_str);

//     mqtt_publish_message("telemetry/line", payload);
// }

// ============================================================
// MQTT Initialization
// ============================================================
void mqtt_client_start(const char *broker_ip, int port)
{
    printf("Resolving MQTT broker %s ...\n", broker_ip);
    if (!ip4addr_aton(broker_ip, &broker_ip_addr)) {
        printf("Invalid broker IP format.\n");
        return;
    }
    broker_port = port;

    mqtt_client = mqtt_client_new();
    if (!mqtt_client) {
        printf("Failed to create MQTT client.\n");
        return;
    }

    client_info.client_id   = "pico_telemetry";
    client_info.client_user = NULL;
    client_info.client_pass = NULL;
    client_info.keep_alive  = 60;
    client_info.will_topic  = NULL;
    client_info.will_msg    = NULL;
    client_info.will_qos    = 0;
    client_info.will_retain = 0;

    printf("Connecting to MQTT broker %s:%d ...\n", broker_ip, port);
    mqtt_client_connect(mqtt_client, &broker_ip_addr, broker_port,
                        mqtt_connection_cb, NULL,
                        (const struct mqtt_connect_client_info_t *)&client_info);
}

// ============================================================
// MQTT Reconnect Handler (for MQTT-05 test)
// ============================================================
static bool mqtt_reconnecting = false;
static absolute_time_t last_attempt_time;
void mqtt_client_reconnect_if_needed(void)
{
    if (mqtt_client == NULL || mqtt_reconnecting)
        return;

    // Do not attempt to reconnect until we have ever connected successfully
    if (!mqtt_first_connection_done)
        return;

    // Retry every 5 seconds at most
    if (absolute_time_diff_us(last_attempt_time, get_absolute_time()) < 5 * 1000000)
        return;

    last_attempt_time = get_absolute_time();

    if (!mqtt_client_is_connected(mqtt_client)) {
        mqtt_reconnecting = true;
        printf("MQTT disconnected. Attempting safe reconnect...\n");

        mqtt_disconnect(mqtt_client);
        for (int i = 0; i < 5; i++) {
            mqtt_loop_poll();
            sleep_ms(100);
        }

        mqtt_client_free(mqtt_client);
        mqtt_client = mqtt_client_new();
        if (!mqtt_client) {
            printf("Failed to recreate MQTT client.\n");
            mqtt_reconnecting = false;
            sleep_ms(2000);
            return;
        }

        if (!ip4addr_aton("172.20.10.13", &broker_ip_addr)) {
            printf("Invalid broker IP on reconnect.\n");
            mqtt_reconnecting = false;
            return;
        }

        printf("Reconnecting to MQTT broker %s:%d ...\n", "172.20.10.13", broker_port);
        err_t err = mqtt_client_connect(mqtt_client, &broker_ip_addr, broker_port,
                                        mqtt_connection_cb, NULL,
                                        (const struct mqtt_connect_client_info_t *)&client_info);

        if (err == ERR_OK)
            printf("Reconnect attempt started (waiting for broker)...\n");
        else
            printf("mqtt_client_connect() failed immediately (err %d)\n", err);

        mqtt_reconnecting = false;
    }
}


// // ============================================================
// // Telemetry Data Publisher (unit testing))
// // ============================================================
// void mqtt_publish_telemetry(void)
// {
//     char payload[256];

//     float speed   = 0.45f + (rand() % 20) / 100.0f;
//     float heading = 90.0f + (rand() % 10);
//     float ax      = (rand() % 100) / 500.0f;
//     float ay      = (rand() % 100) / 500.0f;

//     snprintf(payload, sizeof(payload), "{\"speed\": %.2f}", speed);
//     mqtt_publish_message("telemetry/speed", payload);
//     sleep_ms(80);

//     snprintf(payload, sizeof(payload), "{\"heading\": %.1f}", heading);
//     mqtt_publish_message("telemetry/heading", payload);
//     sleep_ms(80);

//     snprintf(payload, sizeof(payload),
//              "{\"ax\": %.2f, \"ay\": %.2f, \"az\": 9.81}", ax, ay);
//     mqtt_publish_message("telemetry/imu/raw", payload);
//     sleep_ms(80);

//     float fx = ax * 0.95f, fy = ay * 0.95f;
//     snprintf(payload, sizeof(payload),
//              "{\"ax\": %.2f, \"ay\": %.2f, \"az\": 9.81}", fx, fy);
//     mqtt_publish_message("telemetry/imu/filtered", payload);
//     sleep_ms(80);

//     const char *barcode = (rand() % 3 == 0) ? "LEFT" :
//                           (rand() % 3 == 1) ? "RIGHT" : "STOP";
//     snprintf(payload, sizeof(payload), "{\"barcode\": \"%s\"}", barcode);
//     mqtt_publish_message("telemetry/barcode", payload);
//     sleep_ms(80);

//     snprintf(payload, sizeof(payload),
//              "{\"state\": \"%s\"}", (rand() % 2) ? "MOVING" : "TURNING");
//     mqtt_publish_message("telemetry/state", payload);
//     sleep_ms(80);

//     int distance = 120 + (rand() % 50);
//     snprintf(payload, sizeof(payload), "{\"distance\": %d}", distance);
//     mqtt_publish_message("telemetry/distance", payload);
//     sleep_ms(80);

//     int dist  = 25 + (rand() % 10);
//     int clear = 10 + (rand() % 5);
//     int width = 5  + (rand() % 6);
//     const char *status     = (rand() % 2) ? "RECOVERED" : "AVOIDING";
//     const char *line_event = (rand() % 2) ? "ON_TRACK" : "OFF_TRACK";

//     snprintf(payload, sizeof(payload),
//              "{\"distance\": %d, \"clearance\": %d, "
//              "\"obstacle_width\": %d, \"recovery_status\": \"%s\", "
//              "\"line_event\": \"%s\"}",
//              dist, clear, width, status, line_event);
//     mqtt_publish_message("telemetry/obstacle", payload);
//     sleep_ms(80);
// }


