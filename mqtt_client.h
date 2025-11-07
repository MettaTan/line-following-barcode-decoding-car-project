#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdbool.h>

// ============================================================
// Core MQTT Functions
// ============================================================
void mqtt_client_start(const char *broker_ip, int port);
void mqtt_publish_message(const char *topic, const char *payload);
void mqtt_loop_poll(void);
void mqtt_client_reconnect_if_needed(void);
bool mqtt_is_connected(void);
// void mqtt_network_start(void);

// ============================================================
// Telemetry Publishers
// ============================================================
void mqtt_publish_imu(void);
void mqtt_publish_distance(void);
void mqtt_publish_speed(void);
// void mqtt_publish_line(void)

// --- Barcode publishers ---
void mqtt_publish_barcode_edge(int level, uint32_t dt_us);
void mqtt_publish_barcode_duty(float black_ms, float white_ms, float duty_pct);
void mqtt_publish_barcode_snapshot(const char *bits, int count,
                                   const char *guess, float conf);

#endif

// ============================================================
// Wi-Fi credentials (shared between main.c and mqtt_client.c)
// ============================================================
#ifndef WIFI_CREDENTIALS_H
#define WIFI_CREDENTIALS_H

#define WIFI_SSID "Bernice"
#define WIFI_PASS "BER20199"

#endif
