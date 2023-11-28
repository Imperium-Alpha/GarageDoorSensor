#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

#define DOOR_OPEN_SENSOR_PIN 16
#define DOOR_CLOSED_SENSOR_PIN 17

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

char wifi_ssid[16];
char wifi_psw[16];
char host_ip[16];
uint16_t port;
char device_name[50];
char user[50];
char password[50];

const char* publish_topic = "home/stat/garage_door_state";
int status_code; // -2 wtf, -1 unknown, 0 open, 1 closed

volatile bool pin_change = true; // for initial status update

volatile int open_pin_previous_state = LOW;
volatile int closed_pin_previous_state = LOW;

volatile int open_pin_new_state = LOW;
volatile int closed_pin_new_state = LOW;

void reconnect() {
    while (!mqtt_client.connected()) {
        if (mqtt_client.connect(device_name, user, password)) {

        }
        else vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    Serial.println("MQTT Connected");
}

//void IRAM_ATTR sensorPinStateChange() {
//
//    pin_change = true;
//}

void setup() {

    Serial.begin(115200);
    SPIFFS.begin();
    StaticJsonDocument<2048> config;
    bool file_read_successful = false;
    if (SPIFFS.exists("/config.json")) {
        File config_file = SPIFFS.open("/config.json", FILE_READ);
        if (config_file) {
            DeserializationError error = deserializeJson(config, config_file);
            if (!error) {

                config_file.close();
                port = config["port"].as<uint16_t>();

                strcpy(wifi_ssid, config["wifi_ssid"]);
                strcpy(wifi_psw, config["wifi_psw"]);
                strcpy(host_ip, config["host_ip"]);
                strcpy(device_name, config["device_name"]);
                strcpy(user, config["user"]);
                strcpy(password, config["password"]);

                file_read_successful = true;

            }
        }
    }

    SPIFFS.end();

    if (!file_read_successful) {
        ESP.restart();
        return;
    }

    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(wifi_ssid, wifi_psw);

    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    mqtt_client.setBufferSize(2048);
    mqtt_client.setServer(host_ip, port);
    reconnect();

    pinMode(DOOR_OPEN_SENSOR_PIN, INPUT_PULLUP);
    pinMode(DOOR_CLOSED_SENSOR_PIN, INPUT_PULLUP);

    open_pin_previous_state = digitalRead(DOOR_OPEN_SENSOR_PIN);
    closed_pin_previous_state = digitalRead(DOOR_CLOSED_SENSOR_PIN);

//    attachInterrupt(DOOR_CLOSED_SENSOR_PIN, sensorPinStateChange, CHANGE);
//    attachInterrupt(DOOR_OPEN_SENSOR_PIN, sensorPinStateChange, CHANGE);

}

void loop() {

    open_pin_new_state = digitalRead(DOOR_OPEN_SENSOR_PIN);
    closed_pin_new_state = digitalRead(DOOR_CLOSED_SENSOR_PIN);

    if (WiFi.status() == WL_CONNECTED && !mqtt_client.connected()) {
        reconnect();
    } else if (WiFi.status() != WL_CONNECTED) {
        Serial.end();
        ESP.restart();
    }

    if (open_pin_previous_state != open_pin_new_state || closed_pin_previous_state != closed_pin_new_state) {
        pin_change = true;
    }



    if (closed_pin_new_state == LOW && open_pin_new_state == LOW) {
        Serial.println("How is it possible, both sensors detect magnet");
        status_code = -2;
    } else if (closed_pin_new_state == HIGH && open_pin_new_state == HIGH) {
        Serial.println("Both sensor detects no magnet, door is in unknown state");
        status_code = -1;
    } else if (closed_pin_new_state == LOW && open_pin_new_state == HIGH) {
        Serial.println("Door closed sensor detects magnet, door is closed");
        status_code = 1;
    } else if (closed_pin_new_state == HIGH && open_pin_new_state == LOW) {
        Serial.println("Door open sensor detects magnet, door is open");
        status_code = 0;
    }

    // the problem is the open pin always stay low

    if (pin_change) {
        String payload = "";
        if (status_code == 1) payload = "closed";
        else if (status_code == 0) payload = "open";
        else if (status_code == -1) payload = "unknown";
        // else if (status_code == -1) payload = "open";
        // else if (status_code == -2) payload = "wtf";
        else if (status_code == -2) payload = "closed";
        mqtt_client.publish(publish_topic, payload.c_str(), true);
        pin_change = false;
    }



    open_pin_previous_state = open_pin_new_state;
    closed_pin_previous_state = closed_pin_new_state;

    vTaskDelay(2000 / portTICK_PERIOD_MS);
}