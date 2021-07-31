// WIFI
#define CONFIG_WIFI_SSID "singzer"
#define CONFIG_WIFI_PASS "1008610086"

// Default
#define CONFIG_PROJECT_CODE "iot"
#define CONFIG_DEVICE_MODEL "ink-epd"
#define CONFIG_DEVICE_FW 0.89
#define CONFIG_DEVICE_HW "ESP32-REV02"

// MQTT
#define CONFIG_MQTT_HOST "192.168.1.120"
#define CONFIG_MQTT_PORT 1883
#define CONFIG_MQTT_USER "device"
#define CONFIG_MQTT_PASS "password"
#define CONFIG_MQTT_KEEP_ALIVE 60

// OTA
#define CONFIG_OTA_HOST "iot.icepie.net"
#define CONFIG_OTA_PORT 6689
#define CONFIG_OTA_PATH "/" CONFIG_DEVICE_MODEL "/firmware.bin"

/*
 *  Set a topic use a unique value and topic type, just like "project/deviceid/msgtype"
 *  Return a String value
 */
String setTopic(String unique, String type)
{
    String topic = CONFIG_PROJECT_CODE;
    topic += "/";
    topic += unique;
    topic += "/";
    topic += type;

    return topic;
}