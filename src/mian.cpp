/*
    LilyGo Ink Screen Series u8g2Fonts Test
        - Created by Lewis he
*/

// According to the board, cancel the corresponding macro definition
#define LILYGO_T5_V213
// #define LILYGO_T5_V22
// #define LILYGO_T5_V24
// #define LILYGO_T5_V28
// #define LILYGO_T5_V266
// #define LILYGO_EPD_DISPLAY_102
// #define LILYGO_EPD_DISPLAY_154

#include <boards.h>
#include <GxEPD.h>

#if defined(LILYGO_T5_V102) || defined(LILYGO_EPD_DISPLAY_102)
#include <GxGDGDEW0102T4/GxGDGDEW0102T4.h> //1.02" b/w
#elif defined(LILYGO_T5_V266)
#include <GxDEPG0266BN/GxDEPG0266BN.h> // 2.66" b/w   form DKE GROUP
#elif defined(LILYGO_T5_V213)
#include <GxGDEH0213B73/GxGDEH0213B73.h>
// #include <GxDEPG0213BN/GxDEPG0213BN.h>    // 2.13" b/w  form DKE GROUP
#else
// #include <GxGDGDEW0102T4/GxGDGDEW0102T4.h> //1.02" b/w
// #include <GxGDEW0154Z04/GxGDEW0154Z04.h>  // 1.54" b/w/r 200x200
// #include <GxGDEW0154Z17/GxGDEW0154Z17.h>  // 1.54" b/w/r 152x152
// #include <GxGDEH0154D67/GxGDEH0154D67.h>  // 1.54" b/w
// #include <GxDEPG0150BN/GxDEPG0150BN.h>    // 1.51" b/w   form DKE GROUP
// #include <GxDEPG0266BN/GxDEPG0266BN.h>    // 2.66" b/w   form DKE GROUP
// #include <GxDEPG0290R/GxDEPG0290R.h>      // 2.9" b/w/r  form DKE GROUP
// #include <GxDEPG0290B/GxDEPG0290B.h>      // 2.9" b/w    form DKE GROUP
// #include <GxGDEW029Z10/GxGDEW029Z10.h>    // 2.9" b/w/r  form GoodDisplay
// #include <GxGDEW0213Z16/GxGDEW0213Z16.h>  // 2.13" b/w/r form GoodDisplay
// #include <GxGDE0213B1/GxGDE0213B1.h>      // 2.13" b/w  old panel , form GoodDisplay
// #include <GxGDEH0213B72/GxGDEH0213B72.h>  // 2.13" b/w  old panel , form GoodDisplay
// #include <GxGDEH0213B73/GxGDEH0213B73.h>  // 2.13" b/w  old panel , form GoodDisplay
// #include <GxGDEM0213B74/GxGDEM0213B74.h>  // 2.13" b/w  form GoodDisplay 4-color
// #include <GxGDEW0213M21/GxGDEW0213M21.h>  // 2.13"  b/w Ultra wide temperature , form GoodDisplay
// #include <GxDEPG0213BN/GxDEPG0213BN.h>    // 2.13" b/w  form DKE GROUP
// #include <GxGDEW027W3/GxGDEW027W3.h>      // 2.7" b/w   form GoodDisplay
// #include <GxGDEW027C44/GxGDEW027C44.h>    // 2.7" b/w/r form GoodDisplay
// #include <GxGDEH029A1/GxGDEH029A1.h>      // 2.9" b/w   form GoodDisplay
// #include <GxDEPG0750BN/GxDEPG0750BN.h>    // 7.5" b/w   form DKE GROUP
#endif

#include <U8g2_for_Adafruit_GFX.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
// #include "FS.h"
// #include "SPIFFS.h"

/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#define ESP_getChipId() ((uint32_t)ESP.getEfuseMac())

#define ESPhttpUpdate httpUpdate


#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

#include "config.h"
#include "led.h"

// hw set
String deviceSN = "0x" + String(ESP_getChipId(), HEX); // Must be unique on the MQTT network

// mqtt set
String pubTopic = setTopic(deviceSN, "msg");
String subTopic = setTopic(deviceSN, "event");

Led myLed(LED_PIN,true); //Reverse
WiFiClient client;
AsyncMqttClient mqttClient;

GxIO_Class io(SPI, EPD_CS, EPD_DC, EPD_RSET);
GxEPD_Class display(io, EPD_RSET, EPD_BUSY);
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

void update_started()
{
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished()
{
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total)
{
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err)
{
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

void otaUpdate()
{
  // Add optional callback notifiers
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);

  t_httpUpdate_return ret = ESPhttpUpdate.update(client, CONFIG_OTA_HOST, CONFIG_OTA_PORT, CONFIG_OTA_PATH);

  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void sendStatus(unsigned int msg_id, String cmd)
{
  DynamicJsonDocument msg(1024);

  String type = "status";

  // create the job json data
  msg["type"] = type;
  msg["msg_id"] = msg_id;
  msg["event_cmd"] = cmd;

  if (myLed.getStatus())
  {
    msg["data"]["led"] = "on";
  }
  else
  {
    msg["data"]["led"] = "off";
  }

  msg["wifi"]["ssid"] = WiFi.SSID();
  msg["wifi"]["gateway"] = WiFi.gatewayIP().toString();
  msg["wifi"]["ip"] = WiFi.localIP().toString();
  msg["wifi"]["rssi"] = WiFi.RSSI();
  msg["wifi"]["mac"] = WiFi.BSSIDstr();
  msg["wifi"]["channel"] = WiFi.channel();
  msg["wifi"]["dns"] = WiFi.dnsIP().toString();

  msg["sn"] = deviceSN;
  msg["mac"] = WiFi.macAddress();
  msg["model"] = CONFIG_DEVICE_MODEL;
  msg["hw_ver"] = CONFIG_DEVICE_HW;
  msg["fw_ver"] = CONFIG_DEVICE_FW;
  //msg["data"]["full_ver"] = ESP.getFullVersion();

  char buffer[1024];
  serializeJson(msg, buffer);
  mqttClient.publish(pubTopic.c_str(), 0, false, buffer);
}

void sendFeedback(unsigned int msg_id, String cmd)
{
  DynamicJsonDocument msg(1024);

  String type = "feedback";
  // create the job json data
  msg["type"] = type;
  msg["msg_id"] = msg_id;
  msg["event_cmd"] = cmd;

  msg["sn"] = deviceSN;
  msg["model"] = CONFIG_DEVICE_MODEL;
  msg["hw_ver"] = CONFIG_DEVICE_HW;
  msg["fw_ver"] = CONFIG_DEVICE_FW;

  char buffer[256];
  serializeJson(msg, buffer);
  mqttClient.publish(pubTopic.c_str(), 0, false, buffer);
}


TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}


void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(subTopic.c_str(), 0);
  Serial.print("Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSub);
  Serial.println(subTopic.c_str());
  // mqttClient.publish(pubTopic.c_str(),);
  // Serial.println("Publishing at QoS 0");
  // uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  // Serial.print("Publishing at QoS 1, packetId: ");
  // Serial.println(packetIdPub1);
  // uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  // Serial.print("Publishing at QoS 2, packetId: ");
  // Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{

  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload, len);
  JsonObject event = doc.as<JsonObject>();

  String cmd = event["cmd"];
  String data = event["data"];
  unsigned int msg_id = event["msg_id"];

  if (cmd == "ota")
  {
    sendFeedback(msg_id, cmd);
    otaUpdate();
  }
  else if (cmd == "status")
    sendStatus(msg_id, cmd);
  else if (cmd == "led")
  {
    if (data == "on")
      myLed.on();
    else if (data == "off")
      myLed.off();
    sendFeedback(msg_id, cmd);
  } else if (cmd == "wrt") {
    Serial.println(data);
    uint16_t x = display.width() / 2 - 120;
    uint16_t y = display.height() / 2;

    display.fillScreen(GxEPD_WHITE);
    // display.setForegroundColor(GxEPD_BLACK);
    // display.setBackgroundColor(GxEPD_BLACK);

    u8g2Fonts.setCursor(x, y); // start writing at this position
    u8g2Fonts.setFont(u8g2_font_wqy16_t_gb2312);
    u8g2Fonts.print(data);

    display.update();
    sendFeedback(msg_id, cmd);
  }
}

void onMqttPublish(uint16_t packetId)
{
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


void setup(void)
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("setup");
    myLed.on();

    SPI.begin(EPD_SCLK, EPD_MISO, EPD_MOSI);
    display.init(); // enable diagnostic output on Serial
    display.eraseDisplay();
    display.setRotation(1);
    // display.fillScreen(GxEPD_WHITE);
    // display.setTextColor(GxEPD_BLACK);
    u8g2Fonts.begin(display);

    u8g2Fonts.setFontMode(1);                  // use u8g2 transparent mode (this is default)
    u8g2Fonts.setFontDirection(0);             // left to right (this is default)
    u8g2Fonts.setForegroundColor(GxEPD_BLACK); // apply Adafruit GFX color
    u8g2Fonts.setBackgroundColor(GxEPD_WHITE); // apply Adafruit GFX color

    u8g2Fonts.setFont(u8g2_font_helvR14_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall

    uint16_t x = display.width() / 2 - 60;
    uint16_t y = display.height() / 2;

    display.fillScreen(GxEPD_WHITE);
    // display.setForegroundColor(GxEPD_BLACK);
    // display.setBackgroundColor(GxEPD_BLACK);

    u8g2Fonts.setCursor(x, y); // start writing at this position
    u8g2Fonts.setFont(u8g2_font_wqy16_t_gb2312);
    u8g2Fonts.print("我能吞下玻璃");
    u8g2Fonts.setCursor(x + 32, y + 32);
    u8g2Fonts.print("而不伤身体！");

    display.update();

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
    WiFi.onEvent(WiFiEvent);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT).setCredentials(CONFIG_MQTT_USER, CONFIG_MQTT_PASS).setClientId(deviceSN.c_str()).setKeepAlive(CONFIG_MQTT_KEEP_ALIVE);

    connectToWifi();

    // display.eraseDisplay();
}

void loop()
{

    // delay(10000);
}