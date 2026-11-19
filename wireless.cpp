#include "wireless.h"
#include "tx.h"
#include "rx.h"
#include "web.h"
#include "debug_log.h"
#include "helpers.h"
#include "serial_intercept.h"

#define AP_IP       IPAddress(192, 168, 50, 1)
#define AP_GATEWAY  IPAddress(192, 168, 50, 1)
#define AP_SUBNET   IPAddress(255, 255, 255, 0)

WiFiClient espClient;
PubSubClient client(espClient);
AsyncWebServer server(80);

tx_request_t tx_requests[NUM_CHANNELS] = {};

static unsigned long last_mqtt_attempt_ms = 0;
static uint16_t mqtt_fail_count = 0;
static const unsigned long MQTT_RECONNECT_INTERVAL_MS = 5000;
static const uint16_t MQTT_MAX_FAILS_BEFORE_REBOOT = 200;

static bool attempt_mqtt_connect();
static void mqtt_callback(char* topic, byte* payload, unsigned int length);
static void publish_discovery();
static void publish_all_states();

extern const char* FW_VERSION_STR;

static String availability_topic() {
  return String(device_config.mqtt_topic_tx) + "/status";
}

void wireless_setup() {
    bool wifi_sta_mode = (device_config.wifi_ssid[0] != '\0');
  
    if (wifi_sta_mode)
        start_sta_mode();
    else
        start_ap_mode();

    web_begin(server, client);
    server.begin();
    web_log_mark_network_ready();
    Serial.println("HTTP server started");
    ElegantOTA.begin(&server);
    
    if (wifi_sta_mode) {
        led_set_blink(100);
        client.setServer(device_config.mqtt_server, device_config.mqtt_port);
        client.setBufferSize(3072);
        client.setCallback(mqtt_callback);
    }
}

void wireless_loop() {
    if (WiFi.getMode() == WIFI_AP) {
        led_set_blink(1000);
    } else {
        if (!client.connected()) {
            unsigned long now = millis();
            if (last_mqtt_attempt_ms == 0 || (now - last_mqtt_attempt_ms >= MQTT_RECONNECT_INTERVAL_MS)) {
                last_mqtt_attempt_ms = now;
                if (attempt_mqtt_connect()) {
                    mqtt_fail_count = 0;
                    Serial.println("MQTT connected");
                } else {
                    mqtt_fail_count++;
                    if (mqtt_fail_count >= MQTT_MAX_FAILS_BEFORE_REBOOT) {
                        Serial.println("MQTT unreachable for too long, restarting ESP...");
                        delay(200);
                        ESP.restart();
                    }
                }
            }
        } else {
            client.loop();
        }
    }
    ElegantOTA.loop();
}

static void publish_all_states() {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
    public_message(mqtt_data[ch]);
  }
}

static bool attempt_mqtt_connect() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  String lwt_topic = availability_topic();
  bool ok = false;

  if (device_config.mqtt_user[0] != '\0') {
    ok = client.connect(device_config.device_name,
                        device_config.mqtt_user,
                        device_config.mqtt_pass,
                        lwt_topic.c_str(),
                        0,
                        true,
                        "offline");
  } else {
    ok = client.connect(device_config.device_name,
                        lwt_topic.c_str(),
                        0,
                        true,
                        "offline");
  }

  if (ok) {
    client.publish(lwt_topic.c_str(), "online", true);
    client.unsubscribe(device_config.mqtt_topic_rx);
    client.subscribe(device_config.mqtt_topic_rx);
    publish_discovery();
    publish_all_states();
    return true;
  }

  Serial.print("MQTT connect failed, rc=");
  Serial.println(client.state());
  return false;
}

static void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  (void)topic;
  JsonDocument jsonDoc;
  String jsonString;
  jsonString.reserve(length + 1);

  for (unsigned int i = 0; i < length; i++) 
      jsonString += (char)tolower(payload[i]);

  DeserializationError err = deserializeJson(jsonDoc, jsonString);
  if (err) {
    public_debug_message("MQTT JSON parse failed");
    return;
  }
  
  uint8_t ch = 0;
  if (jsonDoc["ch"].is<uint8_t>()) {
    ch = jsonDoc["ch"].as<uint8_t>();
    if (ch >= NUM_CHANNELS) return;
  }

  mqtt_data[ch].retries = 0;

  uint8_t temp = jsonDoc["temp"].is<uint8_t>()
                   ? jsonDoc["temp"].as<uint8_t>()
                   : mqtt_data[ch].temp;

  String state = jsonDoc["state"].is<const char*>()
                   ? jsonDoc["state"].as<String>()
                   : mqtt_data[ch].state;

  uint8_t fan = jsonDoc["fan"].is<const char*>()
                  ? fan_label_to_damper_angle(jsonDoc["fan"].as<String>())
                  : mqtt_data[ch].fan;

  AcMode mode = jsonDoc["mode"].is<const char*>()
                  ? ac_mode_from_string(jsonDoc["mode"].as<String>())
                  : ac_mode_from_string(mqtt_data[ch].mode);

  AcModel model = ac_model_from_string(device_config.ac_model);

  if (model == AC_MODEL_VAT6 && mode == AC_MODE_UNKNOWN) {
    public_debug_message("Command rejected: missing HEAT/COOL mode");
    return;
  }

  if (!tx_requests[ch].pending) {
      tx_requests[ch].pending = true;
      tx_requests[ch].ch = ch;
      tx_requests[ch].temp = temp;
      tx_requests[ch].state = state;
      tx_requests[ch].fan = fan;
      tx_requests[ch].mode = mode;
  } else {
      public_debug_message("Received duplicate command on ch " + String(ch));
  }
}

void public_message(mqtt_data_t data) {
  JsonDocument doc;
  char output[180];
  
  doc["ch"] = data.ch;
  doc["temp"] = data.temp;
  doc["state"] = data.state;
  doc["fan"] = damper_angle_to_fan_label(data.fan);
  doc["mode"] = data.mode;

  serializeJson(doc, output, sizeof(output));
  Serial.println(output);

  if (!client.connected()) return;
  String ch_topic = String(device_config.mqtt_topic_tx) + "/ch" + String(data.ch);
  client.publish(ch_topic.c_str(), output, true);
}

void public_debug_message(String msg) {
  JsonDocument doc;
  char output[1024];
  
  doc["device"] = device_config.device_name;
  doc["msg"] = msg;
  
  serializeJson(doc, output, sizeof(output));
  Serial.println(output);
  if (!client.connected()) return;
  if (!client.publish(device_config.mqtt_topic_tx, output)) {
      doc["msg"] = "MQTT publish failed!";
      serializeJson(doc, output, sizeof(output));
      client.publish(device_config.mqtt_topic_tx, output);
  }
}

void public_raw_message(JsonDocument& doc) {
  char output[2048];
  
  doc["device"] = device_config.device_name;
  
  serializeJson(doc, output, sizeof(output));
  Serial.println(output);
  if (!client.connected()) return;
  if (!client.publish(device_config.mqtt_topic_tx, output)) {
      doc["msg"] = "MQTT publish failed!";
      serializeJson(doc, output, sizeof(output));
      client.publish(device_config.mqtt_topic_tx, output);
  }
}

void start_ap_mode() {
    Serial.println("No WiFi SSID configured, entering AP mode");
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET)) {
        Serial.println("Failed to configure AP IP");
    }

    WiFi.softAP("ESP_Damper_Setup");

    Serial.println("AP mode started");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    led_set_blink(1000);
}

void start_sta_mode() {
    Serial.print("Connecting to: ");
    Serial.print(device_config.wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(device_config.device_name);
    WiFi.setSleep(false);
    WiFi.begin(device_config.wifi_ssid, device_config.wifi_pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println(" connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

static String device_unique_base() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac.toLowerCase();
  return String("esp_damper_") + mac;
}

static void publish_discovery() {
  const String base = device_unique_base();

  if (!device_config.enable_discovery) {
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
      String t = String("homeassistant/climate/") + base + "_ch" + ch + "/config";
      client.publish(t.c_str(), (const uint8_t*)"", 0, true);
    }
    Serial.println("HA MQTT discovery: cleared");
    return;
  }

  const AcModel model = ac_model_from_string(device_config.ac_model);
  const bool is_vat6 = (model == AC_MODEL_VAT6);
  const String lwt_topic = availability_topic();

  JsonDocument doc;
  static char buffer[2560];

  for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
    doc.clear();

    const String unique_id    = base + "_ch" + ch;
    const String object_id    = String("damper_ch") + ch;
    const String config_topic = String("homeassistant/climate/") + unique_id + "/config";

    doc["name"]      = String("Damper Ch") + ch;
    doc["unique_id"] = unique_id;
    doc["object_id"] = object_id;

    doc["availability_topic"]   = lwt_topic;
    doc["payload_available"]    = "online";
    doc["payload_not_available"] = "offline";

    JsonObject dev = doc["device"].to<JsonObject>();
    JsonArray ids  = dev["identifiers"].to<JsonArray>();
    ids.add(base);
    dev["name"]         = device_config.device_name;
    dev["manufacturer"] = "ESP_Damper";
    dev["model"]        = "TAC-910 / VAT-6 IR bridge";
    dev["sw_version"]   = FW_VERSION_STR;

    JsonArray modes = doc["modes"].to<JsonArray>();
    modes.add("off");
    if (is_vat6) {
      modes.add("cool");
      modes.add("heat");
    } else {
      modes.add("auto");
    }

    JsonArray fan_modes = doc["fan_modes"].to<JsonArray>();
    fan_modes.add("Low");
    fan_modes.add("Medium");
    fan_modes.add("High");
    fan_modes.add("Auto");

    doc["min_temp"]  = 16;
    doc["max_temp"]  = 30;
    doc["temp_step"] = 1;

    const String ch_tx_topic = String(device_config.mqtt_topic_tx) + "/ch" + String(ch);
    const char* rx_topic     = device_config.mqtt_topic_rx;

    doc["current_temperature_topic"]    = ch_tx_topic;
    doc["current_temperature_template"] = "{{ value_json.temp }}";

    doc["temperature_state_topic"]      = ch_tx_topic;
    doc["temperature_state_template"]   = "{{ value_json.temp }}";

    doc["mode_state_topic"] = ch_tx_topic;
    if (is_vat6) {
      doc["mode_state_template"] =
          "{% if value_json.state == 'off' %}off"
          "{% elif value_json.mode in ['cool','heat'] %}{{ value_json.mode }}"
          "{% endif %}";
    } else {
      doc["mode_state_template"] =
          "{% if value_json.state == 'off' %}off{% else %}auto{% endif %}";
    }

    doc["fan_mode_state_topic"]    = ch_tx_topic;
    doc["fan_mode_state_template"] = "{{ value_json.fan }}";

    doc["temperature_command_topic"] = rx_topic;
    if (is_vat6) {
      doc["temperature_command_template"] =
          String("{\"ch\":") + ch +
          ",\"temp\":{{ value | int }}"
          ",\"state\":\"on\""
          ",\"mode\":\"{{ this.state if this.state in ['cool','heat'] else 'cool' }}\""
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}";
    } else {
      doc["temperature_command_template"] =
          String("{\"ch\":") + ch +
          ",\"temp\":{{ value | int }}"
          ",\"state\":\"on\""
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}";
    }

    doc["mode_command_topic"] = rx_topic;
    if (is_vat6) {
      doc["mode_command_template"] =
          String("{% if value == 'off' %}") +
          "{\"ch\":" + ch + ",\"state\":\"off\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}"
          "{% else %}"
          "{\"ch\":" + ch +
          ",\"state\":\"on\""
          ",\"mode\":\"{{ value }}\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}"
          "{% endif %}";
    } else {
      doc["mode_command_template"] =
          String("{% if value == 'off' %}") +
          "{\"ch\":" + ch + ",\"state\":\"off\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}"
          "{% else %}"
          "{\"ch\":" + ch +
          ",\"state\":\"on\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"fan\":\"{{ this.attributes.fan_mode or 'Auto' }}\"}"
          "{% endif %}";
    }

    doc["fan_mode_command_topic"] = rx_topic;
    if (is_vat6) {
      doc["fan_mode_command_template"] =
          String("{\"ch\":") + ch +
          ",\"state\":\"on\""
          ",\"fan\":\"{{ value }}\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}"
          ",\"mode\":\"{{ this.state if this.state in ['cool','heat'] else 'cool' }}\"}";
    } else {
      doc["fan_mode_command_template"] =
          String("{\"ch\":") + ch +
          ",\"state\":\"on\""
          ",\"fan\":\"{{ value }}\""
          ",\"temp\":{{ this.attributes.temperature or 22 }}}";
    }

    size_t n = serializeJson(doc, buffer, sizeof(buffer));

    if (!client.publish(config_topic.c_str(), (const uint8_t*)buffer, n, true)) {
      Serial.print("Discovery publish failed for ch ");
      Serial.println(ch);
    }
  }

  Serial.println("HA MQTT discovery: published");
}