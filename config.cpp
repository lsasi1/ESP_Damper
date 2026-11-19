#include "config.h"
#include "globals.h"
#include <Preferences.h>
#include <cstring>

Preferences prefs;
Preferences state_prefs;
device_config_t device_config;

// משתנים לניהול השהיית שמירה (Debounce) לכל ערוץ
static unsigned long last_change_time[NUM_CHANNELS] = {0};
static bool save_pending[NUM_CHANNELS] = {false};
const unsigned long NVS_SAVE_DELAY_MS = 5000; // השהיה של 5 שניות

void config_set_defaults() {
  strcpy(device_config.device_name, "ESP Damper");

  strcpy(device_config.wifi_ssid, "");
  strcpy(device_config.wifi_pass, "");

  strcpy(device_config.mqtt_server, "192.168.1.123");
  device_config.mqtt_port = 1883;

  strcpy(device_config.mqtt_topic_rx, "/home/damper_cmd");
  strcpy(device_config.mqtt_topic_tx, "/home/damper");

  device_config.dark_mode = false;

  device_config.mqtt_user[0] = '\0';
  device_config.mqtt_pass[0] = '\0';

  device_config.debug_verbose = false;
  device_config.rx_ack_en = true;
  device_config.tx_ack_en = true;

  strcpy(device_config.ac_model, "tac910");
  device_config.enable_discovery = true;
  device_config.timing_diagnostics_en = false;
}

void config_begin() {
  prefs.begin("config", false);
}

void config_load() {
  if (!prefs.getBool("valid", false)) {
    config_set_defaults();
    config_save();
    return;
  }

  config_set_defaults();
  prefs.getBytes("cfg", &device_config, sizeof(device_config));

  if (device_config.ac_model[0] == '\0') {
    strcpy(device_config.ac_model, "tac910");
    config_save();
  }

  if (strcmp(device_config.ac_model, "tac910") != 0 &&
      strcmp(device_config.ac_model, "vat6") != 0) {
    strcpy(device_config.ac_model, "tac910");
    config_save();
  }

  if (device_config.mqtt_port == 0) {
    device_config.mqtt_port = 1883;
    config_save();
  }
}

void config_save() {
  prefs.putBytes("cfg", &device_config, sizeof(device_config));
  prefs.putBool("valid", true);
}

void config_clear() {
  prefs.clear();
  prefs.putBool("valid", false);
  state_prefs.clear();
}

// ==========================================
// Persistent State Storage Implementation (Debounced)
// ==========================================

void states_storage_init() {
  state_prefs.begin("damper_st", false);
  states_load_all();
}

void states_load_all() {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
    char key_t[16], key_s[16], key_f[16], key_m[16];
    snprintf(key_t, sizeof(key_t), "ch%d_t", ch);
    snprintf(key_s, sizeof(key_s), "ch%d_s", ch);
    snprintf(key_f, sizeof(key_f), "ch%d_f", ch);
    snprintf(key_m, sizeof(key_m), "ch%d_m", ch);

    mqtt_data[ch].ch = ch;
    mqtt_data[ch].temp = state_prefs.getUChar(key_t, 22);
    mqtt_data[ch].state = state_prefs.getString(key_s, "off");
    mqtt_data[ch].fan = state_prefs.getUChar(key_f, 0);
    mqtt_data[ch].mode = state_prefs.getString(key_m, "auto");
    mqtt_data[ch].pending = false;
    save_pending[ch] = false;
  }
}

// תיוג בקשת שמירה עם Debounce (לא כותב מיד לזיכרון)
void state_save_channel(uint8_t ch) {
  if (ch >= NUM_CHANNELS) return;
  last_change_time[ch] = millis();
  save_pending[ch] = true;
}

// פונקציה הרצה ברקע בלולאה ומבצעת את הכתיבה ל-NVS רק אחרי שקט של 5 שניות
void states_handle_deferred_save() {
  unsigned long now = millis();
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
    if (save_pending[ch] && (now - last_change_time[ch] >= NVS_SAVE_DELAY_MS)) {
      save_pending[ch] = false;

      char key_t[16], key_s[16], key_f[16], key_m[16];
      snprintf(key_t, sizeof(key_t), "ch%d_t", ch);
      snprintf(key_s, sizeof(key_s), "ch%d_s", ch);
      snprintf(key_f, sizeof(key_f), "ch%d_f", ch);
      snprintf(key_m, sizeof(key_m), "ch%d_m", ch);

      state_prefs.putUChar(key_t, mqtt_data[ch].temp);
      state_prefs.putString(key_s, mqtt_data[ch].state);
      state_prefs.putUChar(key_f, mqtt_data[ch].fan);
      state_prefs.putString(key_m, mqtt_data[ch].mode);

      if (device_config.debug_verbose) {
        Serial.printf("NVS debounced save executed for channel %d\n", ch);
      }
    }
  }
}