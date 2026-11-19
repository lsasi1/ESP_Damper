#include "web.h"
#include "config.h"
#include "debug_log.h"
#include "serial_intercept.h"
#include <WiFi.h>
#include <ArduinoJson.h>

extern const char* FW_VERSION_STR;

static PubSubClient* mqtt_ref = nullptr;
static unsigned long boot_ms = 0;

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP Damper Control</title>
<style>
  :root { --bg: #0f172a; --card: #1e293b; --text: #f8fafc; --muted: #94a3b8; --accent: #38bdf8; --border: #334155; }
  * { box-sizing: border-box; }
  body { font-family: system-ui, -apple-system, sans-serif; background: var(--bg); color: var(--text); margin: 0; padding: 16px; }
  .container { max-width: 720px; margin: 0 auto; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 20px; margin-bottom: 16px; }
  h1 { margin: 0 0 4px; font-size: 1.4rem; }
  h2 { margin: 0 0 14px; font-size: 1.1rem; color: var(--accent); border-bottom: 1px solid var(--border); padding-bottom: 8px; }
  .sub { color: var(--muted); font-size: 0.85rem; margin-bottom: 16px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
  @media (max-width: 560px) { .grid { grid-template-columns: 1fr; } }
  label { display: block; font-size: 0.85rem; color: var(--muted); margin-bottom: 4px; }
  input[type="text"], input[type="password"], input[type="number"], select {
    width: 100%; padding: 8px 10px; border-radius: 6px; border: 1px solid var(--border);
    background: #0f172a; color: var(--text); font-size: 0.95rem;
  }
  .chk { display: flex; align-items: center; gap: 8px; margin: 8px 0; cursor: pointer; }
  .chk input { width: 16px; height: 16px; }
  .btn { background: var(--accent); color: #0f172a; border: none; padding: 10px 18px; border-radius: 8px; font-weight: 600; cursor: pointer; font-size: 0.95rem; }
  .btn:hover { opacity: 0.9; }
  .btn-secondary { background: #334155; color: var(--text); text-decoration: none; display: inline-block; margin-left: 8px; }
  .btn-sm { padding: 5px 12px; font-size: 0.8rem; }
  .status-bar { display: flex; flex-wrap: wrap; gap: 10px; font-size: 0.85rem; color: var(--muted); }
  .badge { background: #0f172a; padding: 4px 10px; border-radius: 999px; border: 1px solid var(--border); }
  .log-hdr { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid var(--border); padding-bottom: 8px; margin-bottom: 12px; }
  .log-hdr h2 { border: none; padding: 0; margin: 0; }
  #logBox { background: #090d16; color: #a5f3fc; font-family: ui-monospace, monospace; font-size: 0.8rem; height: 260px; overflow-y: auto; padding: 10px; border-radius: 8px; border: 1px solid var(--border); white-space: pre-wrap; }
</style>
</head>
<body>
<div class="container">
  <div class="card">
    <h1>ESP Damper Controller</h1>
    <div class="sub">Firmware: <span id="fwVer">...</span> | Free Heap: <span id="heapFree">...</span> KB</div>
    <div class="status-bar">
      <span class="badge">WiFi: <strong id="stWifi">...</strong></span>
      <span class="badge">IP: <strong id="stIp">...</strong></span>
      <span class="badge">MQTT: <strong id="stMqtt">...</strong></span>
      <span class="badge">Uptime: <strong id="stUp">...</strong></span>
    </div>
  </div>

  <form method="POST" action="/save">
    <div class="card">
      <h2>Device & AC Model</h2>
      <div class="grid">
        <div>
          <label for="device_name">Device Name</label>
          <input type="text" id="device_name" name="device_name" required>
        </div>
        <div>
          <label for="ac_model">AC Protocol Model</label>
          <select id="ac_model" name="ac_model">
            <option value="tac910">TAC-910</option>
            <option value="vat6">VAT-6</option>
          </select>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>WiFi Settings</h2>
      <div class="grid">
        <div>
          <label for="wifi_ssid">WiFi SSID</label>
          <input type="text" id="wifi_ssid" name="wifi_ssid">
        </div>
        <div>
          <label for="wifi_pass">WiFi Password (leave blank to keep)</label>
          <input type="password" id="wifi_pass" name="wifi_pass" placeholder="••••••••">
        </div>
      </div>
    </div>

    <div class="card">
      <h2>MQTT & Home Assistant</h2>
      <div class="grid">
        <div>
          <label for="mqtt_server">Broker IP / Host</label>
          <input type="text" id="mqtt_server" name="mqtt_server">
        </div>
        <div>
          <label for="mqtt_port">Broker Port</label>
          <input type="number" id="mqtt_port" name="mqtt_port" value="1883">
        </div>
        <div>
          <label for="mqtt_user">MQTT Username</label>
          <input type="text" id="mqtt_user" name="mqtt_user">
        </div>
        <div>
          <label for="mqtt_pass">MQTT Password (leave blank to keep)</label>
          <input type="password" id="mqtt_pass" name="mqtt_pass" placeholder="••••••••">
        </div>
        <div>
          <label for="mqtt_topic_rx">Command Topic (RX)</label>
          <input type="text" id="mqtt_topic_rx" name="mqtt_topic_rx">
        </div>
        <div>
          <label for="mqtt_topic_tx">State Base Topic (TX)</label>
          <input type="text" id="mqtt_topic_tx" name="mqtt_topic_tx">
        </div>
      </div>
      <div style="margin-top:12px;">
        <label class="chk"><input type="checkbox" id="enable_discovery" name="enable_discovery" value="1"> Enable Home Assistant MQTT Discovery</label>
      </div>
    </div>

    <div class="card">
      <h2>Diagnostics & Debug</h2>
      <label class="chk"><input type="checkbox" id="debug_verbose" name="debug_verbose" value="1"> Verbose Serial / MQTT Frame Logging</label>
      <label class="chk"><input type="checkbox" id="timing_diag" name="timing_diag" value="1"> RX/ACK Timing Diagnostics</label>
      <div style="margin-top:14px;">
        <button type="submit" class="btn">Save & Reboot</button>
        <a href="/update" class="btn btn-secondary">Firmware OTA Update</a>
        <button type="button" class="btn btn-secondary" onclick="fetch('/reboot',{method:'POST'}).then(()=>alert('Rebooting...'))">Reboot</button>
      </div>
    </div>
  </form>

  <div class="card">
    <div class="log-hdr">
      <h2>Live Serial Log</h2>
      <button type="button" class="btn btn-secondary btn-sm" onclick="clearLogs()">Clear</button>
    </div>
    <div id="logBox">Loading logs...</div>
  </div>
</div>

<script>
function formatLogItem(item) {
  if (typeof item === 'string') return item;
  if (item && typeof item === 'object') {
    const msg = item.msg ?? item.message ?? item.line ?? item.text ?? item.m ?? JSON.stringify(item);
    const ts = item.ts ?? item.t ?? item.time;
    return ts !== undefined ? `[${ts}] ${msg}` : String(msg);
  }
  return String(item ?? '');
}

async function loadConfig() {
  try {
    const r = await fetch('/config');
    const c = await r.json();
    document.getElementById('fwVer').textContent = c.fw_version || '';
    document.getElementById('device_name').value = c.device_name || '';
    document.getElementById('ac_model').value = c.ac_model || 'tac910';
    document.getElementById('wifi_ssid').value = c.wifi_ssid || '';
    document.getElementById('mqtt_server').value = c.mqtt_server || '';
    document.getElementById('mqtt_port').value = c.mqtt_port || 1883;
    document.getElementById('mqtt_user').value = c.mqtt_user || '';
    document.getElementById('mqtt_topic_rx').value = c.mqtt_topic_rx || '';
    document.getElementById('mqtt_topic_tx').value = c.mqtt_topic_tx || '';
    document.getElementById('enable_discovery').checked = !!c.enable_discovery;
    document.getElementById('debug_verbose').checked = !!c.debug_verbose;
    document.getElementById('timing_diag').checked = !!c.timing_diag;
  } catch (e) { console.error(e); }
}

async function pollStatus() {
  try {
    const r = await fetch('/status');
    const s = await r.json();
    document.getElementById('stWifi').textContent = s.wifi || '-';
    document.getElementById('stIp').textContent = s.ip || '-';
    document.getElementById('stMqtt').textContent = s.mqtt || '-';
    document.getElementById('stUp').textContent = (s.uptime || 0) + 's';
    if (s.heap_kb !== undefined) document.getElementById('heapFree').textContent = s.heap_kb;
  } catch (e) {}
}

async function pollLogs() {
  try {
    const r = await fetch('/logs');
    const data = await r.json();
    const box = document.getElementById('logBox');
    const atBottom = box.scrollHeight - box.scrollTop <= box.clientHeight + 30;
    const rawLines = Array.isArray(data) ? data : (data.lines || data.logs || []);
    box.textContent = rawLines.map(formatLogItem).join('\n');
    if (atBottom) box.scrollTop = box.scrollHeight;
  } catch (e) {}
}

async function clearLogs() {
  try {
    await fetch('/logs/clear', { method: 'POST' });
    document.getElementById('logBox').textContent = '';
  } catch (e) {}
}

loadConfig();
pollStatus();
pollLogs();
setInterval(pollStatus, 3000);
setInterval(pollLogs, 2000);
</script>
</body>
</html>
)rawliteral";

void web_begin(AsyncWebServer &server, PubSubClient &mqttClient) {
  mqtt_ref = &mqttClient;
  boot_ms = millis();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    web_log_mark_ui_active_and_clear_if_needed();
    request->send(200, "text/html", INDEX_HTML);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["fw_version"]       = FW_VERSION_STR;
    doc["device_name"]      = device_config.device_name;
    doc["ac_model"]         = device_config.ac_model;
    doc["wifi_ssid"]        = device_config.wifi_ssid;
    doc["mqtt_server"]      = device_config.mqtt_server;
    doc["mqtt_port"]        = device_config.mqtt_port;
    doc["mqtt_user"]        = device_config.mqtt_user;
    doc["mqtt_topic_rx"]    = device_config.mqtt_topic_rx;
    doc["mqtt_topic_tx"]    = device_config.mqtt_topic_tx;
    doc["enable_discovery"] = device_config.enable_discovery;
    doc["debug_verbose"]    = device_config.debug_verbose;
    doc["timing_diag"]      = device_config.timing_diagnostics_en;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    if (WiFi.getMode() == WIFI_AP) {
      doc["wifi"] = "AP Mode";
      doc["ip"]   = WiFi.softAPIP().toString();
    } else if (WiFi.status() == WL_CONNECTED) {
      doc["wifi"] = "Connected (" + String(WiFi.RSSI()) + " dBm)";
      doc["ip"]   = WiFi.localIP().toString();
    } else {
      doc["wifi"] = "Disconnected";
      doc["ip"]   = "0.0.0.0";
    }

    doc["mqtt"]    = (mqtt_ref && mqtt_ref->connected()) ? "Connected" : "Disconnected";
    doc["uptime"]  = (millis() - boot_ms) / 1000;
    doc["heap_kb"] = ESP.getFreeHeap() / 1024;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
    web_log_mark_ui_active_and_clear_if_needed();
    request->send(200, "application/json", web_log_json());
  });

  server.on("/logs/clear", HTTP_POST, [](AsyncWebServerRequest *request) {
    web_log_clear();
    request->send(200, "text/plain", "OK");
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("device_name", true)) {
      strlcpy(device_config.device_name, request->getParam("device_name", true)->value().c_str(), sizeof(device_config.device_name));
    }
    if (request->hasParam("ac_model", true)) {
      strlcpy(device_config.ac_model, request->getParam("ac_model", true)->value().c_str(), sizeof(device_config.ac_model));
    }
    if (request->hasParam("wifi_ssid", true)) {
      strlcpy(device_config.wifi_ssid, request->getParam("wifi_ssid", true)->value().c_str(), sizeof(device_config.wifi_ssid));
    }
    if (request->hasParam("wifi_pass", true)) {
      String p = request->getParam("wifi_pass", true)->value();
      if (p.length() > 0) {
        strlcpy(device_config.wifi_pass, p.c_str(), sizeof(device_config.wifi_pass));
      }
    }
    if (request->hasParam("mqtt_server", true)) {
      strlcpy(device_config.mqtt_server, request->getParam("mqtt_server", true)->value().c_str(), sizeof(device_config.mqtt_server));
    }
    if (request->hasParam("mqtt_port", true)) {
      int port = request->getParam("mqtt_port", true)->value().toInt();
      if (port > 0 && port <= 65535) device_config.mqtt_port = (uint16_t)port;
    }
    if (request->hasParam("mqtt_user", true)) {
      strlcpy(device_config.mqtt_user, request->getParam("mqtt_user", true)->value().c_str(), sizeof(device_config.mqtt_user));
    }
    if (request->hasParam("mqtt_pass", true)) {
      String p = request->getParam("mqtt_pass", true)->value();
      if (p.length() > 0) {
        strlcpy(device_config.mqtt_pass, p.c_str(), sizeof(device_config.mqtt_pass));
      }
    }
    if (request->hasParam("mqtt_topic_rx", true)) {
      strlcpy(device_config.mqtt_topic_rx, request->getParam("mqtt_topic_rx", true)->value().c_str(), sizeof(device_config.mqtt_topic_rx));
    }
    if (request->hasParam("mqtt_topic_tx", true)) {
      strlcpy(device_config.mqtt_topic_tx, request->getParam("mqtt_topic_tx", true)->value().c_str(), sizeof(device_config.mqtt_topic_tx));
    }

    device_config.enable_discovery      = request->hasParam("enable_discovery", true);
    device_config.debug_verbose         = request->hasParam("debug_verbose", true);
    device_config.timing_diagnostics_en = request->hasParam("timing_diag", true);

    config_save();

    request->send(200, "text/html",
      "<html><body style='background:#0f172a;color:#f8fafc;font-family:sans-serif;padding:24px;'>"
      "<h2>Settings Saved! Rebooting...</h2>"
      "<script>setTimeout(()=>location.href='/', 4000);</script>"
      "</body></html>");

    delay(500);
    ESP.restart();
  });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Rebooting");
    delay(300);
    ESP.restart();
  });
}