/**
 * @file wifi_ws_templates.h
 * @brief AtechWiFi — Multi-network WiFi with WebSocket gateway connection
 *
 * Header-only WiFi module for Atech IoT projects.
 * Provides priority-based multi-network connection, automatic reconnection,
 * and real-time event publishing over a persistent WebSocket to the gateway.
 *
 * Gateway WebSocket protocol:
 *   Send: {"type": "event", "payload": {"event_type": "...", "key": "...", "value": ...}}
 *   Recv: {"type": "event_ack"}
 *   Send: {"type": "ping"}
 *   Recv: {"type": "pong"}
 *   Recv: {"action": "...", "value": "..."} — commands from dashboard (dispatched to onMessage callback)
 */

#ifndef ATHERA_WIFI_WS_TEMPLATES_H
#define ATHERA_WIFI_WS_TEMPLATES_H

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>

struct WiFiNetwork {
    const char* ssid;
    const char* password;
    int priority;
};

class AtechWiFi {
public:
    /**
     * @brief Construct with gateway host, port, and project ID (used as device ID)
     * @param networks Array of WiFi networks to try
     * @param networkCount Number of networks
     * @param gatewayHost Gateway hostname (e.g. "gateway.atech.dev")
     * @param gatewayPort Gateway port (443 for WSS)
     * @param projectId Project UUID — used as device ID in /ws/{projectId}
     * @param useSSL Use wss:// (true) or ws:// (false)
     */
    AtechWiFi(WiFiNetwork* networks, int networkCount,
               const char* gatewayHost, int gatewayPort,
               const char* projectId, bool useSSL = true)
        : _networks(networks)
        , _networkCount(networkCount)
        , _gatewayHost(gatewayHost)
        , _gatewayPort(gatewayPort)
        , _projectId(projectId)
        , _useSSL(useSSL)

        , _wifiConnected(false)
        , _wsConnected(false)

        , _currentNetworkIndex(-1)
        , _lastReconnectAttempt(0)
        , _reconnectInterval(30000)
        , _eventsPosted(0)
        , _eventsAcked(0)
        , _connectionDrops(0)
        , _lastPing(0)
        , _lastHealthBeacon(0)
        , _bootCount(0)
        , _bootCountLoaded(false)
        , _gotIp(false)
        , _authFailedMask(0)
        , _lastDisconnectReason(0)
        , _eventHandlerRegistered(false)
        , _wsHandlerRegistered(false)
    {
    }

    // ========== WiFi Connection ==========

    bool connect() {
        _loadBootCount();
        _registerEventHandler();

        // Sort by priority (high to low)
        for (int i = 1; i < _networkCount; i++) {
            WiFiNetwork key = _networks[i];
            int j = i - 1;
            while (j >= 0 && _networks[j].priority < key.priority) {
                _networks[j + 1] = _networks[j];
                j--;
            }
            _networks[j + 1] = key;
        }

        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(false);

        for (int i = 0; i < _networkCount; i++) {
            // Skip SSIDs that AUTH_FAIL'd earlier this boot — they're not coming back
            // without new credentials, so retrying just delays the next viable network.
            if (i < 32 && (_authFailedMask & (1UL << i))) {
                Serial.printf("[Atech WiFi] Skipping '%s' (AUTH_FAIL earlier this boot)\n",
                              _networks[i].ssid);
                continue;
            }

            Serial.printf("[Atech WiFi] Trying: %s (priority %d)\n",
                          _networks[i].ssid, _networks[i].priority);

            // Set BEFORE WiFi.begin so the async event handler knows which SSID owns
            // any DISCONNECTED event that fires during/after this call.
            _currentNetworkIndex = i;
            _gotIp = false;

            if (_networks[i].password && _networks[i].password[0] != '\0') {
                WiFi.begin(_networks[i].ssid, _networks[i].password);
            } else {
                WiFi.begin(_networks[i].ssid);
            }

            // Wait on GOT_IP, not WL_CONNECTED. The latter flips true before DHCP
            // completes, so HTTP/WS attempts can fire against IP 0.0.0.0. Bump the
            // timeout to 15s to cover slow DHCP servers; early-exit on AUTH_FAIL
            // since that SSID won't recover without new credentials.
            unsigned long start = millis();
            while (!_gotIp && millis() - start < 15000) {
                delay(250);
                Serial.print(".");
                if (i < 32 && (_authFailedMask & (1UL << i))) break;
            }
            Serial.println();

            if (_gotIp) {
                _wifiConnected = true;
                _reconnectInterval = 30000;
                // F5: disable WiFi modem sleep AFTER connect succeeds.
                // esp_wifi_set_ps requires the driver to be started, which doesn't
                // happen reliably until after WiFi.begin() on arduino-esp32 v2.x.
                // Some drivers also reset power-save settings on esp_wifi_stop/start,
                // so we re-assert this on every successful reconnect.
                // Trade-off: ~50mA extra current for reliable packets on marginal APs.
                WiFi.setSleep(false);
                Serial.printf("[Atech WiFi] Connected to: %s\n", _networks[i].ssid);
                Serial.printf("[Atech WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
                _connectWebSocket();
                return true;
            }

            Serial.printf("[Atech WiFi] Failed: %s (last reason %d)\n",
                          _networks[i].ssid, _lastDisconnectReason);
            WiFi.disconnect();
        }

        Serial.println("[Atech WiFi] All networks failed!");
        _wifiConnected = false;
        return false;
    }

    /**
     * @brief Call every loop — handles WiFi reconnect and WebSocket maintenance
     */
    void maintain() {
        // WiFi check — driven by GOT_IP event, not raw WL_CONNECTED polling. This
        // closes the "WL_CONNECTED but DHCP not done" race that fired the WS handshake
        // against a 0.0.0.0 local IP.
        if (_gotIp) {
            if (!_wifiConnected) {
                _wifiConnected = true;
                _reconnectInterval = 30000;
                _connectWebSocket();
            }
        } else {
            if (_wifiConnected) {
                _wifiConnected = false;
                _wsConnected = false;
                _connectionDrops++;
                Serial.printf("[Atech WiFi] Connection lost (last reason %d)\n", _lastDisconnectReason);

                // F3 cleanup: stop the WS layer's own 3s reconnect loop while WiFi
                // is down. Otherwise it hammers handshakes against the dead network
                // stack and stacks lwIP errors. We'll rearm WS in _connectWebSocket()
                // after the next GOT_IP.
                _ws.disconnect();
            }

            unsigned long now = millis();
            if (now - _lastReconnectAttempt < _reconnectInterval) return;
            _lastReconnectAttempt = now;

            Serial.printf("[Atech WiFi] Reconnecting (backoff %lus)...\n", _reconnectInterval / 1000);
            if (connect()) return;
            _reconnectInterval = min(_reconnectInterval * 2, (unsigned long)300000);
            return;
        }

        // WebSocket loop
        _ws.loop();

        // Keepalive ping every 15s
        if (_wsConnected && millis() - _lastPing > 15000) {
            _lastPing = millis();
            _ws.sendTXT("{\"type\":\"ping\"}");
        }

        // Health beacon every 30s — telemetry for the reliability watcher
        if (_wsConnected && millis() - _lastHealthBeacon > HEALTH_INTERVAL_MS) {
            _lastHealthBeacon = millis();
            _sendHealth();
        }
    }

    // True only after DHCP completes (GOT_IP event). Stricter than the legacy
    // WiFi.status() == WL_CONNECTED check, which can be true before localIP() is valid.
    bool isConnected() { return _gotIp; }
    bool isWebSocketConnected() { return _wsConnected; }

    String getCurrentNetwork() {
        if (_currentNetworkIndex >= 0 && _currentNetworkIndex < _networkCount) {
            return String(_networks[_currentNetworkIndex].ssid);
        }
        return "";
    }

    String getIP() { return WiFi.localIP().toString(); }
    int getSignalStrength() { return WiFi.RSSI(); }
    const char* getGatewayHost() { return _gatewayHost; }
    const char* getProjectId() { return _projectId; }

    // ========== Incoming Messages ==========

    /**
     * @brief Register a callback for incoming commands from the gateway/dashboard
     *
     * Callback receives (action, value) extracted from incoming JSON.
     * Dashboard sends: {"type":"send_to_device","device_id":"...","payload":{"action":"...","value":"..."}}
     * Gateway forwards the payload to the device, and this callback fires with action + value.
     *
     * Example:
     *   wifi.onMessage([](const char* action, const char* value) {
     *     if (strcmp(action, "set_text") == 0) {
     *       strncpy(displayMessage, value, sizeof(displayMessage) - 1);
     *     }
     *   });
     */
    typedef void (*MessageCallback)(const char* action, const char* value);
    void onMessage(MessageCallback cb) { _messageCallback = cb; }

    // ========== Event Publishing (WebSocket) ==========

    bool postSensorEvent(const char* key, float value, const char* moduleType = "") {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "sensor";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        doc["payload"]["source"] = moduleType;
        return _sendEvent(doc);
    }

    bool postSensorEventInt(const char* key, int value, const char* moduleType = "") {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "sensor";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        doc["payload"]["source"] = moduleType;
        return _sendEvent(doc);
    }

    // String-valued sensor events — for multi-value readings packed as a string
    // (e.g. robot arm joint angles "0.5,0.3,-0.2,...") or structured JSON snippets.
    bool postSensorEventStr(const char* key, const char* value, const char* moduleType = "") {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "sensor";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        doc["payload"]["source"] = moduleType;
        return _sendEvent(doc);
    }

    bool postButtonEvent(const char* key, int value) {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "button";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        return _sendEvent(doc);
    }

    bool postStateEvent(const char* key, const char* value) {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "state";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        return _sendEvent(doc);
    }

    bool postLogEvent(const char* message) {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "log";
        doc["payload"]["key"] = "log";
        doc["payload"]["value"] = message;
        return _sendEvent(doc);
    }

    // ── String overloads ──────────────────────────────────────────────
    // Accept Arduino String / StringSumHelper so the natural codegen idiom
    // `postLogEvent("x=" + String(value))` compiles. (`const char* + String`
    // yields a StringSumHelper, which has no implicit conversion to const
    // char*.) String literals still bind to the const char* overloads above;
    // these forward via .c_str().
    bool postLogEvent(const String& message) { return postLogEvent(message.c_str()); }
    bool postStateEvent(const char* key, const String& value) { return postStateEvent(key, value.c_str()); }
    bool postSensorEventStr(const char* key, const String& value, const char* moduleType = "") {
        return postSensorEventStr(key, value.c_str(), moduleType);
    }

    // ========== Statistics ==========

    int getEventsPosted() { return _eventsPosted; }
    int getEventsAcked() { return _eventsAcked; }
    int getConnectionDrops() { return _connectionDrops; }

    String getStats() {
        JsonDocument doc;
        doc["events_posted"] = _eventsPosted;
        doc["events_acked"] = _eventsAcked;
        doc["connection_drops"] = _connectionDrops;
        doc["wifi_connected"] = isConnected();
        doc["ws_connected"] = _wsConnected;
        if (_currentNetworkIndex >= 0) {
            doc["network"] = _networks[_currentNetworkIndex].ssid;
        }
        doc["rssi"] = WiFi.RSSI();
        String out;
        serializeJson(doc, out);
        return out;
    }

private:
    WiFiNetwork* _networks;
    int _networkCount;
    const char* _gatewayHost;
    int _gatewayPort;
    const char* _projectId;
    bool _useSSL;


    WebSocketsClient _ws;
    bool _wifiConnected;
    bool _wsConnected;
    int _currentNetworkIndex;
    unsigned long _lastReconnectAttempt;
    unsigned long _reconnectInterval;
    unsigned long _lastPing;
    unsigned long _lastHealthBeacon;
    uint32_t _bootCount;
    bool _bootCountLoaded;
    static constexpr unsigned long HEALTH_INTERVAL_MS = 30000;

    // Event-driven WiFi state. Written from the WiFi task's event handler, read
    // from the Arduino loop. `volatile` prevents the compiler from caching/reordering
    // reads; aligned word stores don't tear on Xtensa, so no atomic type is needed
    // for these single-flag transitions. Not a full memory barrier — we rely on
    // the external WiFi.begin/disconnect calls for ordering between writes in
    // connect() and reads in the event handler.
    volatile bool _gotIp;
    volatile uint32_t _authFailedMask;   // bit i = network i has AUTH_FAIL'd this boot; cap 32 SSIDs
    volatile int _lastDisconnectReason;  // last STA_DISCONNECTED reason code (for diagnostics)
    bool _eventHandlerRegistered;        // register-once guard for WiFi.onEvent
    bool _wsHandlerRegistered;           // register-once guard for _ws.onEvent (F4: prevents stacked callbacks)

    int _eventsPosted;
    int _eventsAcked;
    int _connectionDrops;
    MessageCallback _messageCallback = nullptr;

    void _connectWebSocket() {
        // F4: tear down any prior session before begin(). WebSocketsClient's
        // beginSSL doesn't release the previous WiFiClientSecure context on its
        // own, so calling it twice in a row leaks the TLS state. _ws.disconnect()
        // forces the cleanup. No-op on the first call (already disconnected).
        _ws.disconnect();

        String path = String("/ws/") + _projectId;
        Serial.printf("[Atech WS] Connecting to %s://%s:%d%s\n",
                      _useSSL ? "wss" : "ws", _gatewayHost, _gatewayPort, path.c_str());

        if (_useSSL) {
            _ws.beginSSL(_gatewayHost, _gatewayPort, path.c_str());
        } else {
            _ws.begin(_gatewayHost, _gatewayPort, path.c_str());
        }

        // F4: register the event handler exactly once. WebSocketsClient stacks
        // callbacks if onEvent is called repeatedly — old lambdas leak and fire
        // alongside new ones.
        if (!_wsHandlerRegistered) {
            _ws.onEvent([this](WStype_t type, uint8_t* payload, size_t length) {
                _onWebSocketEvent(type, payload, length);
            });
            _wsHandlerRegistered = true;
        }
        _ws.setReconnectInterval(3000);
    }

    void _onWebSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
        switch (type) {
            case WStype_CONNECTED:
                Serial.printf("[Atech WS] Connected to gateway (/ws/%s)\n", _projectId);
                _wsConnected = true;
                _lastHealthBeacon = millis();
                _sendHealth();
                break;

            case WStype_DISCONNECTED:
                // Include the truncated peer string the WS library hands us in `payload`
                // when available — distinguishes "TLS handshake failed" from "server
                // closed cleanly" from "no route to host" without needing tethered logs.
                if (payload && length > 0) {
                    Serial.printf("[Atech WS] Disconnected: %.*s\n", (int)length, (const char*)payload);
                } else {
                    Serial.println("[Atech WS] Disconnected from gateway");
                }
                _wsConnected = false;
                break;

            case WStype_TEXT: {
                JsonDocument doc;
                deserializeJson(doc, payload, length);
                const char* msgType = doc["type"];
                if (msgType && strcmp(msgType, "event_ack") == 0) {
                    _eventsAcked++;
                } else if (msgType && strcmp(msgType, "pong") == 0) {
                    // Keepalive response — ignore
                } else {
                    // Command from dashboard — extract action + value with fallbacks.
                    // Scalars (string/number/bool) pass through as text; objects and
                    // arrays are serialized back to a JSON substring so structured
                    // actions like set_color {"r":255,"g":0,"b":0} survive the trip
                    // to the handler instead of collapsing to "".
                    const char* action = doc["action"] | doc["type"] | "";
                    JsonVariantConst v = doc["value"];
                    if (v.isNull()) v = doc["message"];
                    if (v.isNull()) v = doc["payload"]["message"];
                    String valueStr;
                    if (v.is<const char*>()) {
                        const char* s = v.as<const char*>();
                        if (s) valueStr = s;
                    } else if (v.is<JsonObjectConst>() || v.is<JsonArrayConst>()) {
                        serializeJson(v, valueStr);
                    } else if (!v.isNull()) {
                        valueStr = v.as<String>();
                    }
                    // Diagnostic: every incoming command is logged. When "wifi can't
                    // control the board," the user can now tell from the serial output
                    // whether (a) nothing is arriving at the device, (b) it arrives but
                    // no handler is registered, or (c) the handler is firing but the
                    // action name doesn't match any case. Each is a different fix.
                    if (action[0] != '\0') {
                        if (_messageCallback) {
                            Serial.printf("[Atech WS] action='%s' value='%s' -> handler\n",
                                          action, valueStr.c_str());
                            _messageCallback(action, valueStr.c_str());
                        } else {
                            Serial.printf("[Atech WS] action='%s' but no onMessage handler registered (call wifi.onMessage in setup)\n",
                                          action);
                        }
                    } else if (length > 0) {
                        Serial.printf("[Atech WS] Inbound message has no action key: %.*s\n",
                                      (int)length, (const char*)payload);
                    }
                }
                break;
            }

            case WStype_ERROR:
                // Surface the WS library's error string — without it every failure
                // (TLS, DNS, refused, malformed frame) collapses to the same "Error"
                // line and the user can't tell why control isn't working.
                if (payload && length > 0) {
                    Serial.printf("[Atech WS] Error: %.*s\n", (int)length, (const char*)payload);
                } else {
                    Serial.println("[Atech WS] Error (no detail)");
                }
                break;

            default:
                break;
        }
    }

    bool _sendEvent(JsonDocument& doc) {
        if (!_wsConnected) return false;

        String msg;
        serializeJson(doc, msg);
        _ws.sendTXT(msg);
        _eventsPosted++;
        return true;
    }

    // Register the WiFi event handler exactly once. The handler runs in the WiFi
    // task context and is the source of truth for connection state — no more polling
    // WiFi.status() in tight loops, no more racing DHCP.
    void _registerEventHandler() {
        if (_eventHandlerRegistered) return;
        WiFi.onEvent([this](arduino_event_id_t event, arduino_event_info_t info) {
            _onWifiEvent(event, info);
        });
        _eventHandlerRegistered = true;
    }

    // Event handler. Runs in the WiFi task context — writes here are observed by
    // maintain() / connect() via the volatile fields above.
    void _onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
        switch (event) {
            case ARDUINO_EVENT_WIFI_STA_GOT_IP:
                _gotIp = true;
                _lastDisconnectReason = 0;
                Serial.printf("[Atech WiFi] GOT_IP %s\n", WiFi.localIP().toString().c_str());
                break;

            case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
                _gotIp = false;
                int reason = info.wifi_sta_disconnected.reason;
                _lastDisconnectReason = reason;
                Serial.printf("[Atech WiFi] DISCONNECTED reason=%d\n", reason);

                // AUTH_FAIL is terminal for this SSID until reboot/new creds — no point
                // hammering the AP with the same wrong password every backoff window.
                // Other disconnect reasons (AP_NOT_FOUND, BEACON_TIMEOUT, etc.) are
                // transient and DO retry.
                if (reason == WIFI_REASON_AUTH_FAIL) {
                    // Match against the FAILED SSID from the event payload — not
                    // _currentNetworkIndex. The index can advance before this
                    // async event is delivered (e.g. SSID #0 fails and we've
                    // already moved to SSID #1 by the time AUTH_FAIL arrives for
                    // #0). Trusting the index would mark the wrong SSID dead.
                    const uint8_t* failedSsid = info.wifi_sta_disconnected.ssid;
                    uint8_t failedLen = info.wifi_sta_disconnected.ssid_len;
                    for (int j = 0; j < _networkCount && j < 32; j++) {
                        const char* netSsid = _networks[j].ssid;
                        if (!netSsid) continue;
                        size_t netLen = strlen(netSsid);
                        if (netLen == failedLen && memcmp(failedSsid, netSsid, netLen) == 0) {
                            _authFailedMask |= (1UL << j);
                            Serial.printf("[Atech WiFi] AUTH_FAIL — '%s' marked dead until reboot\n",
                                          netSsid);
                            break;
                        }
                    }
                }
                break;
            }

            default:
                break;
        }
    }

    // Bumped once per boot, persisted to NVS. Lets the reliability watcher
    // detect boot loops (boot_count rising fast = device is crashing).
    void _loadBootCount() {
        if (_bootCountLoaded) return;
        Preferences prefs;
        prefs.begin("atech_wifi", false);
        _bootCount = prefs.getUInt("boots", 0) + 1;
        prefs.putUInt("boots", _bootCount);
        prefs.end();
        _bootCountLoaded = true;
    }

    // Periodic telemetry. Free heap trend reveals leaks; RSSI explains
    // disconnects; reconnect_count + boot_count surface fleet patterns.
    void _sendHealth() {
        if (!_wsConnected) return;
        JsonDocument doc;
        doc["type"] = "health";
        doc["uptime_s"] = millis() / 1000;
        doc["free_heap"] = ESP.getFreeHeap();
        doc["min_free_heap"] = ESP.getMinFreeHeap();
        doc["rssi"] = WiFi.RSSI();
        doc["wifi_state"] = _wifiConnected ? "connected" : "disconnected";
        doc["ws_state"] = _wsConnected ? "connected" : "disconnected";
        doc["reconnect_count"] = _connectionDrops;
        doc["boot_count"] = _bootCount;
        String out;
        serializeJson(doc, out);
        _ws.sendTXT(out);
    }
};

#endif // ATHERA_WIFI_WS_TEMPLATES_H
