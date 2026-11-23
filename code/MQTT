/*
 * nichoir_mqtt.ino — M5Stack Timer Camera (ESP32 + OV3660 + PSRAM)
 * Prend une photo JPEG toutes les CAPTURE_INTERVAL_S secondes et la publie
 * en binaire brut sur le topic MQTT "nichoir/photo" (broker Mosquitto sur RPi 5).
 *
 * Carte : M5Stack > M5TimerCAM (FQBN m5stack:esp32:timer_cam)
 * Libs  : Timer-CAM (M5Stack), PubSubClient (Nick O'Leary)
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include "M5TimerCAM.h"

// ------------------------- A ADAPTER -------------------------
const char* WIFI_SSID     = "electroProjectWifi";      // 2,4 GHz uniquement !
const char* WIFI_PASSWORD = "B1MesureEnv";
const char* MQTT_HOST     = "192.168.2.106";      // IP du Raspberry Pi 5
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = nullptr;             // ou "user" si Mosquitto en a besoin
const char* MQTT_PASS     = nullptr;
const char* TOPIC_PHOTO   = "nichoir/photo";
const char* TOPIC_STATUS  = "nichoir/status";
const uint32_t CAPTURE_INTERVAL_S = 30;          // une photo toutes les N secondes
const size_t   CHUNK_SIZE         = 1024;        // taille des morceaux envoyés
// -------------------------------------------------------------

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
String       clientId;
uint32_t     lastCapture = 0;

// ---------- WiFi (mode STATION) ----------
bool connectWiFi(uint32_t timeoutMs = 20000) {
    if (WiFi.status() == WL_CONNECTED) return true;
    Serial.printf("WiFi: connexion a %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);              // meilleur debit pendant l'envoi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
        delay(500);
        Serial.print('.');
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(" ECHEC");
        WiFi.disconnect(true);
        return false;
    }
    Serial.printf(" OK, IP=%s RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
}

// ---------- MQTT ----------
bool connectMQTT() {
    if (mqtt.connected()) return true;
    Serial.printf("MQTT: connexion a %s:%u ... ", MQTT_HOST, MQTT_PORT);
    // Last Will : le broker publie "offline" si la camera disparait
    bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS,
                           TOPIC_STATUS, 1, true, "offline");
    if (ok) {
        Serial.println("OK");
        mqtt.publish(TOPIC_STATUS, "online", true);
    } else {
        Serial.printf("ECHEC rc=%d\n", mqtt.state());
    }
    return ok;
}

// Reconnexion non bloquante (au plus une tentative toutes les 5 s)
bool ensureConnected() {
    static uint32_t lastTry = 0;
    if (WiFi.status() == WL_CONNECTED && mqtt.connected()) return true;
    if (millis() - lastTry < 5000) return false;
    lastTry = millis();
    if (!connectWiFi()) return false;
    return connectMQTT();
}

// ---------- Camera ----------
bool initCamera() {
    if (!TimerCAM.Camera.begin()) {        // config interne : JPEG, PSRAM, 2 fb
        Serial.println("Camera: init ECHEC");
        return false;
    }
    sensor_t* s = TimerCAM.Camera.sensor;
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_framesize(s, FRAMESIZE_SVGA);   // 800x600 : ~30-60 ko. UXGA=1600x1200 max
    s->set_quality(s, 12);                 // 0-63, plus petit = meilleure qualite
    s->set_vflip(s, 1);                    // selon l'orientation dans le nichoir
    s->set_hmirror(s, 0);
    Serial.println("Camera: OK");
    return true;
}

// Publication du JPEG par morceaux : pas de buffer MQTT geant en RAM
bool publishPhoto(const uint8_t* buf, size_t len) {
    if (!mqtt.beginPublish(TOPIC_PHOTO, len, false)) {
        Serial.println("MQTT: beginPublish ECHEC");
        return false;
    }
    size_t sent = 0;
    while (sent < len) {
        size_t n = min(CHUNK_SIZE, len - sent);
        size_t w = mqtt.write(buf + sent, n);
        if (w == 0) {                       // socket bloquee/coupee
            Serial.printf("MQTT: write ECHEC a %u/%u\n", sent, len);
            mqtt.disconnect();              // flux corrompu -> on repart propre
            return false;
        }
        sent += w;
        yield();                            // laisse tourner la pile WiFi/watchdog
    }
    if (!mqtt.endPublish()) {
        Serial.println("MQTT: endPublish ECHEC");
        return false;
    }
    return true;
}

bool captureAndSend() {
    // Jette une image : avec fb_count=2 la premiere peut etre ancienne
    if (TimerCAM.Camera.get()) TimerCAM.Camera.free();

    if (!TimerCAM.Camera.get()) {
        Serial.println("Camera: capture ECHEC");
        return false;
    }
    camera_fb_t* fb = TimerCAM.Camera.fb;
    TimerCAM.Power.setLed(64);
    uint32_t t0 = millis();
    bool ok = false;
    for (int essai = 1; essai <= 3 && !ok; essai++) {
        if (essai > 1) {
            Serial.printf("Nouvel essai %d/3...\n", essai);
            mqtt.disconnect();
            wifiClient.stop();
            delay(500);
            if (!connectWiFi() || !connectMQTT()) continue;
        }
        ok = publishPhoto(fb->buf, fb->len);
    }
    Serial.printf("Photo %ux%u, %u octets -> %s (%lu ms)\n",
                  fb->width, fb->height, fb->len, ok ? "OK" : "ECHEC",
                  (unsigned long)(millis() - t0));
    TimerCAM.Camera.free();                 // TOUJOURS liberer le framebuffer
    TimerCAM.Power.setLed(0);
    return ok;
}

// ---------- Arduino ----------
void setup() {
    TimerCAM.begin(true);                   // maintien alim batterie + RTC + Serial 115200
    delay(200);
    Serial.println("\n=== Nichoir Timer Camera MQTT ===");
    Serial.printf("PSRAM: %s (%u octets libres)\n",
                  psramFound() ? "OK" : "ABSENTE !", ESP.getFreePsram());
    Serial.printf("Batterie: %d mV\n", TimerCAM.Power.getBatteryVoltage());

    if (!initCamera()) {
        delay(3000);
        ESP.restart();
    }

    clientId = "nichoir-cam-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setKeepAlive(60);                  // s
    mqtt.setSocketTimeout(5);              // s

    connectWiFi();
    connectMQTT();
    lastCapture = millis() - CAPTURE_INTERVAL_S * 1000UL;   // 1re photo tout de suite
}

void loop() {
    if (ensureConnected()) {
        mqtt.loop();                        // keepalive / PINGREQ
        if (millis() - lastCapture >= CAPTURE_INTERVAL_S * 1000UL) {
            lastCapture = millis();
            captureAndSend();
        }
    }
    delay(10);
}
