#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoOTA.h>
#include <BH1750.h>
#include "DHT.h"

// ===================== PIN / PWM =====================
#define DHTPIN 4
#define BUZZER 45

#define PIR_PIN 14
#define LED_PIN 15

#define I2C_SDA 16
#define I2C_SCL 17

#define INA 18
#define INB 19

#define PWM_A 0
#define PWM_B 1
#define PWM_BUZZER 2
#define PWM_FREQ 5000
#define BUZZER_PWM_FREQ 2000
#define PWM_RES 8
#define FAN_SPEED 200
#define BUZZER_MED_DUTY 128
#define BUZZER_MAX_DUTY 255

#define SENSOR_INTERVAL_MS 2000UL
#define FAN_OFF_DELAY_MS 60000UL

// ===================== OTA CONFIG =====================
// Kitchen node: doi OTA_HOSTNAME thanh "kitchen-node"
#define OTA_HOSTNAME        "bedroom-node"
#define OTA_PASSWORD        "Bedroom@k18"
#define OTA_WIFI_SSID       "hvt"
#define OTA_WIFI_PASS       "10012511"
#define OTA_WINDOW_MS       300000UL   // 5 phut cho phep upload
#define OTA_WIFI_TIMEOUT_MS 15000UL    // timeout ket noi WiFi

DHT dht;
BH1750 lightMeter;

uint8_t gatewayMAC[] = {0x1C, 0xDB, 0xD4, 0xCB, 0xCE, 0xE8};

typedef struct __attribute__((packed)) {
  float temp;
  float humid;
  float light;
  bool motion;
  bool fan;
  bool led;
  uint8_t alarm_level;
} SensorData;

typedef struct __attribute__((packed)) {
  char command[20];
} ControlData;

SensorData sensorData;

const float LUX_THRESHOLD = 50.0;

bool manualMode = false;
bool fanState = false;
bool ledState = false;
uint8_t alarmLevel = 0;
unsigned long lastMotionMs = 0;
unsigned long lastSensorMs = 0;

// OTA state
volatile bool otaRequested = false;  // dat trong callback ESP-NOW, xu ly trong loop()
bool otaMode = false;
unsigned long otaStartMs = 0;

// ===================== ACTUATORS =====================
void fanOn()
{
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, FAN_SPEED);
  fanState = true;
}

void fanOff()
{
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
  fanState = false;
}

void ledOn()
{
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
}

void ledOff()
{
  digitalWrite(LED_PIN, LOW);
  ledState = false;
}

void setAlarm(uint8_t level)
{
  alarmLevel = level;
  if (level == 0) {
    ledcWrite(PWM_BUZZER, 0);
    digitalWrite(LED_PIN, LOW);
    fanOff();
  } else if (level == 1) {
    ledcWrite(PWM_BUZZER, BUZZER_MED_DUTY);
    digitalWrite(LED_PIN, HIGH);
    fanOn();
  } else {
    ledcWrite(PWM_BUZZER, BUZZER_MAX_DUTY);
    digitalWrite(LED_PIN, HIGH);
    fanOn();
  }
}

// ===================== COMMAND HANDLING =====================
void applyCommand(const char *command)
{
  if (strcmp(command, "FAN_ON") == 0 || strcmp(command, "ON") == 0) {
    manualMode = true;
    fanOn();
  } else if (strcmp(command, "FAN_OFF") == 0 || strcmp(command, "OFF") == 0) {
    manualMode = true;
    fanOff();
  } else if (strcmp(command, "LED_ON") == 0) {
    manualMode = true;
    ledOn();
  } else if (strcmp(command, "LED_OFF") == 0) {
    manualMode = true;
    ledOff();
  } else if (strcmp(command, "ALARM_MED") == 0) {
    manualMode = true;
    setAlarm(1);
  } else if (strcmp(command, "ALARM_MAX") == 0 || strcmp(command, "ALERT_ON") == 0) {
    manualMode = true;
    setAlarm(2);
  } else if (strcmp(command, "ALARM_OFF") == 0 || strcmp(command, "ALERT_OFF") == 0) {
    manualMode = false;
    setAlarm(0);
  } else if (strcmp(command, "AUTO") == 0) {
    manualMode = false;
    setAlarm(0);
  } else if (strcmp(command, "OTA") == 0) {
    otaRequested = true;  // chi dat co, loop() se xu ly
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  if (len != sizeof(ControlData)) {
    return;
  }

  ControlData cmd;
  memcpy(&cmd, incomingData, sizeof(cmd));
  cmd.command[sizeof(cmd.command) - 1] = '\0';

  Serial.print("Command: ");
  Serial.println(cmd.command);
  applyCommand(cmd.command);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// ===================== SENSORS =====================
void readSensor(float &t, float &h, float &lux, bool &motion)
{
  t = dht.getTemperature();
  h = dht.getHumidity();
  lux = lightMeter.readLightLevel();
  motion = digitalRead(PIR_PIN) == HIGH;
}

void sendData(float t, float h, float lux, bool motion)
{
  sensorData.temp = t;
  sensorData.humid = h;
  sensorData.light = lux;
  sensorData.motion = motion;
  sensorData.fan = fanState;
  sensorData.led = ledState;
  sensorData.alarm_level = alarmLevel;

  esp_now_send(gatewayMAC, (uint8_t *)&sensorData, sizeof(sensorData));
}

// ===================== ESP-NOW =====================
void initESPNow()
{
  WiFi.mode(WIFI_STA);
  Serial.print("Node MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, gatewayMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Add Peer Failed");
    while (true);
  }

  Serial.println("ESP-NOW READY");
}

// ===================== OTA =====================
void enterOtaMode()
{
  Serial.println("Entering OTA mode...");

  // An toan: tat toan bo co cau chap hanh truoc khi nap
  fanOff();
  ledOff();
  setAlarm(0);

  // Ngat ESP-NOW de radio duoc tu do ket noi router
  esp_now_deinit();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > OTA_WIFI_TIMEOUT_MS) {
      Serial.println("OTA WiFi failed, restarting");
      ESP.restart();
    }
    delay(200);
  }

  Serial.print("OTA IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { Serial.println("OTA start"); });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA end"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA %u%%\r", progress * 100 / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error %u, restarting\n", error);
    ESP.restart();
  });
  ArduinoOTA.begin();

  otaMode = true;
  otaStartMs = millis();
  Serial.println("OTA READY (waiting for upload)");
}

// ===================== SETUP / LOOP =====================
void setup()
{
  Serial.begin(115200);

  dht.setup(DHTPIN, DHT::DHT22);
  Wire.begin(I2C_SDA, I2C_SCL);
  lightMeter.begin();

  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  ledcSetup(PWM_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_BUZZER, BUZZER_PWM_FREQ, PWM_RES);
  ledcAttachPin(INA, PWM_A);
  ledcAttachPin(INB, PWM_B);
  ledcAttachPin(BUZZER, PWM_BUZZER);

  fanOff();
  ledOff();
  setAlarm(0);

  initESPNow();
  Serial.println("BEDROOM NODE READY");
}

void loop()
{
  // --- Che do OTA: chi phuc vu upload, het han thi khoi dong lai ---
  if (otaRequested && !otaMode) {
    enterOtaMode();
  }
  if (otaMode) {
    ArduinoOTA.handle();
    if (millis() - otaStartMs > OTA_WINDOW_MS) {
      Serial.println("OTA timeout, restarting");
      ESP.restart();
    }
    return;
  }

  // --- Che do binh thuong ---
  unsigned long now = millis();
  if (now - lastSensorMs < SENSOR_INTERVAL_MS) {
    return;
  }
  lastSensorMs = now;

  float t, h, lux;
  bool motion;
  readSensor(t, h, lux, motion);

  if (motion) {
    lastMotionMs = now;
  }

  Serial.printf("T=%.1f H=%.1f LUX=%.2f PIR=%d\n", t, h, lux, motion ? 1 : 0);
  sendData(t, h, lux, motion);

  if (!manualMode) {
    bool holdFan = motion || (now - lastMotionMs < FAN_OFF_DELAY_MS);
    if (holdFan) {
      fanOn();
    } else {
      fanOff();
    }

    if (lux < LUX_THRESHOLD && motion) {
      ledOn();
    } else {
      ledOff();
    }
  }
}