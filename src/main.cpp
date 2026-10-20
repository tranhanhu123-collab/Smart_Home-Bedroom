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

// ---- Nut nhan tich hop LED ----
// SIG2: ngo ra trang thai nut (mac dinh HIGH, nhan = LOW)
// SIG1: ngo vao dieu khien LED nut (mac dinh LOW, HIGH = sang)
#define BTN_LED_SIG2     5   // Nut vang (den)
#define BTN_LED_SIG1     6
#define BTN_FAN_SIG2     7   // Nut xanh (quat)
#define BTN_FAN_SIG1     8
#define BTN_DEBOUNCE_MS  30UL
#define BTN_LONG_PRESS_MS 2000UL       // giu nut bao lau thi ve tu dong
#define OVERRIDE_RELEASE_MS 1800000UL  // 30 phut khong co chuyen dong -> tu ve auto (dat 0 de tat)

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

// ---- Nguong nhiet do cho quat tu dong ----
#define FAN_TEMP_ON_C   28.0f   // >= nhiet do nay thi cho phep bat quat
#define FAN_TEMP_OFF_C  26.0f   // <= nhiet do nay thi tat quat (tranh bat/tat lien tuc)

// ===================== OTA CONFIG =====================
#define OTA_HOSTNAME        "bedroom-node"
#define OTA_PASSWORD        "Bedroom@k18"
#define OTA_WIFI_SSID       "hvt"
#define OTA_WIFI_PASS       "10012511"
#define OTA_WINDOW_MS       300000UL
#define OTA_WIFI_TIMEOUT_MS 15000UL

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

// ---- Ghi de tu nut nhan (manual) ----
bool fanOverride = false;   // true: nut nhan dang quyet dinh quat
bool ledOverride = false;   // true: nut nhan dang quyet dinh den

// ---- Trang thai nhiet do (co tre) ----
bool tempHot = false;

// OTA state
volatile bool otaRequested = false;
bool otaMode = false;
unsigned long otaStartMs = 0;

// ===================== BUTTON =====================
enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

struct Button {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeMs;
  unsigned long pressStartMs;
  bool longFired;
};

Button btnLed = {BTN_LED_SIG2, HIGH, HIGH, 0, 0, false};
Button btnFan = {BTN_FAN_SIG2, HIGH, HIGH, 0, 0, false};

// Nhan ngan: tra ve BTN_SHORT khi tha nut.
// Nhan giu: tra ve BTN_LONG ngay khi du thoi gian (khi dang giu).
BtnEvent readButton(Button &b)
{
  bool reading = digitalRead(b.pin);
  unsigned long now = millis();

  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.lastChangeMs = now;
  }

  if ((now - b.lastChangeMs) > BTN_DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;
    if (b.stableState == LOW) {          // vua nhan xuong
      b.pressStartMs = now;
      b.longFired = false;
    } else if (!b.longFired) {           // vua tha ra, chua phai nhan giu
      return BTN_SHORT;
    }
  }

  if (b.stableState == LOW && !b.longFired &&
      (now - b.pressStartMs) >= BTN_LONG_PRESS_MS) {
    b.longFired = true;
    return BTN_LONG;
  }
  return BTN_NONE;
}

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
    ledOff();
    fanOff();
  } else if (level == 1) {
    ledcWrite(PWM_BUZZER, BUZZER_MED_DUTY);
    ledOn();
    fanOn();
  } else {
    ledcWrite(PWM_BUZZER, BUZZER_MAX_DUTY);
    ledOn();
    fanOn();
  }
}

// Den tren CA 2 NUT deu theo trang thai LED chieu sang:
// LED tat -> 2 den nut sang (SIG1 = HIGH); LED bat -> 2 den nut tat.
void updateButtonLamps()
{
  bool lampOn = !ledState;
  digitalWrite(BTN_LED_SIG1, lampOn ? HIGH : LOW);
  digitalWrite(BTN_FAN_SIG1, lampOn ? HIGH : LOW);
}

// Nhay den trong nut 3 lan de bao "da ve tu dong"
void blinkLamp(uint8_t pin)
{
  for (int i = 0; i < 3; i++) {
    digitalWrite(pin, HIGH); delay(100);
    digitalWrite(pin, LOW);  delay(100);
  }
}

void handleButtons()
{
  // ---- Nut vang: den ----
  BtnEvent e = readButton(btnLed);
  if (e == BTN_SHORT) {
    ledOverride = true;
    if (ledState) ledOff(); else ledOn();
    Serial.printf("Button LED -> %s (MANUAL)\n", ledState ? "ON" : "OFF");
  } else if (e == BTN_LONG) {
    ledOverride = false;
    manualMode = false;
    blinkLamp(BTN_LED_SIG1);
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;  // danh gia lai ngay
    Serial.println("Button LED -> AUTO");
  }

  // ---- Nut xanh: quat ----
  e = readButton(btnFan);
  if (e == BTN_SHORT) {
    fanOverride = true;
    if (fanState) fanOff(); else fanOn();
    Serial.printf("Button FAN -> %s (MANUAL)\n", fanState ? "ON" : "OFF");
  } else if (e == BTN_LONG) {
    fanOverride = false;
    manualMode = false;
    blinkLamp(BTN_FAN_SIG1);
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;
    Serial.println("Button FAN -> AUTO");
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
    fanOverride = false;
    ledOverride = false;
    setAlarm(0);
  } else if (strcmp(command, "AUTO") == 0) {
    manualMode = false;
    fanOverride = false;
    ledOverride = false;
    setAlarm(0);
  } else if (strcmp(command, "OTA") == 0) {
    otaRequested = true;
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

  setAlarm(0);
  updateButtonLamps();

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

  // Nut nhan: SIG2 la ngo ra cua module (mac dinh HIGH) -> doc vao
  //           SIG1 la ngo vao cua module (mac dinh LOW)  -> xuat ra
  pinMode(BTN_LED_SIG2, INPUT_PULLUP);
  pinMode(BTN_FAN_SIG2, INPUT_PULLUP);
  pinMode(BTN_LED_SIG1, OUTPUT);
  pinMode(BTN_FAN_SIG1, OUTPUT);

  ledcSetup(PWM_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_BUZZER, BUZZER_PWM_FREQ, PWM_RES);
  ledcAttachPin(INA, PWM_A);
  ledcAttachPin(INB, PWM_B);
  ledcAttachPin(BUZZER, PWM_BUZZER);

  fanOff();
  ledOff();
  setAlarm(0);
  updateButtonLamps();

  initESPNow();
  Serial.println("BEDROOM NODE READY");
}

void loop()
{
  // --- Che do OTA ---
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

  // --- Nut nhan: quet lien tuc, phan hoi ngay ---
  handleButtons();
  updateButtonLamps();

  // --- Che do binh thuong (chu ky 2s) ---
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

  // Cap nhat trang thai nhiet do co tre (bo qua neu DHT doc loi)
  if (!isnan(t)) {
    if (t >= FAN_TEMP_ON_C) tempHot = true;
    else if (t <= FAN_TEMP_OFF_C) tempHot = false;
  }

  // Khong co chuyen dong du lau -> tu tra ve auto (OVERRIDE_RELEASE_MS = 0: tat tinh nang nay)
  if (OVERRIDE_RELEASE_MS > 0 && !motion &&
      (now - lastMotionMs >= OVERRIDE_RELEASE_MS)) {
    if (fanOverride || ledOverride) {
      Serial.println("No presence: release button override");
    }
    fanOverride = false;
    ledOverride = false;
  }

  Serial.printf("T=%.1f H=%.1f LUX=%.2f PIR=%d HOT=%d FAN=%d LED=%d | fanMan=%d ledMan=%d gwMan=%d\n",
                t, h, lux, motion ? 1 : 0, tempHot ? 1 : 0,
                fanState ? 1 : 0, ledState ? 1 : 0,
                fanOverride ? 1 : 0, ledOverride ? 1 : 0, manualMode ? 1 : 0);

  if (!manualMode) {
    // Quat: tu dong khi nut xanh KHONG o manual
    // Bat khi (co nguoi hoac vua co nguoi < 60s) VA nhiet do cao
    if (!fanOverride) {
      bool presence = motion || (now - lastMotionMs < FAN_OFF_DELAY_MS);
      if (presence && tempHot) fanOn(); else fanOff();
    }

    // Den: tu dong khi nut vang KHONG o manual
    if (!ledOverride) {
      if (lux < LUX_THRESHOLD && motion) ledOn(); else ledOff();
    }
  }

  updateButtonLamps();
  sendData(t, h, lux, motion);
}