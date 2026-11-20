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
#define BTN_LED_SIG1        5   // Nut vang (den)
#define BTN_LED_SIG2        6
#define BTN_FAN_SIG1        7   // Nut xanh (quat)
#define BTN_FAN_SIG2        8
#define BTN_DEBOUNCE_MS     30UL
#define BTN_LONG_PRESS_MS   2000UL       // giu nut bao lau thi ve tu dong
#define OVERRIDE_RELEASE_MS 1800000UL    // 30 phut khong co hoat dong -> tu ve auto (dat 0 de tat)

#define PWM_A           0
#define PWM_B           1
#define PWM_BUZZER      2
#define PWM_FREQ        5000
#define BUZZER_PWM_FREQ 2000
#define PWM_RES         8
#define FAN_SPEED       200
#define BUZZER_MED_DUTY 128
#define BUZZER_MAX_DUTY 255

#define SENSOR_INTERVAL_MS 2000UL
#define PRESENCE_HOLD_MS   4500UL   // Giữ thêm 4,5 giây kể từ khi xác nhận xung đã xuống LOW.

// ---- Nguong nhiet do cho quat tu dong ----
#define FAN_TEMP_ON_C   28.0f   // >= nhiet do nay thi cho phep bat quat
#define FAN_TEMP_OFF_C  26.0f   // <= nhiet do nay thi tat quat (tranh bat/tat lien tuc)

// ===================== PIR FILTER =====================
#define PIR_WARMUP_MS          60000UL
#define PIR_SAMPLE_INTERVAL_MS 20UL
#define PIR_HIGH_CONFIRM_MS    600UL
#define PIR_LOW_CONFIRM_MS     400UL

// Đổi thành true rồi nạp qua USB để chỉ đọc GPIO14 và in Serial.
constexpr bool PIR_RAW_ONLY = false;

enum class PirPhase { Warmup, WaitLow, Ready };
PirPhase pirPhase = PirPhase::Warmup;
int pirRaw = -1;                       // Chưa lấy mẫu.
bool pirFilteredState = false;
bool pirHoldActive = false;
unsigned long pirStableSinceMs = 0;
unsigned long pirLastSampleMs = 0;
unsigned long bootMs = 0;

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

const float LUX_THRESHOLD = 50.0f;

bool manualMode = false;
bool fanState = false;
bool ledState = false;
uint8_t alarmLevel = 0;
unsigned long lastMotionMs = 0;
unsigned long lastSensorMs = 0;

// ---- Ghi de tu nut nhan (manual) ----
bool fanOverride = false;    // true: nut nhan dang quyet dinh quat
bool ledOverride = false;    // true: nut nhan dang quyet dinh den
unsigned long lastBtnMs = 0; // lan nhan nut gan nhat (cho bo dem 30 phut)

// ---- Snapshot trang thai truoc bao dong de khoi phuc ----
struct PreAlarmSnapshot
{
  bool valid;
  bool manualMode, fanOverride, ledOverride;
  bool fan, led;
};
PreAlarmSnapshot preAlarm = {false, false, false, false, false, false};

// ---- Trang thai nhiet do (co tre) ----
bool tempHot = false;

// OTA state
volatile bool otaRequested = false;
bool otaMode = false;
unsigned long otaStartMs = 0;

// ===================== PROTOTYPES =====================
void fanOn();
void fanOff();
void ledOn();
void ledOff();
void setAlarm(uint8_t level);
void updateButtonLamps();
void snapshotBeforeAlarm();
void restoreAfterAlarm();
void blinkLamp(uint8_t pin);
void handleButtons();
void applyCommand(const char *command);
void updatePirFilter();
void sendData(float t, float h, float lux, bool motion);
void initESPNow();
void enterOtaMode();

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
    if (b.stableState == LOW) {
      b.pressStartMs = now;
      b.longFired = false;
    } else if (!b.longFired) {
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
  if (fanState) return;
  ledcWrite(PWM_A, 0);
  for (int speed = 0; speed <= FAN_SPEED; speed += 10)
  {
    ledcWrite(PWM_B, speed);
    delay(15);
  }
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

void snapshotBeforeAlarm()
{
  if (alarmLevel != 0 || preAlarm.valid) return;
  preAlarm = {true, manualMode, fanOverride, ledOverride, fanState, ledState};
}

void restoreAfterAlarm()
{
  PreAlarmSnapshot snap = preAlarm;
  preAlarm.valid = false;

  alarmLevel = 0;
  ledcWrite(PWM_BUZZER, 0);

  manualMode  = snap.manualMode;
  fanOverride = snap.fanOverride;
  ledOverride = snap.ledOverride;

  if (manualMode || fanOverride) {
    if (snap.fan) fanOn(); else fanOff();
  }
  if (manualMode || ledOverride) {
    if (snap.led) ledOn(); else ledOff();
  }

  lastBtnMs = millis();
  lastSensorMs = millis() - SENSOR_INTERVAL_MS;
  updateButtonLamps();
  Serial.println("Alarm cleared: restored previous state");
}

void updateButtonLamps()
{
  bool lampOn = !ledState;
  digitalWrite(BTN_LED_SIG1, lampOn ? HIGH : LOW);
  digitalWrite(BTN_FAN_SIG1, lampOn ? HIGH : LOW);
}

void blinkLamp(uint8_t pin)
{
  for (int i = 0; i < 3; i++) {
    digitalWrite(pin, HIGH); delay(100);
    digitalWrite(pin, LOW);  delay(100);
  }
}

void handleButtons()
{
  BtnEvent eled = readButton(btnLed);
  BtnEvent efan = readButton(btnFan);

  if (alarmLevel > 0) {
    if (eled != BTN_NONE || efan != BTN_NONE) {
      Serial.println("Alarm active: ignore button press");
    }
    return;
  }

  if (eled == BTN_SHORT) {
    ledOverride = true;
    lastBtnMs = millis();
    if (ledState) ledOff(); else ledOn();
    Serial.printf("Button LED -> %s (MANUAL)\n", ledState ? "ON" : "OFF");
  } else if (eled == BTN_LONG) {
    ledOverride = false;
    manualMode = false;
    blinkLamp(BTN_LED_SIG1);
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;
    Serial.println("Button LED -> AUTO");
  }

  if (efan == BTN_SHORT) {
    fanOverride = true;
    lastBtnMs = millis();
    if (fanState) fanOff(); else fanOn();
    Serial.printf("Button FAN -> %s (MANUAL)\n", fanState ? "ON" : "OFF");
  } else if (efan == BTN_LONG) {
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
  if (alarmLevel > 0 &&
      (strcmp(command, "FAN_ON") == 0  || strcmp(command, "ON") == 0  ||
       strcmp(command, "FAN_OFF") == 0 || strcmp(command, "OFF") == 0 ||
       strcmp(command, "LED_ON") == 0  || strcmp(command, "LED_OFF") == 0)) {
    Serial.println("Alarm active: ignore device command");
    return;
  }

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
    snapshotBeforeAlarm();
    manualMode = true;
    setAlarm(1);
  } else if (strcmp(command, "ALARM_MAX") == 0 || strcmp(command, "ALERT_ON") == 0) {
    snapshotBeforeAlarm();
    manualMode = true;
    setAlarm(2);
  } else if (strcmp(command, "ALARM_OFF") == 0 || strcmp(command, "ALERT_OFF") == 0) {
    if (preAlarm.valid) {
      restoreAfterAlarm();
    } else if (alarmLevel > 0) {
      manualMode = false;
      fanOverride = false;
      ledOverride = false;
      setAlarm(0);
    } else {
      Serial.println("ALARM_OFF lap lai: bo qua (da het bao dong)");
    }
  } else if (strcmp(command, "AUTO") == 0) {
    preAlarm.valid = false;
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

// ===================== PIR FILTER =====================
const char *pirPhaseName()
{
  switch (pirPhase) {
    case PirPhase::Warmup:  return "WARMUP";
    case PirPhase::WaitLow: return "WAIT_LOW";
    case PirPhase::Ready:   return "READY";
  }
  return "?";
}

void updatePirFilter()
{
  const unsigned long now = millis();
  const unsigned long sampleGap = now - pirLastSampleMs;
  if (sampleGap < PIR_SAMPLE_INTERVAL_MS) return;
  pirLastSampleMs = now;

  const int raw = digitalRead(PIR_PIN);
  const bool changed = (raw != pirRaw);
  pirRaw = raw;
  static unsigned long lastRawLogMs = 0;
  if (changed || (PIR_RAW_ONLY && now - lastRawLogMs >= 1000UL)) {
    Serial.printf("[PIR] t=%lu ms GPIO%d RAW=%d\n", now, PIR_PIN, raw);
    lastRawLogMs = now;
  }
  if (PIR_RAW_ONLY) return;

  // Nếu loop bận (chớp đèn nút, khởi động quạt), xác nhận lại từ mẫu mới.
  if (changed || sampleGap > 3 * PIR_SAMPLE_INTERVAL_MS) pirStableSinceMs = now;

  if (pirPhase == PirPhase::Warmup) {
    if (now - bootMs < PIR_WARMUP_MS) return;
    pirPhase = PirPhase::WaitLow;
    pirStableSinceMs = now;
    Serial.println("[PIR] WAIT_LOW");
  }

  const unsigned long stableMs = now - pirStableSinceMs;
  if (pirPhase == PirPhase::WaitLow) {
    if (raw == LOW && stableMs >= PIR_LOW_CONFIRM_MS) {
      pirPhase = PirPhase::Ready;
      Serial.println("[PIR] READY");
    }
    return;
  }

  const bool high = (raw == HIGH);
  const unsigned long confirmMs = high ? PIR_HIGH_CONFIRM_MS : PIR_LOW_CONFIRM_MS;
  if (high == pirFilteredState || stableMs < confirmMs) return;

  pirFilteredState = high;
  lastMotionMs = now;
  pirHoldActive = !high;  // Bắt đầu giữ khi xác nhận LOW, hủy giữ khi có xung HIGH mới.
  Serial.printf("[PIR] t=%lu ms FLT=%d\n", now, high ? 1 : 0);
}

// ===================== ESP-NOW =====================
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

void initESPNow()
{
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);  // Giam cong suat phat de chong nhieu RF sang PIR
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
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Tang lai cong suat toi da khi ket noi Wi-Fi OTA
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

  pinMode(PIR_PIN, INPUT_PULLDOWN);
  bootMs = millis();
  pirLastSampleMs = bootMs;
  if (PIR_RAW_ONLY) {
    Serial.println("[PIR] RAW_ONLY");
    return;
  }
  Serial.printf("[PIR] WARMUP %lu s\n", PIR_WARMUP_MS / 1000UL);

  dht.setup(DHTPIN, DHT::DHT22);
  Wire.begin(I2C_SDA, I2C_SCL);
  lightMeter.begin();

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

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

  delay(200);
  neopixelWrite(48, 0, 0, 0);

  Serial.println("BEDROOM NODE READY");
}

void loop()
{
  if (PIR_RAW_ONLY) {
    updatePirFilter();
    delay(1);
    return;
  }

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

  // --- Nut nhan & loc PIR: quet lien tuc ---
  handleButtons();
  updateButtonLamps();
  updatePirFilter();

  unsigned long now = millis();
  static float lastT = NAN, lastH = NAN, lastLux = 0.0f;
  static bool lastPresence = false;

  // 1. Trang thai co nguoi cap nhat ngay khi PIR doi trang thai
  const bool motion = pirFilteredState;
  if (pirHoldActive && now - lastMotionMs >= PRESENCE_HOLD_MS) pirHoldActive = false;
  const bool presence = motion || pirHoldActive;
  bool presenceChanged = (presence != lastPresence);
  lastPresence = presence;

  bool sensorDue = (now - lastSensorMs >= SENSOR_INTERVAL_MS);
  if (!sensorDue && !presenceChanged) {
    return;
  }

  // 2. Chi doc DHT22 theo chu ky 2s
  if (sensorDue) {
    lastSensorMs = now;
    lastT = dht.getTemperature();
    lastH = dht.getHumidity();
    lastLux = lightMeter.readLightLevel();

    if (!isnan(lastT)) {
      if (lastT >= FAN_TEMP_ON_C) tempHot = true;
      else if (lastT <= FAN_TEMP_OFF_C) tempHot = false;
    }

    unsigned long lastActivity = (lastMotionMs > lastBtnMs) ? lastMotionMs : lastBtnMs;
    if (OVERRIDE_RELEASE_MS > 0 && !presence &&
        (now - lastActivity >= OVERRIDE_RELEASE_MS)) {
      if (fanOverride || ledOverride) {
        Serial.println("No presence: release button override");
      }
      fanOverride = false;
      ledOverride = false;
    }
  } else if (presence) {
    lastLux = lightMeter.readLightLevel();
  }

  // 3. Dieu khien thiet bi
  if (!manualMode) {
    if (!fanOverride) {
      if (presence && tempHot) fanOn();
      else fanOff();
    }
    if (!ledOverride) {
      if (!presence) {
        ledOff();
      } else if (!ledState && lastLux >= 0.0f && lastLux < LUX_THRESHOLD) {
        ledOn();
      }
    }
  }

  // 4. Gửi theo chu kỳ hoặc ngay khi PRES đổi, kể cả lúc DHT22 chưa có số đo.
  Serial.printf("T=%.1f H=%.1f LUX=%.2f PIR=%s RAW=%d FLT=%d PRES=%d HOT=%d FAN=%d LED=%d | fanMan=%d ledMan=%d gwMan=%d\n",
                lastT, lastH, lastLux, pirPhaseName(),
                pirRaw, motion ? 1 : 0, presence ? 1 : 0,
                tempHot ? 1 : 0, fanState ? 1 : 0, ledState ? 1 : 0,
                fanOverride ? 1 : 0, ledOverride ? 1 : 0, manualMode ? 1 : 0);

  neopixelWrite(48, 0, 0, 0);
  updateButtonLamps();
  sendData(lastT, lastH, lastLux, presence);
}
