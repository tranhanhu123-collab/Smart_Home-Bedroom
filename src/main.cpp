#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <ArduinoOTA.h>
#include <BH1750.h>
#include "DHT.h"
#include <cmath>

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
constexpr uint8_t FAN_HIGH = 3;
constexpr uint8_t FAN_PWM[] = {0, 100, 150, 200};  // Tắt, nhẹ, trung bình, mạnh (PWM 8 bit).
constexpr unsigned long FAN_TAP_GAP_MS = 500UL;
constexpr unsigned long FAN_RAMP_MS = 300UL;
constexpr unsigned long FAN_START_HOLD_MS = 120UL;
#define BUZZER_MED_DUTY 128
#define BUZZER_MAX_DUTY 255

#define SENSOR_INTERVAL_MS 2000UL
#define PRESENCE_HOLD_MS   5000UL  // Sau khi xác nhận có người, cho phép khoảng nghỉ giữa hai xung.
//  Xác nhận có người ở cạnh lên thứ hai nếu hai cạnh đã lọc cách nhau 2 - 5 giây.
constexpr unsigned long ENTRY_MIN_INTERVAL_MS = 2000UL;
constexpr unsigned long ENTRY_WINDOW_MS = 5000UL;

// ---- Nguong nhiet do cho quat tu dong ----
#define FAN_TEMP_ON_C   28.0f   // >= nhiet do nay thi cho phep bat quat
#define FAN_TEMP_OFF_C  26.0f   // <= nhiet do nay thi tat quat (tranh bat/tat lien tuc)

// ===================== PIR FILTER =====================
#define PIR_WARMUP_MS          30000UL
#define PIR_STARTUP_LOW_MS      3000UL  // Thời gian LOW liên tục sau khi hết thời gian khởi động.
#define PIR_SAMPLE_INTERVAL_MS 20UL

// Đổi thành true rồi nạp qua USB để chỉ đọc GPIO14 và in Serial.
constexpr bool PIR_RAW_ONLY = false;

enum class PirPhase { Warmup, WaitLow, Ready };
PirPhase pirPhase = PirPhase::Warmup;
int pirRaw = -1;                       // Chưa lấy mẫu.
bool pirFilteredState = false;
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

// Giữ nguyên SensorData đến khi bridge gửi FAN_GET để nhận cả mức và chế độ quạt.
struct __attribute__((packed)) FanTelemetry {
  SensorData sensors;
  uint8_t fan_level;
  uint8_t fan_mode;  // 0 AUTO, 1 nút nhấn, 2 gateway, 3 báo động.
};
static_assert(sizeof(SensorData) == 16 && sizeof(ControlData) == 20 && sizeof(FanTelemetry) == 18,
              "Unexpected ESP-NOW packet layout");
SensorData sensorData;
constexpr unsigned int COMMAND_QUEUE_DEPTH = 12;
QueueHandle_t commandQueue = nullptr;

struct __attribute__((packed)) FanButtonTelemetry {
  FanTelemetry fan;
  uint8_t fan_button;
};
static_assert(sizeof(FanButtonTelemetry) == 19, "Unexpected button packet size");

size_t fanReportBytes = sizeof(SensorData);
uint8_t pendingFanButton = 0;
bool fanReplyDue = false;

constexpr unsigned long LUX_SAMPLE_MS = 300UL;
constexpr float LUX_ON_THRESHOLD = 50.0f;
constexpr float LUX_OFF_THRESHOLD = 100.0f;
constexpr uint8_t BRIGHT_SAMPLES_TO_TURN_OFF = 2;
uint8_t brightSamples = 0;

// Lệnh cho quạt và đèn chỉ đổi chế độ của thiết bị tương ứng.
enum class DeviceMode : uint8_t { Auto = 0, Button = 1, Gateway = 2 };
DeviceMode fanMode = DeviceMode::Auto;
DeviceMode ledMode = DeviceMode::Auto;
uint8_t fanLevel = 0;
uint8_t fanDuty = 0;
bool fanStarting = false;
unsigned long fanStartMs = 0;
bool ledState = false;
uint8_t alarmLevel = 0;
unsigned long lastMotionMs = 0;
unsigned long lastSensorMs = 0;
unsigned long lastBtnMs = 0; // lan nhan nut gan nhat (cho bo dem 30 phut)

// ---- Snapshot trang thai truoc bao dong de khoi phuc ----
struct PreAlarmSnapshot
{
  bool valid;
  DeviceMode fanMode, ledMode;
  uint8_t fanLevel;
  bool led;
};
PreAlarmSnapshot preAlarm{};

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
uint8_t fanTapCount = 0;
unsigned long lastFanReleaseMs = 0;
bool ignoreFanUntilRelease = false;

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
void updateFanPwm()
{
  uint8_t duty = FAN_PWM[fanLevel];
  if (fanStarting) {
    const unsigned long elapsed = millis() - fanStartMs;
    if (elapsed < FAN_RAMP_MS) {
      duty = static_cast<uint8_t>(FAN_PWM[FAN_HIGH] * elapsed / FAN_RAMP_MS);
    } else if (elapsed < FAN_RAMP_MS + FAN_START_HOLD_MS) {
      duty = FAN_PWM[FAN_HIGH];
    } else {
      fanStarting = false;
    }
  }
  if (duty != fanDuty) {
    ledcWrite(PWM_B, duty);
    fanDuty = duty;
  }
}

void setFanLevel(uint8_t level)
{
  if (level > FAN_HIGH || level == fanLevel) return;
  if (level == 0) {
    fanStarting = false;
  } else if (fanLevel == 0) {
    // Khởi động êm đến mức quạt đã chạy ổn, rồi hạ xuống mức đã chọn.
    fanStartMs = millis();
    fanStarting = true;
  }
  fanLevel = level;
  updateFanPwm();
}

void fanOn() { setFanLevel(FAN_HIGH); }
void fanOff() { setFanLevel(0); }

void cancelFanGesture()
{
  fanTapCount = 0;
  if (btnFan.stableState == LOW || digitalRead(btnFan.pin) == LOW) ignoreFanUntilRelease = true;
}

uint8_t fanModeForReport()
{
  return alarmLevel > 0 ? 3 : static_cast<uint8_t>(fanMode);
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
  cancelFanGesture();
  if (level != alarmLevel) fanReplyDue = true;
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
  preAlarm = {true, fanMode, ledMode, fanLevel, ledState};
}

void restoreAfterAlarm()
{
  PreAlarmSnapshot snap = preAlarm;
  preAlarm.valid = false;

  alarmLevel = 0;
  fanReplyDue = true;
  ledcWrite(PWM_BUZZER, 0);

  fanMode = snap.fanMode;
  ledMode = snap.ledMode;
  cancelFanGesture();
  if (fanMode != DeviceMode::Auto) {
    setFanLevel(snap.fanLevel);
  }
  // Trong dải 50–100 lux, AUTO cần trạng thái đèn trước khi báo động.
  if (snap.led) ledOn(); else ledOff();

  lastBtnMs = millis();
  lastSensorMs = millis() - SENSOR_INTERVAL_MS;
  updateButtonLamps();
  Serial.println("Alarm cleared: restored previous state");
}

void updateButtonLamps()
{
  // Sau ba nhịp xác nhận AUTO, giữ tiếp nút vàng để xem BH1750 đã đủ sáng chưa.
  const bool showBrightness = alarmLevel == 0 && btnLed.longFired && btnLed.stableState == LOW;
  const bool locatorOn = !ledState;
  const bool yellowOn = showBrightness ? brightSamples >= BRIGHT_SAMPLES_TO_TURN_OFF : locatorOn;
  digitalWrite(BTN_LED_SIG1, yellowOn ? HIGH : LOW);
  digitalWrite(BTN_FAN_SIG1, locatorOn ? HIGH : LOW);
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
    ledMode = DeviceMode::Button;
    lastBtnMs = millis();
    if (ledState) ledOff(); else ledOn();
    Serial.printf("Button LED -> %s (MANUAL)\n", ledState ? "ON" : "OFF");
  } else if (eled == BTN_LONG) {
    ledMode = DeviceMode::Auto;
    blinkLamp(BTN_LED_SIG1);
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;
    Serial.println("Button LED -> AUTO");
  }

  if (ignoreFanUntilRelease) {
    if (btnFan.stableState == HIGH && btnFan.lastReading == HIGH) ignoreFanUntilRelease = false;
    return;
  }

  if (efan == BTN_LONG) {
    fanTapCount = 0;
    fanMode = DeviceMode::Auto;
    blinkLamp(BTN_FAN_SIG1);
    fanReplyDue = true;
    Serial.println("Button FAN -> AUTO");
    pendingFanButton = 255;
  } else if (efan == BTN_SHORT) {
    if (fanTapCount < 5) ++fanTapCount;
    lastFanReleaseMs = millis();
  }
  // Chỉ áp dụng một lần sau nhịp cuối. Giữ nút tiếp theo sẽ hủy chuỗi và về AUTO.
  if (fanTapCount && btnFan.stableState == HIGH && btnFan.lastReading == HIGH &&
      millis() - lastFanReleaseMs >= FAN_TAP_GAP_MS) {
    const uint8_t taps = fanTapCount;
    fanTapCount = 0;
    if (taps <= 3) {
      const bool turnOff =
          taps == 1 && fanMode != DeviceMode::Auto && fanLevel > 0;

      fanMode = DeviceMode::Button;
      lastBtnMs = millis();
      setFanLevel(turnOff ? 0 : taps);
      pendingFanButton = taps;
      fanReplyDue = true;
      Serial.printf("Button FAN -> level %u\n", static_cast<unsigned>(fanLevel));
    }
  }
}

// ===================== COMMAND HANDLING =====================
void applyCommand(const char *command)
{
  const bool levelCommand = strncmp(command, "FAN_", 4) == 0 &&
      command[4] >= '0' && command[4] <= '3' && command[5] == '\0';
  const bool fanAuto = strcmp(command, "FAN_AUTO") == 0;
  const bool fanGet = strcmp(command, "FAN_GET") == 0;
  const bool buttonGet = strcmp(command, "FAN_BTN_GET") == 0;
  if (fanGet || buttonGet) {
    fanReportBytes = buttonGet ? sizeof(FanButtonTelemetry) : sizeof(FanTelemetry);
    fanReplyDue = true;
    return;
  }
  if (levelCommand || fanAuto) fanReplyDue = true;
  const bool on = strcmp(command, "FAN_ON") == 0 || strcmp(command, "ON") == 0;
  const bool off = strcmp(command, "FAN_OFF") == 0 || strcmp(command, "OFF") == 0;
  if (alarmLevel > 0 && (on || off || levelCommand || fanAuto ||
      strcmp(command, "LED_ON") == 0 || strcmp(command, "LED_OFF") == 0)) {
    Serial.println("Alarm active: ignore device command");
    return;
  }

  if (levelCommand || on || off) {
    cancelFanGesture();
    fanMode = DeviceMode::Gateway;
    uint8_t level = on ? FAN_HIGH : 0;
    if (levelCommand) level = static_cast<uint8_t>(command[4] - '0');
    setFanLevel(level);
    fanReplyDue = true;
  } else if (fanAuto) {
    cancelFanGesture();
    fanMode = DeviceMode::Auto;
  } else if (strcmp(command, "LED_ON") == 0) {
    ledMode = DeviceMode::Gateway;
    ledOn();
  } else if (strcmp(command, "LED_OFF") == 0) {
    ledMode = DeviceMode::Gateway;
    ledOff();
  } else if (strcmp(command, "ALARM_MED") == 0) {
    snapshotBeforeAlarm();
    setAlarm(1);
  } else if (strcmp(command, "ALARM_MAX") == 0 || strcmp(command, "ALERT_ON") == 0) {
    snapshotBeforeAlarm();
    setAlarm(2);
  } else if (strcmp(command, "ALARM_OFF") == 0 || strcmp(command, "ALERT_OFF") == 0) {
    if (preAlarm.valid) {
      restoreAfterAlarm();
    } else if (alarmLevel > 0) {
      fanMode = DeviceMode::Auto;
      ledMode = DeviceMode::Auto;
      setAlarm(0);
    } else {
      Serial.println("ALARM_OFF lap lai: bo qua (da het bao dong)");
    }
  } else if (strcmp(command, "AUTO") == 0) {
    preAlarm.valid = false;
    fanMode = DeviceMode::Auto;
    fanReplyDue = true;
    ledMode = DeviceMode::Auto;
    setAlarm(0);
  } else if (strcmp(command, "OTA") == 0) {
    otaRequested = true;
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  if (len != sizeof(ControlData) || commandQueue == nullptr) {
    return;
  }

  ControlData cmd;
  memcpy(&cmd, incomingData, sizeof(cmd));
  cmd.command[sizeof(cmd.command) - 1] = '\0';

  // Callback Wi-Fi chỉ xếp lệnh. Vòng loop xử lý PWM, nút và báo động theo thứ tự.
  if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
    Serial.println("ESP-NOW command queue full");
  }
}

void handleCommands()
{
  if (commandQueue == nullptr) return;
  ControlData cmd;
  for (unsigned int i = 0; i < COMMAND_QUEUE_DEPTH; ++i) {
    if (xQueueReceive(commandQueue, &cmd, 0) != pdTRUE) break;
    Serial.print("Command: ");
    Serial.println(cmd.command);
    applyCommand(cmd.command);
    if (otaRequested) break;
  }
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
    Serial.printf("[PIR] WAIT_LOW (%lu ms)\n", PIR_STARTUP_LOW_MS);
  }

  const unsigned long stableMs = now - pirStableSinceMs;
  if (pirPhase == PirPhase::WaitLow) {
    if (raw == LOW && stableMs >= PIR_STARTUP_LOW_MS) {
      pirPhase = PirPhase::Ready;
      Serial.println("[PIR] READY");
    }
    return;
  }

  const bool high = (raw == HIGH);
  if (high == pirFilteredState) return;

  pirFilteredState = high;
  Serial.printf("[PIR] t=%lu ms FLT=%d\n", now, high ? 1 : 0);
}

bool updateRoomPresence(bool motion, unsigned long now)
{
  enum class RoomState { Empty, WaitingForSecondPulse, Occupied };
  static RoomState state = RoomState::Empty;
  static unsigned long firstPulseMs = 0;
  static bool previousMotion = false;
  const unsigned long sinceLastEdgeMs = now - lastMotionMs;

  // Xét hết hạn trước khi xử lý cạnh mới, kể cả khi vòng lặp vừa bị trễ.
  if (state == RoomState::WaitingForSecondPulse && now - firstPulseMs > ENTRY_WINDOW_MS) {
    state = RoomState::Empty;
    Serial.println("[ROOM] EMPTY (entry window expired)");
  } else if (state == RoomState::Occupied && !previousMotion &&
             sinceLastEdgeMs >= PRESENCE_HOLD_MS) {
    state = RoomState::Empty;
    Serial.printf("[ROOM] EMPTY (FLT=0 for %lu ms)\n", sinceLastEdgeMs);
  }

  if (motion != previousMotion) {
    previousMotion = motion;
    lastMotionMs = now;
    if (motion && state == RoomState::Empty) {
      firstPulseMs = now;
      state = RoomState::WaitingForSecondPulse;
      Serial.println("[ROOM] WAIT_SECOND");
    } else if (motion && state == RoomState::WaitingForSecondPulse &&
               now - firstPulseMs >= ENTRY_MIN_INTERVAL_MS) {
      state = RoomState::Occupied;
      Serial.println("[ROOM] OCCUPIED");
    } else if (motion && state == RoomState::Occupied) {
      Serial.printf("[ROOM] LOW_GAP_MS=%lu\n", sinceLastEdgeMs);
    }
  }
  return state == RoomState::Occupied;
}

// ===================== ESP-NOW =====================
void sendData(float t, float h, float lux, bool motion)
{
  sensorData.temp = t;
  sensorData.humid = h;
  sensorData.light = lux;
  sensorData.motion = motion;
  sensorData.fan = fanLevel != 0;
  sensorData.led = ledState;
  sensorData.alarm_level = alarmLevel;

  const FanButtonTelemetry packet{
    {sensorData, fanLevel, fanModeForReport()}, pendingFanButton
  };

  if (esp_now_send(gatewayMAC, reinterpret_cast<const uint8_t *>(&packet),
                  fanReportBytes) == ESP_OK) {
    pendingFanButton = 0;
  } else {
    esp_now_send(gatewayMAC, reinterpret_cast<const uint8_t *>(&sensorData), sizeof(sensorData));
  }
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

  commandQueue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(ControlData));
  if (commandQueue != nullptr) esp_now_register_recv_cb(OnDataRecv);
  else Serial.println("ESP-NOW command queue unavailable");
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

  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
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
  if (!otaMode) handleCommands();
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
  updateFanPwm();
  updateButtonLamps();
  updatePirFilter();

  unsigned long now = millis();
  static float lastT = NAN, lastH = NAN, lastLux = NAN;
  static unsigned long lastLuxMs = 0;
  static bool lastPresence = false;

  // 1. Xác nhận có người trước khi điều khiển tự động hoặc báo lên gateway.
  const bool motion = pirFilteredState;
  const bool presence = updateRoomPresence(motion, now);
  bool presenceChanged = (presence != lastPresence);
  lastPresence = presence;

  const bool sensorDue = (now - lastSensorMs >= SENSOR_INTERVAL_MS);
  const bool luxDue = (now - lastLuxMs >= LUX_SAMPLE_MS);
  if (!sensorDue && !presenceChanged && !luxDue && !fanReplyDue) {
    return;
  }

  // 2. DHT22 và BH1750 có chu kỳ đọc riêng.
  if (sensorDue) {
    lastSensorMs = now;
    lastT = dht.getTemperature();
    lastH = dht.getHumidity();

    if (!isnan(lastT)) {
      if (lastT >= FAN_TEMP_ON_C) tempHot = true;
      else if (lastT <= FAN_TEMP_OFF_C) tempHot = false;
    }

    unsigned long lastActivity = (lastMotionMs > lastBtnMs) ? lastMotionMs : lastBtnMs;
    if (OVERRIDE_RELEASE_MS > 0 && !presence &&
        now - lastActivity >= OVERRIDE_RELEASE_MS &&
        (fanMode == DeviceMode::Button || ledMode == DeviceMode::Button)) {
      Serial.println("No presence: release button override");
      if (fanMode == DeviceMode::Button) fanMode = DeviceMode::Auto;
      if (ledMode == DeviceMode::Button) ledMode = DeviceMode::Auto;
    }
  }
  if (luxDue) {
    lastLuxMs = now;
    lastLux = lightMeter.readLightLevel();
    if (std::isfinite(lastLux) && lastLux >= LUX_OFF_THRESHOLD) {
      if (brightSamples < BRIGHT_SAMPLES_TO_TURN_OFF) ++brightSamples;
    } else {
      brightSamples = 0;
    }
  }

  // 3. Dieu khien thiet bi
  const bool ledBeforeAuto = ledState;
  if (alarmLevel == 0) {
    if (fanMode == DeviceMode::Auto && (sensorDue || presenceChanged || fanReplyDue)) {
      if (presence && tempHot) fanOn();
      else fanOff();
    }
    if (ledMode == DeviceMode::Auto) {
      if (ledState && (!presence || brightSamples >= BRIGHT_SAMPLES_TO_TURN_OFF)) {
        ledOff();
      } else if (presence && !ledState && lastLux >= 0.0f && lastLux < LUX_ON_THRESHOLD) {
        ledOn();
      }
    }
  }

  // Phản hồi cả khi chuyển giữa hai mức đang quay hoặc được hỏi lại cùng mức.
  if (!sensorDue && !presenceChanged && ledBeforeAuto == ledState && !fanReplyDue) return;
  fanReplyDue = false;
  const bool gwFan = fanMode == DeviceMode::Gateway;
  const bool gwLed = ledMode == DeviceMode::Gateway;
  Serial.printf("T=%.1f H=%.1f LUX=%.2f PIR=%s RAW=%d FLT=%d PRES=%d HOT=%d FAN=%d LED=%d | fanMan=%d ledMan=%d gwMan=%d gwLed=%d alarm=%d bright=%d/%d\n",
                lastT, lastH, lastLux, pirPhaseName(),
                pirRaw, motion ? 1 : 0, presence ? 1 : 0,
                tempHot ? 1 : 0, fanLevel != 0, ledState ? 1 : 0,
                fanMode == DeviceMode::Button, ledMode == DeviceMode::Button, gwFan || gwLed || alarmLevel > 0,
                gwLed, alarmLevel, brightSamples, BRIGHT_SAMPLES_TO_TURN_OFF);

  neopixelWrite(48, 0, 0, 0);
  updateButtonLamps();
  sendData(lastT, lastH, lastLux, presence);
}
