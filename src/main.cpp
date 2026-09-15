#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <BH1750.h>
#include "DHT.h"

// ==========================================
// 1. KHAI BÁO CHÂN (PINS DEFINITION)
// ==========================================
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
#define PWM_FREQ 5000
#define PWM_RES 8
#define FAN_SPEED 200

DHT dht;
BH1750 lightMeter;


uint8_t gatewayMAC[] ={ 0x1C,0xDB,0xD4,0xCB,0xCE,0xE8};

typedef struct {
  float temp;
  float humid;
  float lux;
} SensorData;

typedef struct {
  char command[10];
} ControlData;

SensorData sensorData;

// ==========================================
// 2. CÁC BIẾN QUẢN LÝ TRẠNG THÁI & THỜI GIAN
// ==========================================
unsigned long previousMillis = 0;
const long interval = 2000; // Đọc cảm biến mỗi 2 giây (2000ms)

bool isAutoMode = true;     // Mặc định khởi động là chế độ Tự động
int manualFanSpeed = 0;     // Tốc độ quạt thủ công
bool manualLedState = false; // Trạng thái đèn thủ công

// Ngưỡng cấu hình tự động
const float TEMP_THRESHOLD = 28.0; // Ngưỡng nhiệt độ bật quạt (°C)
const float LUX_THRESHOLD = 50.0;   // Ngưỡng ánh sáng để xác định trời tối (lux)
bool manualMode = false;
bool fanState = false;
bool ledState = false;


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

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  ControlData cmd;
  memcpy(&cmd,
         incomingData,
         sizeof(cmd));

  Serial.print("Command: ");
  Serial.println(cmd.command);

  if(strcmp(cmd.command,"ON")==0)
  {
    manualMode = true;

    fanOn();

    digitalWrite(LED_PIN,HIGH);

    ledState = true;
  }

  else if(strcmp(cmd.command,"OFF")==0)
  {
    manualMode = true;

    fanOff();

    digitalWrite(LED_PIN,LOW);

    ledState = false;
  }

  else if(strcmp(cmd.command,"AUTO")==0)
  {
    manualMode = false;
  }
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send Status: ");
  if(status == ESP_NOW_SEND_SUCCESS)
    Serial.println("SUCCESS");
  else
    Serial.println("FAIL");
}

void readSensor(float &t, float &h, float &lux)
{
  t = dht.getTemperature();
  h = dht.getHumidity();
  lux = lightMeter.readLightLevel();
}


void sendData(float t, float h, float lux)
{
  sensorData.temp = t;
  sensorData.humid = h;
  sensorData.lux = lux;
  esp_now_send(
      gatewayMAC,
      (uint8_t *)&sensorData,
      sizeof(sensorData));
}

// Khởi tạo ESP-NOW và thiết lập chế độ WiFi cho ESP32.
void initESPNow()
{
  WiFi.mode(WIFI_STA);
  Serial.print("Node MAC: ");
  Serial.println(WiFi.macAddress());
  if(esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Init Failed");
    while(true);
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, gatewayMAC, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Add Peer Failed");
    while(true);
  }

  Serial.println("ESP-NOW READY");
}


void setup() {
  Serial.begin(115200);
  dht.setup(DHTPIN, DHT::DHT22);
  Wire.begin(I2C_SDA, I2C_SCL);
  lightMeter.begin();
  
  pinMode(PIR_PIN, INPUT);
  
  // Cấu hình chân Quạt
  ledcSetup(PWM_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(INA, PWM_A);
  ledcAttachPin(INB, PWM_B);
  
  // Cấu hình chân Đèn LED, Buzzer
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER,OUTPUT);
  digitalWrite(BUZZER,LOW); // Tắt buzzer lúc mới khởi động
  digitalWrite(LED_PIN, LOW); // Tắt đèn lúc mới khởi động
  
    initESPNow();
  Serial.println("NODE READY");

}

void loop() {
  float t,h,lux;
  readSensor(t, h, lux);
  Serial.printf("T=%.1f H=%.1f LUX=%.2f\n",t,h,lux);
  sendData(t,h,lux);

  if(!manualMode)
  {
    // Chế độ tự động
    if(t > TEMP_THRESHOLD)
    {
      fanOn();
      digitalWrite(BUZZER, HIGH);
    }
    else
    {
      fanOff();
      digitalWrite(BUZZER, LOW);
    }

    if(lux < LUX_THRESHOLD)
    {
      digitalWrite(LED_PIN, HIGH);
      ledState = true;
    }
    else
    {
      digitalWrite(LED_PIN, LOW);
      ledState = false;
    }
  }

  delay(3000);
}