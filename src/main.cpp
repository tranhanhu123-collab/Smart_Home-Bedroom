#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>

// ==========================================
// 1. KHAI BÁO CHÂN (PINS DEFINITION)
// ==========================================
#define DHTPIN 4          
#define DHTTYPE DHT22     

#define PIR_PIN 14        
#define FAN_PIN 18        
#define LED_PIN 15      

#define I2C_SDA 16        
#define I2C_SCL 17        

DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;

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

void setup() {
  Serial.begin(115200);
  dht.begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  lightMeter.begin();
  
  pinMode(PIR_PIN, INPUT);
  
  // Cấu hình chân Quạt
  pinMode(FAN_PIN, OUTPUT);
  analogWrite(FAN_PIN, 0); 
  
  // Cấu hình chân Đèn LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Tắt đèn lúc mới khởi động
  
  Serial.println(F("\n=============================================="));
  Serial.println(F("           HỆ THỐNG AIoT SMART HOME           "));
  Serial.println(F(" CÁC LỆNH TERMINAL ĐIỀU KHIỂN HỖ TRỢ:"));
  Serial.println(F(" - Gõ 'AUTO'      : Bật chế độ Tự động theo cảm biến"));
  Serial.println(F(" - Gõ 'FAN ON'    : Bật quạt thủ công (mức 220)"));
  Serial.println(F(" - Gõ 'FAN OFF'   : Tắt quạt thủ công"));
  Serial.println(F(" - Gõ 'FAN SPEED X': Chỉnh tốc độ quạt (0-255). VD: FAN SPEED 150"));
  Serial.println(F(" - Gõ 'LED ON'    : Bật đèn LED thủ công"));
  Serial.println(F(" - Gõ 'LED OFF'   : Tắt đèn LED thủ công"));
  Serial.println(F("==============================================\n"));
}

void loop() {
  // ==========================================
  // NHIỆM VỤ 1: LẮNG NGHE LỆNH TỪ TERMINAL (Chạy liên tục không delay)
  // ==========================================
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); 
    command.toUpperCase();

    if (command == "AUTO") {
      isAutoMode = true;
      Serial.println(F("\n[HỆ THỐNG] Chế độ: TỰ ĐỘNG (Cảm biến tự quyết định)"));
    } 
    // --- Điều khiển Quạt thủ công ---
    else if (command == "FAN ON") {
      isAutoMode = false;
      manualFanSpeed = 220; 
      analogWrite(FAN_PIN, manualFanSpeed);
      Serial.println(F("\n[THỦ CÔNG] -> BẬT QUẠT"));
    } 
    else if (command == "FAN OFF") {
      isAutoMode = false;
      manualFanSpeed = 0;
      analogWrite(FAN_PIN, 0);
      Serial.println(F("\n[THỦ CÔNG] -> TẮT QUẠT"));
    } 
    else if (command.startsWith("FAN SPEED ")) {
      isAutoMode = false;
      int s = command.substring(10).toInt();
      if (s >= 0 && s <= 255) {
        manualFanSpeed = s;
        analogWrite(FAN_PIN, manualFanSpeed);
        Serial.print(F("\n[THỦ CÔNG] -> TỐC ĐỘ QUẠT: ")); Serial.println(s);
      } else {
        Serial.println(F("\n[LỖI] Tốc độ quạt từ 0 đến 255!"));
      }
    } 
    // --- Điều khiển Đèn LED thủ công ---
    else if (command == "LED ON") {
      isAutoMode = false;
      manualLedState = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println(F("\n[THỦ CÔNG] -> BẬT ĐÈN LED"));
    } 
    else if (command == "LED OFF") {
      isAutoMode = false;
      manualLedState = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println(F("\n[THỦ CÔNG] -> TẮT ĐÈN LED"));
    } 
    else if (command.length() > 0) {
      Serial.println(F("\n[LỖI] Lệnh sai! Thử lại với: AUTO, FAN ON, FAN OFF, FAN SPEED X, LED ON, LED OFF"));
    }
  }

  // ==========================================
  // NHIỆM VỤ 2: ĐỌC CẢM BIẾN & TỰ ĐỘNG ĐIỀU KHIỂN (Mỗi 2 giây)
  // ==========================================
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; 
    
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float lux = lightMeter.readLightLevel();
    int motionDetected = digitalRead(PIR_PIN);
    
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Lỗi: Không thể đọc dữ liệu từ DHT22!"));
      return; 
    }
    
    // In thông số hệ thống ra Terminal
    Serial.println("\n--- THÔNG SỐ CẢM BIẾN ---");
    Serial.print("Nhiệt độ: "); Serial.print(t); Serial.print(" °C | ");
    Serial.print("Độ ẩm: "); Serial.print(h); Serial.print(" % | ");
    Serial.print("Ánh sáng: "); Serial.print(lux); Serial.println(" lx");
    Serial.print("Chuyển động: "); Serial.println(motionDetected ? "CÓ NGƯỜI" : "KHÔNG CÓ AI");
    
    // XỬ LÝ ĐIỀU KHIỂN THIẾT BỊ
    if (isAutoMode) {
      // 1. Tự động điều khiển QUẠT dựa trên Nhiệt độ + Chuyển động
      int fanSpeed = 0;
      String fanLabel = "TẮT";
      
      if (motionDetected == HIGH) {
        if (t >= 32.0) { fanSpeed = 220; fanLabel = "MẠNH (Cấp 3)"; } 
        else if (t >= 30.0) { fanSpeed = 180; fanLabel = "VỪA (Cấp 2)"; } 
        else if (t >= 28.0) { fanSpeed = 140; fanLabel = "NHẸ (Cấp 1)"; } 
        else { fanSpeed = 0; fanLabel = "TẮT (Phòng mát)"; }
      } else {
        fanSpeed = 0; fanLabel = "TẮT (Không có người)";
      }
      analogWrite(FAN_PIN, fanSpeed);
      
      // 2. Tự động điều khiển ĐÈN dựa trên Ánh sáng + Chuyển động
      bool ledState = false;
      if (motionDetected == HIGH && lux < LUX_THRESHOLD) {
        ledState = true;
        digitalWrite(LED_PIN, HIGH);
      } else {
        ledState = false;
        digitalWrite(LED_PIN, LOW);
      }
      
      // In trạng thái tự động
      Serial.print("[AUTO] Quạt: "); Serial.print(fanLabel);
      Serial.print(" | Đèn LED: "); Serial.println(ledState ? "BẬT (Trời tối + Có người)" : "TẮT");
    } 
    else {
      // Chế độ Thủ công: Giữ nguyên trạng thái do người dùng đặt từ trước
      Serial.print("[MANUAL] Quạt đang ở mức: "); Serial.print(manualFanSpeed);
      Serial.print(" | Đèn LED đang: "); Serial.println(manualLedState ? "BẬT" : "TẮT");
    }
    Serial.println("-------------------------");
  }
}