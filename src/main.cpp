#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- 1. 硬體腳位定義 ---
// 唯一的一組 I2C 匯流排 (MPU6500, OLED, BME280 全部並聯在此)
#define I2C_SDA_PIN 0 
#define I2C_SCL_PIN 1 

#define IBUS_RX_PIN 10      
#define IBUS_TX_PIN 7       

#define SERVO_LEFT_PIN 2    
#define SERVO_RIGHT_PIN 3   
#define SERVO_STEER_PIN 4   

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool display_connected = false;

Servo servoLeft;
Servo servoRight;
Servo servoSteer;

Adafruit_BME280 bme; 
bool baro_connected = false;

// --- 2. 系統變數 ---
uint16_t ibus_channels[14]; 
const uint8_t MPU_ADDR = 0x68; 
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

const int SERVO_MIN = 30;
const int SERVO_MAX = 150;

// 針對 X 朝下安裝，姿態改由 Y 和 Z 負責
float gyroY_offset = 0, gyroZ_offset = 0; 
float ground_pressure = 101325.0; 
float relative_alt = 0.0;
float roll = 0, pitch = 0;
uint32_t last_time;
uint32_t last_print_time = 0; 

String current_mode = "Booting...";

void setup() {
  Serial.begin(115200);
  
  // 初始化唯一的 I2C 匯流排
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
  Wire.setClock(400000); 

  Serial1.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN); 

  // 初始化 OLED 螢幕
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("⚠️ OLED 未連接"));
  } else {
    display_connected = true;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println("C3 F-22");
    display.display();
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  servoLeft.setPeriodHertz(50);
  servoRight.setPeriodHertz(50);
  servoSteer.setPeriodHertz(50);
  
  servoLeft.attach(SERVO_LEFT_PIN, 1000, 2000);
  servoRight.attach(SERVO_RIGHT_PIN, 1000, 2000);
  servoSteer.attach(SERVO_STEER_PIN, 1000, 2000);
  servoLeft.write(90); servoRight.write(90); servoSteer.write(90);

  // 初始化六軸感測器
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // 初始化氣壓計 (預設使用 Wire)
  if (bme.begin(0x76) || bme.begin(0x77)) {
    baro_connected = true;
  } else {
    Serial.println(F("⚠️ BME280 未接上或啟動失敗"));
  }

  Serial.println("\n>>> 系統啟動，請保持飛機靜止 3 秒進行校正 <<<");
  delay(3000); 
  
  // 靜態校正 Y 軸與 Z 軸陀螺儀
  long gyroY_sum = 0, gyroZ_sum = 0;
  int calib_count = 0;
  unsigned long calib_start = millis(); 
  
  while (millis() - calib_start < 3000) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    // 加上 (size_t) 消除編譯器的橘色警告
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true); 
    Wire.read(); Wire.read(); // 捨棄 X 軸不校正
    gyroY_sum += (Wire.read() << 8 | Wire.read()); // 讀取 Y 軸
    gyroZ_sum += (Wire.read() << 8 | Wire.read()); // 讀取 Z 軸
    calib_count++;
    delay(5); 
  }

  gyroY_offset = (float)gyroY_sum / calib_count;
  gyroZ_offset = (float)gyroZ_sum / calib_count;

  if (baro_connected) {
    float p_sum = 0;
    for(int i=0; i<50; i++) { p_sum += bme.readPressure(); delay(10); }
    ground_pressure = p_sum / 50.0;
  }
  
  for (int i = 0; i < 2; i++) {
    servoLeft.write(70); servoRight.write(110); delay(120);
    servoLeft.write(110); servoRight.write(70); delay(120);
  }
  servoLeft.write(90); servoRight.write(90);
  
  last_time = micros(); 
}

void loop() {
  read_ibus();          
  updateSensors();      
  flightControl();      
  printTelemetry();     
  delay(2); 
}

void read_ibus() {
  static uint8_t buffer[32];
  static uint8_t buffer_index = 0;
  static uint32_t last_ibus_time = 0;
  
  if (millis() - last_ibus_time > 10) buffer_index = 0;
  
  while (Serial1.available()) {
    buffer[buffer_index++] = Serial1.read();
    last_ibus_time = millis();
    if (buffer_index == 32) {
      buffer_index = 0;
      if (buffer[0] == 0x20 && buffer[1] == 0x40) {
        uint16_t checksum = 0xFFFF;
        for (int i = 0; i < 30; i++) checksum -= buffer[i];
        if (checksum == (buffer[30] | (buffer[31] << 8))) {
          for (int i = 0; i < 14; i++) ibus_channels[i] = buffer[2 + i * 2] | (buffer[3 + i * 2] << 8);
        }
      }
    }
  }
}

void updateSensors() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  // 加上 (size_t) 消除編譯器的橘色警告
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true); 
  
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); 
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  
  uint32_t current_time = micros();
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;
  if (dt <= 0 || dt > 0.1) dt = 0.005; 

  // === 針對 Y朝前、X朝下、Z朝右 的實體安裝方向 ===
  
  // 1. Pitch (俯仰)：機頭抬起為正
  float accel_pitch = atan2(AcY, sqrt(AcX * AcX + AcZ * AcZ)) * 180.0 / PI; 
  float gyro_rate_pitch = -(GyZ - gyroZ_offset) / 131.0; 
  
  // 2. Roll (翻滾)：右翼下壓為正 (反相處理避免 180 度反轉)
  float accel_roll  = atan2(-AcZ, -AcX) * 180.0 / PI; 
  float gyro_rate_roll = (GyY - gyroY_offset) / 131.0; 
  
  // 3. 互補濾波器融合
  roll  = 0.90 * (roll + gyro_rate_roll * dt) + 0.10 * accel_roll;
  pitch = 0.90 * (pitch + gyro_rate_pitch * dt) + 0.10 * accel_pitch;

  static uint32_t last_baro_time = 0;
  if (baro_connected && (millis() - last_baro_time > 100)) {
    last_baro_time = millis();
    relative_alt = 44330.0 * (1.0 - pow(bme.readPressure() / ground_pressure, 0.1903));
  }
}

void flightControl() {
  int ail_stick = (ibus_channels[0] == 0) ? 1500 : constrain(ibus_channels[0], 1000, 2000); 
  int ele_stick = (ibus_channels[1] == 0) ? 1500 : constrain(ibus_channels[1], 1000, 2000); 
  int rud_stick = (ibus_channels[3] == 0) ? 1500 : constrain(ibus_channels[3], 1000, 2000); 
  int swc_mode  = (ibus_channels[5] == 0) ? 1000 : constrain(ibus_channels[5], 1000, 2000); 
  
  int steer_switch = (ibus_channels[6] == 0) ? 1000 : constrain(ibus_channels[6], 1000, 2000); 
  int trim_knob    = (ibus_channels[7] == 0) ? 1500 : constrain(ibus_channels[7], 1000, 2000); 
  
  int center_trim = map(trim_knob, 1000, 2000, -30, 30); 
  
  int final_steer = 90;

  if (steer_switch > 1600 && ibus_channels[0] != 0) {
    int ail_deviation = ail_stick - 1500;
    int link_steer = map(ail_deviation, -500, 500, 45, -45); 
    final_steer = 90 + link_steer + center_trim;
  } else {
    int ind_steer = map(rud_stick, 1000, 2000, SERVO_MAX, SERVO_MIN);
    final_steer = ind_steer + center_trim;
  }
  
  servoSteer.write(constrain(final_steer, SERVO_MIN, SERVO_MAX));

  float target_roll = 0, target_pitch = 0;

  if (swc_mode < 1300 || ibus_channels[0] == 0) { 
    current_mode = "Manual";
    target_roll = map(ail_stick, 1000, 2000, -45, 45);
    target_pitch = map(ele_stick, 1000, 2000, -45, 45);
  } 
  else if (swc_mode >= 1300 && swc_mode < 1700) {
    current_mode = "Assist";
    target_roll  = map(ail_stick, 1000, 2000, -45, 45);
    target_pitch = map(ele_stick, 1000, 2000, -45, 45);
  } 
  else {
    current_mode = "Comfort";
    target_roll  = map(ail_stick, 1000, 2000, -20, 20);
    target_pitch = map(ele_stick, 1000, 2000, -20, 20);
  }

  float roll_correction = (target_roll - roll) * 1.5;
  float pitch_correction = (target_pitch - pitch) * 1.5;

  int left_angle  = 90 + pitch_correction + roll_correction;
  int right_angle = 90 + pitch_correction - roll_correction; 

  servoLeft.write(constrain(left_angle, SERVO_MIN, SERVO_MAX));
  servoRight.write(constrain(180 - right_angle, SERVO_MIN, SERVO_MAX)); 
}

void printTelemetry() {
  if (millis() - last_print_time > 50) { 
    last_print_time = millis();
    
    // === 電腦端 Serial Monitor 輸出 ===
    Serial.print("姿態 [Roll: "); 
    Serial.print(roll, 1);
    Serial.print("  Pitch: "); 
    Serial.print(pitch, 1);
    Serial.print("] | 高度: ");
    if (baro_connected) {
      Serial.print(relative_alt, 1); 
      Serial.print("m");
    } else {
      Serial.print("未連接");
    }
    Serial.print(" | 轉向: ");
    Serial.print(ibus_channels[6] > 1600 ? "聯動" : "獨立");
    Serial.print(" | 模式: ");
    Serial.println(current_mode);

    // === OLED 小螢幕輸出 ===
    if (display_connected) {
      display.clearDisplay();       
      
      display.setTextSize(1);       
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print("Roll: "); display.print(roll, 1);
      display.setCursor(64, 0);
      display.print("Pit: "); display.print(pitch, 1);
      
      display.setCursor(0, 15);
      if (baro_connected) {
        display.print("Alt: "); 
        display.print(relative_alt, 1); 
        display.print(" m");
      } else {
        display.print("BME280: ERROR");
      }

      display.setCursor(0, 30);
      if(ibus_channels[6] > 1600) {
        display.print("Str: LINKED");
      } else {
        display.print("Str: INDEP.");
      }

      display.setCursor(0, 45);
      display.setTextSize(2);
      display.print(current_mode);
      
      display.display(); 
    }
  }
}
