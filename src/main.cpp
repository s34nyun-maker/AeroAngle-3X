#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// ==========================================
// 1. 하드웨어 핀맵 정의 (ESP32-C3 0.42 OLED 보드)
// ==========================================
#define OLED_SDA 5
#define OLED_SCL 6       // OLED 및 MPU6050 3개의 공통 SCL 핀
#define BTN_ZERO 9       // 보드 내장 BO0 버튼 (Active LOW)
#define LED_PIN  8       // 보드 내장 상태 LED (Active LOW)

#define MPU_ADDR 0x68    // MPU6050 기본 I2C 주소

// 3개의 MPU6050 모듈을 위한 개별 SDA 핀 배열
const uint8_t mpu_sda_pins[3] = {0, 1, 2};
const char* ch_names[3] = {"L-E", "R-E", "RUD"}; // 좌엘리베이터, 우엘리베이터, 러더

// 0.42인치 OLED (72x40 픽셀) 전용 U8g2 객체 생성
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA);

// ==========================================
// 2. 센서 채널별 데이터 구조체
// ==========================================
struct SensorChannel {
  uint8_t sda_pin;
  bool connected;
  
  // 자이로 영점 오차 보정값
  float gyro_offset_x;
  float gyro_offset_y;
  float gyro_offset_z;
  
  // 실시간 필터링된 3축 각도 (X: Pitch/Roll, Y: Roll/Pitch, Z: Yaw)
  float angle_x;
  float angle_y;
  float angle_z;
  
  // BO0 버튼 클릭 시 저장되는 기준 영점 각도
  float zero_x;
  float zero_y;
  float zero_z;
  
  // 영점 대비 현재 변화 각도 (+ / -)
  float delta_x;
  float delta_y;
  float delta_z;
  
  // 자동 선택된 축 (0: 대기중 '-', 1: 'X', 2: 'Y', 3: 'Z')
  uint8_t active_axis;
  float display_angle;   // 최종 화면에 표시할 각도
};

SensorChannel ch[3];
unsigned long last_time = 0;

// ==========================================
// 3. I2C 핀 동적 스위칭 및 MPU6050 제어 함수
// ==========================================

// 특정 데이터(SDA) 핀으로 I2C 버스를 전환하는 함수
void selectI2CPins(uint8_t sda_pin) {
  Wire.end();
  Wire.begin(sda_pin, OLED_SCL, 400000); // 400kHz Fast I2C
}

// MPU6050 초기화 (슬립 모드 해제 및 감도 설정)
bool initMPU6050(uint8_t idx) {
  selectI2CPins(ch[idx].sda_pin);
  
  // 연결 확인
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    return false; // 센서 응답 없음
  }
  
  // PWR_MGMT_1 (0x6B) -> 0x00 (슬립 해제)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // CONFIG (0x1A) -> 0x03 (DLPF 디지털 저역통과 필터 44Hz 설정 - 진동 노이즈 제거)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  
  // GYRO_CONFIG (0x1B) -> 0x08 (±500 deg/s 범위 설정)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  // ACCEL_CONFIG (0x1C) -> 0x00 (±2g 범위 설정)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// MPU6050 원시 데이터(가속도 + 자이로) 읽기
bool readMPURaw(uint8_t idx, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  selectI2CPins(ch[idx].sda_pin);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H 레지스터부터 14바이트 요청
  if (Wire.endTransmission(false) != 0) return false;
  
  uint8_t bytes = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (bytes < 14) return false;
  
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // 온도 데이터(2바이트) 스킵
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  
  return true;
}

// ==========================================
// 4. 자이로 초기 오차 보정 및 영점(Zero) 설정 함수
// ==========================================

// 부팅 시 정지 상태에서 자이로 오프셋 계산
void calibrateGyros() {
  const int samples = 150;
  for (int i = 0; i < 3; i++) {
    ch[i].gyro_offset_x = 0;
    ch[i].gyro_offset_y = 0;
    ch[i].gyro_offset_z = 0;
    if (!ch[i].connected) continue;
    
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int valid_cnt = 0;
    
    for (int s = 0; s < samples; s++) {
      int16_t ax, ay, az, gx, gy, gz;
      if (readMPURaw(i, ax, ay, az, gx, gy, gz)) {
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        valid_cnt++;
      }
      delay(2);
    }
    if (valid_cnt > 0) {
      // ±500 deg/s 설정 시 LSB Sensitivity = 65.5 LSB/(deg/s)
      ch[i].gyro_offset_x = (sum_gx / (float)valid_cnt) / 65.5f;
      ch[i].gyro_offset_y = (sum_gy / (float)valid_cnt) / 65.5f;
      ch[i].gyro_offset_z = (sum_gz / (float)valid_cnt) / 65.5f;
    }
    
    // 초기 가속도 기반 각도 세팅
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPURaw(i, ax, ay, az, gx, gy, gz)) {
      ch[i].angle_x = atan2(ay, hypot(ax, az)) * 180.0f / PI;
      ch[i].angle_y = atan2(-ax, hypot(ay, az)) * 180.0f / PI;
      ch[i].angle_z = 0.0f;
    }
  }
}

// BO0 버튼 클릭 시 현재 각도를 0도로 리셋하고 자동 축 선택 초기화
void setZeroAngles() {
  digitalWrite(LED_PIN, LOW); // LED 켜기
  
  for (int i = 0; i < 3; i++) {
    // 자이로 적분 축(Z축) 리셋 및 현재 필터 각도를 영점으로 지정
    ch[i].angle_z = 0.0f;
    ch[i].zero_x  = ch[i].angle_x;
    ch[i].zero_y  = ch[i].angle_y;
    ch[i].zero_z  = 0.0f;
    
    ch[i].delta_x = 0.0f;
    ch[i].delta_y = 0.0f;
    ch[i].delta_z = 0.0f;
    
    ch[i].active_axis = 0;     // 축 자동 선택 대기 상태로 초기화
    ch[i].display_angle = 0.0f;
  }
  
  // OLED 화면에 영점 완료 메시지 표시
  selectI2CPins(OLED_SDA);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawFrame(0, 0, 72, 40);
  u8g2.drawStr(9, 24, "ZERO SET!");
  u8g2.sendBuffer();
  
  delay(500);
  digitalWrite(LED_PIN, HIGH); // LED 끄기
  last_time = micros();        // 적분 타이머 갱신
}

// ==========================================
// 5. 초기화 (Setup)
// ==========================================
void setup() {
  pinMode(BTN_ZERO, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 초기화 중 LED 점등
  
  // OLED 초기화 및 부팅 화면 표시
  Wire.begin(OLED_SDA, OLED_SCL, 400000);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(4, 15, "RC THROW METER");
  u8g2.drawStr(8, 30, "CAL GYRO...");
  u8g2.sendBuffer();
  
  // 3개 채널 MPU6050 초기화
  for (int i = 0; i < 3; i++) {
    ch[i].sda_pin = mpu_sda_pins[i];
    ch[i].connected = initMPU6050(i);
  }
  
  // 정지 상태 자이로 오프셋 보정 (약 1초 소요)
  calibrateGyros();
  
  // 초기 영점 세팅
  setZeroAngles();
  last_time = micros();
}

// ==========================================
// 6. 메인 루프 (Loop)
// ==========================================
void loop() {
  // 1) BO0 버튼(GPIO 9) 입력 확인 (영점 조절)
  if (digitalRead(BTN_ZERO) == LOW) {
    delay(30); // 디바운싱
    if (digitalRead(BTN_ZERO) == LOW) {
      setZeroAngles();
      while (digitalRead(BTN_ZERO) == LOW) { delay(10); } // 버튼 뗄 때까지 대기
    }
  }
  
  // 루프 경과 시간(dt, 단위: 초) 계산
  unsigned long now = micros();
  float dt = (now - last_time) / 1000000.0f;
  last_time = now;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
  
  // 2) 3개 채널 센서 읽기 및 각도 계산
  for (int i = 0; i < 3; i++) {
    if (!ch[i].connected) {
      // 연결이 끊겼던 센서가 다시 연결되었는지 재시도
      ch[i].connected = initMPU6050(i);
      continue;
    }
    
    int16_t ax, ay, az, gx, gy, gz;
    if (!readMPURaw(i, ax, ay, az, gx, gy, gz)) {
      ch[i].connected = false;
      continue;
    }
    
    // 가속도 기반 X, Y 절대 기울기 각도 (단위: 도)
    // atan2와 hypot을 사용하여 날개가 ±90도까지 꺾여도 짐벌락/비선형 왜곡 최소화
    float acc_angle_x = atan2(ay, hypot(ax, az)) * 180.0f / PI;
    float acc_angle_y = atan2(-ax, hypot(ay, az)) * 180.0f / PI;
    
    // 자이로 각속도 계산 (deg/s) - 오프셋 차감
    float gyro_rate_x = (gx / 65.5f) - ch[i].gyro_offset_x;
    float gyro_rate_y = (gy / 65.5f) - ch[i].gyro_offset_y;
    float gyro_rate_z = (gz / 65.5f) - ch[i].gyro_offset_z;
    
    // 정지 상태 미세 드리프트 방지용 데드밴드(Deadband) 적용
    if (fabs(gyro_rate_x) < 0.35f) gyro_rate_x = 0.0f;
    if (fabs(gyro_rate_y) < 0.35f) gyro_rate_y = 0.0f;
    if (fabs(gyro_rate_z) < 0.35f) gyro_rate_z = 0.0f;
    
    // 상보 필터 (X, Y축: 가속도 + 자이로 융합 / Z축: 수직 러더 대응을 위한 자이로 적분)
    ch[i].angle_x = 0.96f * (ch[i].angle_x + gyro_rate_x * dt) + 0.04f * acc_angle_x;
    ch[i].angle_y = 0.96f * (ch[i].angle_y + gyro_rate_y * dt) + 0.04f * acc_angle_y;
    ch[i].angle_z = ch[i].angle_z + gyro_rate_z * dt;
    
    // 영점(0도) 대비 변화량(+, -) 계산
    ch[i].delta_x = ch[i].angle_x - ch[i].zero_x;
    ch[i].delta_y = ch[i].angle_y - ch[i].zero_y;
    ch[i].delta_z = ch[i].angle_z - ch[i].zero_z;
    
    // 3) 움직임 감지 및 측정 축(X, Y, Z) 자동 선택 로직
    float abs_x = fabs(ch[i].delta_x);
    float abs_y = fabs(ch[i].delta_y);
    float abs_z = fabs(ch[i].delta_z);
    
    // 가장 크게 움직인 축 찾기
    float max_delta = abs_x;
    uint8_t max_axis = 1; // 1: X축
    
    if (abs_y > max_delta) {
      max_delta = abs_y;
      max_axis = 2;       // 2: Y축
    }
    if (abs_z > max_delta) {
      max_delta = abs_z;
      max_axis = 3;       // 3: Z축
    }
    
    // 날개가 2.0도 이상 움직이면 가장 변화가 큰 축을 해당 센서의 측정 축으로 자동 고정/갱신
    if (max_delta > 2.0f) {
      ch[i].active_axis = max_axis;
    }
    
    // 선택된 축에 따른 최종 표시 각도 할당 (+ / - 방향 포함)
    if (ch[i].active_axis == 1) {
      ch[i].display_angle = ch[i].delta_x;
    } else if (ch[i].active_axis == 2) {
      ch[i].display_angle = ch[i].delta_y;
    } else if (ch[i].active_axis == 3) {
      ch[i].display_angle = ch[i].delta_z;
    } else {
      // 아직 2도 이상 움직이지 않은 초기 상태에서는 현재 가장 큰 값 표시
      if (max_axis == 1) ch[i].display_angle = ch[i].delta_x;
      else if (max_axis == 2) ch[i].display_angle = ch[i].delta_y;
      else ch[i].display_angle = ch[i].delta_z;
    }
  }
  
  // 4) 내장 OLED 화면 업데이트 (I2C 버스를 OLED 핀으로 복귀 후 전송)
  selectI2CPins(OLED_SDA);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tr); // 72x40 화면에 3줄이 선명하게 들어가는 폰트
  
  const char axis_char[4] = {'-', 'X', 'Y', 'Z'};
  
  for (int i = 0; i < 3; i++) {
    int y_pos = 10 + (i * 13); // 각 줄의 Y 좌표 (10, 23, 36)
    char buf[24];
    
    if (!ch[i].connected) {
      snprintf(buf, sizeof(buf), "%s: ERR", ch_names[i]);
    } else {
      // 미세한 노이즈로 인해 -0.0이 뜨는 현상 방지
      float val = ch[i].display_angle;
      if (fabs(val) < 0.1f) val = 0.0f;
      
      // 출력 포맷 예시 -> "L-E[X]:+15.2" 또는 "RUD[Z]: -8.5"
      snprintf(buf, sizeof(buf), "%s[%c]:%+5.1f", ch_names[i], axis_char[ch[i].active_axis], val);
    }
    u8g2.drawStr(1, y_pos, buf);
  }
  
  u8g2.sendBuffer();
  delay(10); // 약 80~100Hz 주기로 동작
}