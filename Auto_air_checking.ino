
#include <ModbusRTUSlave.h>
#include <driver/pcnt.h>
#include <Wire.h>
#include "Adafruit_HTU21DF.h"
#include <EEPROM.h>
#define EEPROM_SIZE 64  // dung lượng EEPROM giả lập (tối thiểu đủ cho 3 int)

// Tạo đối tượng cảm biến
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

#define MODBUS_SERIAL Serial

// You can change the baud, config, and unit id values if you like.
// Just make sure they match the settings you use in ModbusRTUMasterExample.
#define MODBUS_BAUD 115200
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1

#define DANG_TEST 2
#define RESULT_OK 3
#define RESULT_TUOT_KEP_NG 4
#define RESULT_AP_SUAT_NG 5
#define RESULT_DO_AM_NG 6
#define RESULT_NG_FULL 7
#define RESULT_KHONG_HOAN_THANH 8

#define INPUT_PIN_0 25
#define INPUT_PIN_1 33
#define TIN_HIEU_AP_SUAT 32
#define TIN_HIEU_KEP 35
#define START_BTN 34
#define STOP_BTN 0

#define VAN_KHI_NEN 17
#define VAN_KHI_NEN_2 16
#define CHAN_CHUYEN 16
#define DEN_DO 4
#define DEN_XANH 2
#define COI 15
#define OUTPUT_PIN_5 13

ModbusRTUSlave modbus(MODBUS_SERIAL, 5);
const uint8_t numCoils = 2;
const uint8_t numDiscreteInputs = 2;
const uint8_t numHoldingRegisters = 15;
const uint8_t numInputRegisters = 2;
uint16_t do_am_toi_thieu = 10;
uint16_t do_am_thuc_te = 18;
uint16_t do_am_toi_da = 50;
uint16_t nhiet_do_thuc_te = 30;
uint16_t thoi_gian_cai_dat = 10;
uint16_t thoi_gian_hien_tai = 0;
uint32_t time_start = 0;
uint8_t manual_mode_start = 0;
int trang_thai_cam_bien = 0;
int trang_thai_cuoi_cung = 0;
struct Config {
  uint16_t do_am_thuc_te_ROM;
  uint16_t do_am_toi_da_ROM;
  uint16_t thoi_gian_cai_dat_ROM;
};
uint16_t holdingRegisters[numHoldingRegisters];
uint16_t current_time;
// Hàm nhấp nháy
unsigned long do_previousMillis = 0;
unsigned long xanh_previousMillis = 0;
unsigned long coi_previousMillis = 0;
bool DEN_DO_STATE = LOW;
bool DEN_XANH_STATE = LOW;
bool COI_STATE = LOW;

void Nhay_Coi(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - coi_previousMillis >= interval) {
    coi_previousMillis = currentMillis;
    COI_STATE = !COI_STATE;              // đảo trạng thái
    digitalWrite(COI, COI_STATE);
  }
}

void Nhay_Den_Do(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - do_previousMillis >= interval) {
    do_previousMillis = currentMillis;
    DEN_DO_STATE = !DEN_DO_STATE;              // đảo trạng thái
    digitalWrite(DEN_DO, DEN_DO_STATE);
  }
}
void Nhay_Den_Xanh(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - xanh_previousMillis >= interval) {
    xanh_previousMillis = currentMillis;
    DEN_XANH_STATE = !DEN_XANH_STATE;              // đảo trạng thái
    digitalWrite(DEN_XANH, DEN_XANH_STATE);
  }
}

Config config;  // biến toàn cục

void saveConfig() {
  EEPROM.put(0, config);   // ghi toàn bộ struct vào địa chỉ 0
  EEPROM.commit();         // bắt buộc để lưu xuống flash
  digitalWrite(OUTPUT_PIN_5, HIGH);
}

void readConfig() {
  EEPROM.get(0, config);   // đọc toàn bộ struct từ địa chỉ 0
}

void setup() {
  pinMode(INPUT_PIN_0, INPUT);
  pinMode(INPUT_PIN_1, INPUT);
  pinMode(TIN_HIEU_AP_SUAT, INPUT);
  pinMode(TIN_HIEU_KEP, INPUT);
  pinMode(START_BTN, INPUT);
  pinMode(STOP_BTN, INPUT);
  pinMode(VAN_KHI_NEN, OUTPUT);
  pinMode(CHAN_CHUYEN, OUTPUT);
  pinMode(DEN_DO, OUTPUT);
  pinMode(DEN_XANH, OUTPUT);
  pinMode(COI, OUTPUT);
  pinMode(OUTPUT_PIN_5, OUTPUT);
  digitalWrite(DEN_DO, HIGH);
  digitalWrite(DEN_XANH, HIGH);
  digitalWrite(COI, HIGH);
  delay(2000);
  digitalWrite(VAN_KHI_NEN, LOW);
  digitalWrite(DEN_DO, LOW);
  digitalWrite(DEN_XANH, LOW);
  digitalWrite(COI, LOW);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  Serial.begin(115200, SERIAL_8N1);
  EEPROM.begin(EEPROM_SIZE);
  readConfig();
  do_am_thuc_te = config.do_am_thuc_te_ROM;
  do_am_toi_da = config.do_am_toi_da_ROM;
  thoi_gian_cai_dat = config.thoi_gian_cai_dat_ROM;
  Serial.println(do_am_thuc_te);
  Serial.println(do_am_toi_da);
  Serial.println(thoi_gian_cai_dat);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
  Wire.begin(26, 27);

  // Khởi tạo cảm biến
  if (!htu.begin()) {
    trang_thai_cam_bien = 1;
    while(millis() < 10000)
    if (trang_thai_cam_bien == 1)
    {
      Nhay_Den_Do(200);
      Nhay_Den_Xanh(200);
    }
    }

  
  // Cấu hình chân INT0 (PD0) làm input
  holdingRegisters[1] = do_am_toi_thieu;
  holdingRegisters[2] = do_am_toi_da;
  holdingRegisters[3] = thoi_gian_cai_dat;
}

void loop() 
{
  if ((holdingRegisters[2] != do_am_toi_da) || (holdingRegisters[3] != thoi_gian_cai_dat))
  {
    if (trang_thai_cam_bien == 0)
    {
      config.do_am_thuc_te_ROM = (uint16_t)do_am_thuc_te;
    }
    else
    {
      config.do_am_thuc_te_ROM = holdingRegisters[2] - 1;
    }
    
    config.do_am_toi_da_ROM = holdingRegisters[2];
    config.thoi_gian_cai_dat_ROM = holdingRegisters[3];
    saveConfig();
  }
  if (holdingRegisters[0] == 1)
  {
    time_start = millis();
    holdingRegisters[0] = 2;
  }
  do_am_toi_thieu = holdingRegisters[1];
  do_am_toi_da = holdingRegisters[2];
  thoi_gian_cai_dat = holdingRegisters[3];
  if (trang_thai_cam_bien == 0)
  {
    float temp = htu.readTemperature();
    float hum = htu.readHumidity();
    do_am_thuc_te = (int)hum;
    nhiet_do_thuc_te = int(temp);
  }

  holdingRegisters[8] = do_am_thuc_te;
  holdingRegisters[7] = nhiet_do_thuc_te;
  if (digitalRead(TIN_HIEU_KEP) == HIGH)
  {
    holdingRegisters[4] = 1;
  }
  else
  {
    holdingRegisters[4] = 0;
  }
  
  if (digitalRead(TIN_HIEU_AP_SUAT) == LOW)
  {
    holdingRegisters[5] = 1;
  }
  else
  {
    holdingRegisters[5] = 0;
  }

  if ((do_am_toi_thieu <= do_am_thuc_te) && (do_am_toi_da >= do_am_thuc_te))
  {
    holdingRegisters[6] = 1;
  }
  else
  {
    holdingRegisters[6] = 0;
  }

  // ------------------------------------------------------------
  if(holdingRegisters[0] == 2) 
  {
    digitalWrite(VAN_KHI_NEN, HIGH);
    // digitalWrite(VAN_KHI_NEN_2, HIGH);
    current_time = (uint16_t)((millis()-time_start)/1000);
    holdingRegisters[9] = current_time;

    if (current_time < thoi_gian_cai_dat)
    {
      if(digitalRead(TIN_HIEU_KEP) == LOW)
      {
        delay(10);
        if(digitalRead(TIN_HIEU_KEP) == LOW)
        {
            holdingRegisters[0] = RESULT_TUOT_KEP_NG;
            digitalWrite(CHAN_CHUYEN, HIGH);
            digitalWrite(VAN_KHI_NEN, LOW);
        }
      }
    }
    if (current_time >= thoi_gian_cai_dat)
    {
        
        if ((do_am_thuc_te > do_am_toi_thieu) && (do_am_thuc_te < do_am_toi_da) && (digitalRead(TIN_HIEU_AP_SUAT) == LOW))
        {
            holdingRegisters[0] = RESULT_OK;
            digitalWrite(CHAN_CHUYEN, LOW);
        }
        else
        {
          digitalWrite(CHAN_CHUYEN, HIGH);
        }

        if ((do_am_thuc_te < do_am_toi_thieu) || (do_am_thuc_te > do_am_toi_da) )
        {
            holdingRegisters[0] = RESULT_DO_AM_NG;
        }
        if (digitalRead(TIN_HIEU_AP_SUAT) == HIGH)
        {
            holdingRegisters[0] = RESULT_AP_SUAT_NG;
        }
        if (((do_am_thuc_te < do_am_toi_thieu) || (do_am_thuc_te > do_am_toi_da)) && (digitalRead(TIN_HIEU_AP_SUAT) == HIGH))
        {
            holdingRegisters[0] = RESULT_NG_FULL;
        }
        digitalWrite(VAN_KHI_NEN, LOW);
        // digitalWrite(VAN_KHI_NEN_2, LOW);
    }
  }

    
  if (digitalRead(STOP_BTN) == LOW)
  {
    delay(5);
    if(digitalRead(STOP_BTN) == LOW)
    {
      digitalWrite(VAN_KHI_NEN, LOW);
      // digitalWrite(VAN_KHI_NEN_2, LOW);
      holdingRegisters[0] = RESULT_KHONG_HOAN_THANH;
      current_time = 0;
    }
  }
  if (digitalRead(START_BTN) == LOW)
  {
    if ((holdingRegisters[0] != 1) && (holdingRegisters[0] != 2))
    {
      time_start = millis();
      holdingRegisters[0] = 2;
    }
  }
  if ((holdingRegisters[0] == RESULT_KHONG_HOAN_THANH) || (holdingRegisters[0] == RESULT_TUOT_KEP_NG) || (holdingRegisters[0] == RESULT_AP_SUAT_NG) || (holdingRegisters[0] == RESULT_DO_AM_NG) || (holdingRegisters[0] == RESULT_NG_FULL))
  {
    Nhay_Den_Do(500);
    digitalWrite(DEN_XANH, LOW);
  }
  else if (holdingRegisters[0] == RESULT_OK)
  {
    digitalWrite(DEN_XANH, HIGH);
    digitalWrite(DEN_DO, LOW);
  }
  else if (holdingRegisters[0] == DANG_TEST)
  {
    Nhay_Den_Xanh(500);
    digitalWrite(DEN_DO, LOW);
  }
  if ((holdingRegisters[0] == RESULT_TUOT_KEP_NG) || (holdingRegisters[0] == RESULT_AP_SUAT_NG))
  {
    // Nhay_Coi(500);
    digitalWrite(COI, HIGH);
  }
  else
  {
    digitalWrite(COI, LOW);
  }
  modbus.poll();


}
// python -m esptool --chip esp32 --port COM10 write_flash -z 0x1000 bootloader.bin 0x8000 partition-table.bin 0x10000 firmware.bin

