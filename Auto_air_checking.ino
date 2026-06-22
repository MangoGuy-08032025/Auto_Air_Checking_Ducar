
#include <ModbusRTUSlave.h>
#include <driver/pcnt.h>
#include <Wire.h>
#include "Adafruit_HTU21DF.h"
#include <EEPROM.h>
#include "ota_update.h"
#define EEPROM_SIZE 64  // dung lượng EEPROM giả lập (tối thiểu đủ cho 3 int)
#define DEFAULT_WIFI_OTA    "Linh Anh"
#define DEFAULT_PASS_OTA    "08032025"
// Tạo đối tượng cảm biến
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

#define MODBUS_SERIAL Serial

// You can change the baud, config, and unit id values if you like.
// Just make sure they match the settings you use in ModbusRTUMasterExample.
#define MODBUS_BAUD 115200
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1

#define TRANG_THAI_KHOI_TAO 0
#define BAT_DAU_TEST 1
#define DANG_TEST 2
#define RESULT_OK 3
#define RESULT_TUOT_TRANG_THAI_NG 4
#define RESULT_AP_SUAT_NG 5
#define RESULT_DO_AM_NG 6
#define RESULT_NG_FULL 7
#define RESULT_KHONG_HOAN_THANH 8

#define TRANG_THAI_OK 1
#define TRANG_THAI_NG 0

#define INPUT_PIN_0 25
#define INPUT_PIN_1 33
#define TIN_HIEU_AP_SUAT 32
#define TIN_HIEU_KEP 35
#define START_BTN 34
#define OTA_BTN 12
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
String current_version = "1";
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
bool isOn = false;     // trạng thái chân
uint16_t startMillis = 0; // thời điểm bật
Config config;  // biến toàn cục

void saveConfig() {
  EEPROM.put(0, config);   // ghi toàn bộ struct vào địa chỉ 0
  EEPROM.commit();         // bắt buộc để lưu xuống flash
  digitalWrite(OUTPUT_PIN_5, HIGH);
}

void readConfig() {
  EEPROM.get(0, config);   // đọc toàn bộ struct từ địa chỉ 0
}

// Hàm con: bật chân trong 1s rồi tự tắt
void pulseOneSecond(int pin) {
    if (startMillis < 20)
    {
      digitalWrite(COI, HIGH);
      delay(1);
    }
    else
    {
      digitalWrite(COI, LOW);
    }
    if (startMillis > 20)
    {
      startMillis = 20;
    }
    startMillis =  startMillis + 1;
}

void setup() {
  Serial.begin(115200, SERIAL_8N1);
  pinMode(INPUT_PIN_0, INPUT);
  pinMode(INPUT_PIN_1, INPUT);
  pinMode(TIN_HIEU_AP_SUAT, INPUT);
  pinMode(TIN_HIEU_KEP, INPUT);
  pinMode(START_BTN, INPUT);
  pinMode(STOP_BTN, INPUT);
  pinMode(OTA_BTN, INPUT_PULLUP);
  pinMode(VAN_KHI_NEN, OUTPUT);
  pinMode(CHAN_CHUYEN, OUTPUT);
  pinMode(DEN_DO, OUTPUT);
  pinMode(DEN_XANH, OUTPUT);
  pinMode(COI, OUTPUT);
  pinMode(OUTPUT_PIN_5, OUTPUT);
  if ((digitalRead(START_BTN) == LOW) )
  {
    WiFi.begin(DEFAULT_WIFI_OTA, DEFAULT_PASS_OTA);
    Serial.print(DEFAULT_WIFI_OTA);
    Serial.print("-");
    Serial.println(DEFAULT_PASS_OTA);
    while (WiFi.status() != WL_CONNECTED) 
    {
      Serial.print(".");
      delay(50);
      digitalWrite(DEN_XANH, HIGH);
      digitalWrite(DEN_DO, HIGH);
      digitalWrite(COI, HIGH);
    }
    digitalWrite(COI, LOW);
    checkAndUpdateFirmware();  // gọi hàm OTA
    digitalWrite(DEN_XANH, LOW);
    digitalWrite(DEN_DO, HIGH);
    digitalWrite(COI, HIGH);
    while(1);
  }
  EEPROM.begin(EEPROM_SIZE);
  readConfig();
  do_am_thuc_te = config.do_am_thuc_te_ROM;
  do_am_toi_da = config.do_am_toi_da_ROM;
  thoi_gian_cai_dat = config.thoi_gian_cai_dat_ROM;
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
  digitalWrite(COI, LOW);
  digitalWrite(DEN_DO, LOW);
  digitalWrite(DEN_XANH, LOW);
  // Cấu hình chân INT0 (PD0) làm input
  holdingRegisters[1] = do_am_toi_thieu;
  holdingRegisters[2] = do_am_toi_da;
  holdingRegisters[3] = thoi_gian_cai_dat;
  current_version = FW_VERSION;
  holdingRegisters[10] = current_version.toInt();
  Wire.begin(26, 27);
  delay(100);
  // Khởi tạo cảm biến
  for (int i = 0; i< 5; i++)
  {
    if (!htu.begin()) 
    {
      trang_thai_cam_bien = 1;
      if (trang_thai_cam_bien == 1)
      {
        digitalWrite(DEN_DO, HIGH);
        digitalWrite(DEN_XANH, HIGH);
        digitalWrite(COI, HIGH);
        delay(1000);
      }
    }
    else
    {
      trang_thai_cam_bien = 0;
      digitalWrite(DEN_XANH, HIGH);
      delay(2000);
      break;
    }
  }


}

void loop() 
{
  // Đọc nhiệt độ - độ ẩm
  if (trang_thai_cam_bien == 0)
  {
    float temp = htu.readTemperature();
    float hum = htu.readHumidity();
    if (hum < 1.0)
    {
      hum = 1.0;
    }
    do_am_thuc_te = (int)hum;
    if (do_am_thuc_te <= 1) 
    {
      do_am_thuc_te = 1;
      digitalWrite(OUTPUT_PIN_5, HIGH);
    }
    nhiet_do_thuc_te = int(temp);
  }
  holdingRegisters[8] = (uint16_t)do_am_thuc_te;
  holdingRegisters[7] = (uint16_t)nhiet_do_thuc_te;

  // Nếu thanh ghi 0 == 1(Bắt đầu test)
  if (holdingRegisters[0] == BAT_DAU_TEST)
  {
    time_start = millis();
    holdingRegisters[0] = DANG_TEST;
    do_am_toi_thieu = holdingRegisters[1];
    do_am_toi_da = holdingRegisters[2];
    thoi_gian_cai_dat = holdingRegisters[3];
  }

  // Nếu thanh ghi 0 == 2 (Đang test)
  if(holdingRegisters[0] == DANG_TEST) 
  {
    startMillis = 0;
    digitalWrite(VAN_KHI_NEN, HIGH);
    current_time = (uint16_t)((millis()-time_start)/1000);
    holdingRegisters[9] = current_time;

    // Khi đang test mà kẹp bị tuột -> Dừng test, trả kết quả RESULT_TUOT_TRANG_THAI_NG
    if (current_time < thoi_gian_cai_dat)
    {
      if(digitalRead(TIN_HIEU_KEP) == LOW)
      {
        delay(10);
        if(digitalRead(TIN_HIEU_KEP) == LOW)
        {
            holdingRegisters[0] = RESULT_TUOT_TRANG_THAI_NG;
            digitalWrite(CHAN_CHUYEN, HIGH);
            digitalWrite(VAN_KHI_NEN, LOW);
        }
      }
    }

    // Khi thời gian vượt quá thời gian cài đặt
    if (current_time >= thoi_gian_cai_dat)
    {  
      // Nếu độ ẩm thực tế nằm trong khoảng cài đặt và có tín hiệu đủ áp suất -> trả kết quả RESULT_OK
      if ((do_am_thuc_te < do_am_toi_da) && (digitalRead(TIN_HIEU_AP_SUAT) == LOW))
      {
          holdingRegisters[0] = RESULT_OK;
          digitalWrite(CHAN_CHUYEN, LOW);
      }
      // Nếu nếu không đạt kết quả OK thì bật chặn chuyền
      else
      {
        digitalWrite(CHAN_CHUYEN, HIGH);
      }
      // Nếu độ nằm ngoài ngưỡng thì trả về kết quả RESULT_DO_AM_NG
      if (do_am_thuc_te > do_am_toi_da)
      {
          holdingRegisters[0] = RESULT_DO_AM_NG;
      }
      // Nếu không đạt áp suất thì trả về kết quả RESULT_AP_SUAT_NG
      if (digitalRead(TIN_HIEU_AP_SUAT) == HIGH)
      {
          holdingRegisters[0] = RESULT_AP_SUAT_NG;
      }
      // Nếu không đảm bảo cả áp suất và độ ẩm thì trả về kết quả RESULT_NG_FULL
      if ((do_am_thuc_te > do_am_toi_da) && (digitalRead(TIN_HIEU_AP_SUAT) == HIGH))
      {
          holdingRegisters[0] = RESULT_NG_FULL;
      }
      // Tắt van test khí
      digitalWrite(VAN_KHI_NEN, LOW);
    }
  }
  // Nếu ấn nút dừng thì trả về kết quả RESULT_KHONG_HOAN_THANH
  if (digitalRead(STOP_BTN) == LOW)
  {
    delay(2);
    if(digitalRead(STOP_BTN) == LOW)
    {
      digitalWrite(VAN_KHI_NEN, LOW);
      holdingRegisters[0] = RESULT_KHONG_HOAN_THANH;
      current_time = 0;
    }
  }
  // Nếu ấn nút Start thì cắt đầu test
  if (digitalRead(START_BTN) == LOW)
  {
    if ((holdingRegisters[0] != BAT_DAU_TEST) && (holdingRegisters[0] != DANG_TEST))
    {
      time_start = millis();
      holdingRegisters[0] = DANG_TEST;
    }
  }

  // Nếu kết quả NG thì tắt đèn xanh, sáng đèn đỏ
  if ((holdingRegisters[0] == RESULT_KHONG_HOAN_THANH) || (holdingRegisters[0] == RESULT_TUOT_TRANG_THAI_NG) || (holdingRegisters[0] == RESULT_AP_SUAT_NG) || (holdingRegisters[0] == RESULT_DO_AM_NG) || (holdingRegisters[0] == RESULT_NG_FULL))
  {
    digitalWrite(DEN_DO, HIGH);
    digitalWrite(DEN_XANH, LOW);
    digitalWrite(COI, HIGH);
  }
  // Nếu kết quả OK thì tắt đèn đỏ, sáng đèn xanh
  else if (holdingRegisters[0] == RESULT_OK)
  {
    digitalWrite(DEN_XANH, LOW);
    digitalWrite(DEN_DO, HIGH);
  }
  // Nếu đang test thì tắt đèn đỏ, sáng đèn xanh
  else if (holdingRegisters[0] == DANG_TEST)
  {
    digitalWrite(DEN_DO, LOW);
    digitalWrite(DEN_XANH, HIGH);
    digitalWrite(COI, LOW);
  }

  // Nếu thấy giá trị độ ẩm tối đa và thời gian cài đặt thay đổi thì lưu lại vào ROM
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
  // Trạng thái kẹp trên Modbus
  if (digitalRead(TIN_HIEU_KEP) == HIGH)
  {
    holdingRegisters[4] = TRANG_THAI_OK;
  }
  else
  {
    holdingRegisters[4] = TRANG_THAI_NG;
  }
  // Trạng thái áp suất trên Modbus
  if (digitalRead(TIN_HIEU_AP_SUAT) == LOW)
  {
    holdingRegisters[5] = TRANG_THAI_OK;
  }
  else
  {
    holdingRegisters[5] = TRANG_THAI_NG;
  }
  // Trạng thái độ ẩm trên Modbus
  if (do_am_thuc_te <= do_am_toi_da)
  {
    holdingRegisters[6] = TRANG_THAI_OK;
  }
  else
  {
    holdingRegisters[6] = TRANG_THAI_NG;
  }

  // Nếu thanh ghi 0 ở trạng thái khời tạo thì tắt chặn chuyền, đồng thời, cập nhận giá trị độ ẩm tối thiểu, độ ẩm tối đa và thời gian cài đặt
  if (holdingRegisters[0] == TRANG_THAI_KHOI_TAO)
  {
    digitalWrite(CHAN_CHUYEN, LOW);
    do_am_toi_thieu = holdingRegisters[1];
    do_am_toi_da = holdingRegisters[2];
    thoi_gian_cai_dat = holdingRegisters[3];
  }
  // Nếu kết quả test OK thì bật còi kêu trong vòng 1s
  if (holdingRegisters[0] == RESULT_OK)
  {
    pulseOneSecond(COI);
  }

  modbus.poll();
}
// python -m esptool --chip esp32 --port COM10 write_flash -z 0x1000 bootloader.bin 0x8000 partition-table.bin 0x10000 firmware.bin

