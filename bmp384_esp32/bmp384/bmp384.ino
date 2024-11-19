#include <Wire.h>
#include <math.h>

#define BMP384_ADDR 0x77  // Adres bmp384
#define REG_PWR_CTRL 0x1B
#define REG_OSR 0x1C
#define REG_PRESS_MSB 0x06
#define REG_TEMP_MSB 0x09
#define CMD_FORCE_MODE 0x01

void setup() {
  Serial.begin(115200);
  Wire.begin();  // 

  BMP384_Init();
  delay(100);
}

void loop() {
  uint32_t pressure_raw;
  float altitude;

  pressure_raw = BMP384_ReadPressureRaw();
  altitude = BMP384_CalculateAltitude(pressure_raw);

  Serial.print("Ciśnienie surowe: ");
  Serial.print(pressure_raw);
  Serial.print(", Wysokość: ");
  Serial.print(altitude);
  Serial.println(" m");

  delay(1000);
}

void BMP384_Init() {
  // Ustawienie trybu pracy na forced mode
  Wire.beginTransmission(BMP384_ADDR);
  Wire.write(REG_PWR_CTRL);
  Wire.write(CMD_FORCE_MODE);
  Wire.endTransmission();

  // Ustawienie oversamplingu (ciśnienie x4, temperatura x1)
  Wire.beginTransmission(BMP384_ADDR);
  Wire.write(REG_OSR);
  Wire.write(0x22);  // osr_t = x1, osr_p = x4
  Wire.endTransmission();
}

uint32_t BMP384_ReadPressureRaw() {
  uint8_t press_data[3];

  Wire.beginTransmission(BMP384_ADDR);
  Wire.write(REG_PRESS_MSB);
  Wire.endTransmission();
  Wire.requestFrom(BMP384_ADDR, 3);

  for (int i = 0; i < 3; i++) {
    press_data[i] = Wire.read();
  }

  return (press_data[0] << 16) | (press_data[1] << 8) | press_data[2];
}

uint32_t BMP384_ReadTemperatureRaw() {
  uint8_t temp_data[3];

  Wire.beginTransmission(BMP384_ADDR);
  Wire.write(REG_TEMP_MSB);
  Wire.endTransmission();
  Wire.requestFrom(BMP384_ADDR, 3);

  for (int i = 0; i < 3; i++) {
    temp_data[i] = Wire.read();
  }

  return (temp_data[0] << 16) | (temp_data[1] << 8) | temp_data[2];
}

float BMP384_CalculateAltitude(uint32_t pressure_raw) {
  const float sea_level_pressure = 101325.0;
  float pressure = (float)pressure_raw / 256.0;
  return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}
