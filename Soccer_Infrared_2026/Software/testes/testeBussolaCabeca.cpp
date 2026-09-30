// Teste dedicado da bussola QMC5883P na placa Cabeca.
// Funcao: iniciar sensor, calibrar min/max XY e calcular heading 0..360.
// Saida: leituras RAW e heading em graus no monitor serial.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ 100000

const uint8_t QMC5883P_ADDR = 0x2C;

// Valores de calibração
const float xOffset = 711.50;
const float yOffset = -1624.00;
const float xScale  = 1.013703;
const float yScale  = 0.986663;

int head = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission(true);

  if (err != 0) {
    Serial.print("Erro I2C writeReg reg 0x");
    Serial.print(reg, HEX);
    Serial.print(" -> codigo ");
    Serial.println(err);
    return false;
  }
  return true;
}

bool readReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(reg);

  uint8_t err = Wire.endTransmission(false); // repeated start
  if (err != 0) {
    Serial.print("Erro I2C readReg(endTransmission) reg 0x");
    Serial.print(reg, HEX);
    Serial.print(" -> codigo ");
    Serial.println(err);
    return false;
  }

  size_t n = Wire.requestFrom((int)QMC5883P_ADDR, 1, true);
  if (n != 1) {
    Serial.print("Erro I2C readReg(requestFrom) reg 0x");
    Serial.print(reg, HEX);
    Serial.print(" -> recebidos ");
    Serial.println((int)n);
    return false;
  }

  value = Wire.read();
  return true;
}

void scanI2C() {
  Serial.println("\nEscaneando barramento I2C...");
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("Dispositivo encontrado em 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("Nenhum dispositivo I2C encontrado.");
  }
}

bool initQMC5883P() {
  delay(20);

  bool ok = true;
  ok &= writeReg(0x29, 0x06);
  ok &= writeReg(0x0B, 0x08);
  ok &= writeReg(0x0A, 0xC3);

  delay(20);
  return ok;
}

bool readQMC5883PData(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t status = 0;

  if (!readReg(0x09, status)) {
    return false;
  }

  if ((status & 0x01) == 0) {
    return false; // dado ainda nao pronto
  }

  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(0x01);

  uint8_t err = Wire.endTransmission(false); // repeated start
  if (err != 0) {
    Serial.print("Erro I2C leitura bloco -> codigo ");
    Serial.println(err);
    return false;
  }

  size_t n = Wire.requestFrom((int)QMC5883P_ADDR, 6, true);
  if (n != 6) {
    Serial.print("Leitura incompleta: ");
    Serial.println((int)n);
    return false;
  }

  uint8_t x_lsb = Wire.read();
  uint8_t x_msb = Wire.read();
  uint8_t y_lsb = Wire.read();
  uint8_t y_msb = Wire.read();
  uint8_t z_lsb = Wire.read();
  uint8_t z_msb = Wire.read();

  x = (int16_t)((x_msb << 8) | x_lsb);
  y = (int16_t)((y_msb << 8) | y_lsb);
  z = (int16_t)((z_msb << 8) | z_lsb);

  return true;
}

int calcularHead(int16_t xRaw, int16_t yRaw) {
  float xCorr = (xRaw - xOffset) * xScale;
  float yCorr = (yRaw - yOffset) * yScale;

  float ang = atan2(-yCorr, xCorr) * 180.0 / PI;

  if (ang < 0) ang += 360.0;

  return (int)ang;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nIniciando I2C no ESP32-S3...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(20);

  Serial.print("SDA = ");
  Serial.println(I2C_SDA);
  Serial.print("SCL = ");
  Serial.println(I2C_SCL);

  scanI2C();

  if (!initQMC5883P()) {
    Serial.println("Falha ao inicializar QMC5883P.");
  } else {
    Serial.println("QMC5883P inicializado.");
  }

  uint8_t chipID = 0;
  if (readReg(0x00, chipID)) {
    Serial.print("CHIP ID: 0x");
    Serial.println(chipID, HEX);
  } else {
    Serial.println("Falha ao ler CHIP ID.");
  }

  Serial.print("xOffset: ");
  Serial.println(xOffset);
  Serial.print("yOffset: ");
  Serial.println(yOffset);
  Serial.print("xScale: ");
  Serial.println(xScale, 4);
  Serial.print("yScale: ");
  Serial.println(yScale, 4);
}

void loop() {
  int16_t x, y, z;

  if (readQMC5883PData(x, y, z)) {
    head = calcularHead(x, y);

    Serial.print("X: ");
    Serial.print(x);
    Serial.print("\tY: ");
    Serial.print(y);
    Serial.print("\tZ: ");
    Serial.print(z);
    Serial.print("\tH: ");
    Serial.println(head);
  } else {
    Serial.println("Sem leitura valida da bussola.");
  }

  delay(50);
}