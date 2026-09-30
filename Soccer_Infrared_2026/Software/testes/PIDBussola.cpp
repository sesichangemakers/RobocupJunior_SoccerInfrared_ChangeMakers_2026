// Calibração da bussola QMC5883P na placa Cabeca.
// Funcao: iniciar sensor, calibrar min/max XY e entregar os valores base de calibração para aplicar no teste.
// Saida: leituras RAW e heading em graus no monitor serial.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

const uint8_t QMC5883P_ADDR = 0x2C;

// Calibração dinâmica
int16_t xMin = 32767;
int16_t xMax = -32768;
int16_t yMin = 32767;
int16_t yMax = -32768;

// Offsets e escalas
float xOffset = 0.0;
float yOffset = 0.0;
float xScale  = 1.0;
float yScale  = 1.0;

int head = 0;

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, (uint8_t)1);

  if (Wire.available()) return Wire.read();
  return 0;
}

void initQMC5883P() {
  Wire.begin();
  delay(20);

  // Configuração usada para QMC5883P
  writeReg(0x29, 0x06);
  writeReg(0x0B, 0x08);
  writeReg(0x0A, 0xC3);

  delay(20);
}

bool readQMC5883PData(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t status = readReg(0x09);

  if ((status & 0x01) == 0) {
    return false;
  }

  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(0x01);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, (uint8_t)6);

  if (Wire.available() == 6) {
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

  return false;
}

void atualizarCalibracao(int16_t xRaw, int16_t yRaw) {
  if (xRaw < xMin) xMin = xRaw;
  if (xRaw > xMax) xMax = xRaw;
  if (yRaw < yMin) yMin = yRaw;
  if (yRaw > yMax) yMax = yRaw;

  xOffset = (xMax + xMin) / 2.0;
  yOffset = (yMax + yMin) / 2.0;

  float xRange = (xMax - xMin) / 2.0;
  float yRange = (yMax - yMin) / 2.0;
  float mediaRange = (xRange + yRange) / 2.0;

  if (xRange > 0.0) xScale = mediaRange / xRange;
  if (yRange > 0.0) yScale = mediaRange / yRange;
}

int calcularHead(int16_t xRaw, int16_t yRaw) {
  float xCorr = (xRaw - xOffset) * xScale;
  float yCorr = (yRaw - yOffset) * yScale;

  float ang = atan2(yCorr, xCorr) * 180.0 / PI;

  if (ang < 0) ang += 360.0;

  return (int)ang;
}

void imprimirCalibracao() {
  Serial.println("======== CALIBRACAO ========");
  Serial.print("xMin = "); Serial.println(xMin);
  Serial.print("xMax = "); Serial.println(xMax);
  Serial.print("yMin = "); Serial.println(yMin);
  Serial.print("yMax = "); Serial.println(yMax);
  Serial.print("xOffset = "); Serial.println(xOffset);
  Serial.print("yOffset = "); Serial.println(yOffset);
  Serial.print("xScale = "); Serial.println(xScale, 6);
  Serial.print("yScale = "); Serial.println(yScale, 6);
  Serial.println("============================");
}

void setup() {
  Serial.begin(115200);
  initQMC5883P();

  delay(200);

  Serial.println();
  Serial.println("=== CALIBRACAO COMPLETA QMC5883P ===");
  Serial.print("CHIP ID: 0x");
  Serial.println(readReg(0x00), HEX);
  Serial.println("Gire o sensor lentamente em 360 graus varias vezes.");
  Serial.println("Mantenha o sensor o mais plano possivel.");
  Serial.println("Afaste de motores, imas, bateria e fios de potencia.");
  Serial.println();
}

void loop() {
  int16_t x, y, z;

  if (readQMC5883PData(x, y, z)) {
    atualizarCalibracao(x, y);
    head = calcularHead(x, y);

    Serial.print("X: ");
    Serial.print(x);
    Serial.print("\tY: ");
    Serial.print(y);
    Serial.print("\tZ: ");
    Serial.print(z);

    Serial.print("\tH: ");
    Serial.print(head);

    Serial.print("\txMin: ");
    Serial.print(xMin);
    Serial.print("\txMax: ");
    Serial.print(xMax);
    Serial.print("\tyMin: ");
    Serial.print(yMin);
    Serial.print("\tyMax: ");
    Serial.println(yMax);
  }

  static unsigned long ultimoPrint = 0;
  if (millis() - ultimoPrint > 3000) {
    ultimoPrint = millis();
    imprimirCalibracao();
  }

  delay(50);
}