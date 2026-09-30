#pragma once

#include <Arduino.h>
#include <IPAddress.h>

class CabecaWebServer {
 public:
  static const uint8_t kSensorCount = 32;

  bool begin(const char* apSsid, const char* apPassword, uint16_t port = 80);
  void handleClient();
  bool isRunning() const;
  IPAddress ipAddress() const;
  void setPositionTargetSender(bool (*sender)(float xCm, float yCm));
  void updateMap32Snapshot(uint16_t seq,
                           uint16_t limiar,
                           const uint16_t* sensores,
                           bool valido,
                           unsigned long ultimoRxMs);
  void updateUltrasSnapshot(int16_t ultraDX10,
                            int16_t ultraEX10,
                            int16_t ultraFX10,
                            int16_t ultraTX10,
                            unsigned long ultimoRxMs);
  void updateBussolaSnapshot(float headingAtualDeg,
                             bool headingAtualValido,
                             int16_t headingReferenciaDeg,
                             bool headingReferenciaValida,
                             unsigned long ultimoHeadingRxMs,
                             unsigned long ultimoReferenciaRxMs);
  void updateRuntimeStatus(bool atacante,
                           float loopFps,
                           int32_t wifiRssiDbm,
                           uint8_t apClients,
                           unsigned long updatedMs);

 private:
  bool running_ = false;
  uint16_t mapSeq_ = 0;
  uint16_t mapLimiar_ = 0;
  uint16_t mapSensores_[kSensorCount] = {0};
  bool mapValido_ = false;
  unsigned long mapUltimoRxMs_ = 0;

  int16_t ultraDX10_ = -10;
  int16_t ultraEX10_ = -10;
  int16_t ultraFX10_ = -10;
  int16_t ultraTX10_ = -10;
  unsigned long ultraUltimoRxMs_ = 0;

  float bussolaAtualDeg_ = 0.0f;
  bool bussolaAtualValida_ = false;
  int16_t bussolaReferenciaDeg_ = 0;
  bool bussolaReferenciaValida_ = false;
  unsigned long bussolaUltimoHeadingRxMs_ = 0;
  unsigned long bussolaUltimaRefRxMs_ = 0;

  bool statusAtacante_ = true;
  float statusLoopFps_ = 0.0f;
  int32_t statusWifiRssiDbm_ = -127;
  uint8_t statusApClients_ = 0;
  unsigned long statusUpdatedMs_ = 0;
  bool (*positionTargetSender_)(float xCm, float yCm) = nullptr;
};
