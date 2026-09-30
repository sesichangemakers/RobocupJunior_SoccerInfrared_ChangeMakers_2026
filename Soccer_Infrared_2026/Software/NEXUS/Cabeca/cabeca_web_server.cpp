#include "cabeca_web_server.hpp"

#include <WiFi.h>
#include <WebServer.h>

#include "web/pages/campo_page.hpp"
#include "web/pages/bussola_page.hpp"
#include "web/pages/mapa_page.hpp"
#include "web/pages/menu_page.hpp"
#include "web/pages/placeholder_page.hpp"

namespace {
WebServer server(80);
bool routeConfigured = false;
CabecaWebServer* gSelf = nullptr;

float normalizar360(float ang) {
  while (ang < 0.0f) ang += 360.0f;
  while (ang >= 360.0f) ang -= 360.0f;
  return ang;
}

float erroSignedGraus(float referencia, float atual) {
  float erro = referencia - atual;
  while (erro > 180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

bool parseTargetPair(const String& raw, float& xCm, float& yCm) {
  String texto = raw;
  texto.trim();
  int sep = texto.indexOf('/');
  if (sep <= 0) {
    return false;
  }

  String sx = texto.substring(0, sep);
  String sy = texto.substring(sep + 1);
  sx.trim();
  sy.trim();
  sx.replace(',', '.');
  sy.replace(',', '.');

  if (sx.length() == 0 || sy.length() == 0) {
    return false;
  }

  xCm = sx.toFloat();
  yCm = sy.toFloat();
  return isfinite(xCm) && isfinite(yCm);
}
}

void CabecaWebServer::setPositionTargetSender(bool (*sender)(float xCm, float yCm)) {
  positionTargetSender_ = sender;
}

bool CabecaWebServer::begin(const char* apSsid, const char* apPassword, uint16_t port) {
  if (port != 80) {
    Serial.println("[WEB] Aviso: porta diferente de 80 nao suportada nesta versao.");
  }

  WiFi.mode(WIFI_AP_STA);

  bool ok = WiFi.softAP(apSsid, apPassword);
  if (!ok) {
    Serial.println("[WEB] Falha ao iniciar Access Point.");
    running_ = false;
    return false;
  }

  if (!routeConfigured) {
    gSelf = this;

    server.on("/", HTTP_GET, []() {
      server.send(200, "text/html", WebPages::renderMenuPage());
    });

    server.on("/mapa", HTTP_GET, []() {
      server.send(200, "text/html", WebPages::renderMapaPage());
    });

    server.on("/campo", HTTP_GET, []() {
      server.send(200, "text/html", WebPages::renderCampoPage());
    });

    server.on("/bussola", HTTP_GET, []() {
      server.send(200, "text/html", WebPages::renderBussolaPage());
    });

    server.on("/extra", HTTP_GET, []() {
      server.send(200,
                  "text/html",
                  WebPages::renderPlaceholderPage(
                      "Opcao 4",
                      "Tela reservada para a proxima funcionalidade."));
    });

    server.on("/api/map32", HTTP_GET, []() {
      if (gSelf == nullptr) {
        server.send(500, "application/json", "{\"erro\":\"server\"}");
        return;
      }

      String json;
      json.reserve(900);

      unsigned long idade = 0;
      if (gSelf->mapUltimoRxMs_ > 0) {
        idade = millis() - gSelf->mapUltimoRxMs_;
      }

      json += "{\"seq\":";
      json += String(gSelf->mapSeq_);
      json += ",\"limiar\":";
      json += String(gSelf->mapLimiar_);
      json += ",\"valido\":";
      json += gSelf->mapValido_ ? "true" : "false";
      json += ",\"idade_ms\":";
      json += String(idade);
      json += ",\"sensores\":[";

      for (uint8_t i = 0; i < kSensorCount; i++) {
        if (i > 0) json += ",";
        json += String(gSelf->mapSensores_[i]);
      }

      json += "]}";
      server.send(200, "application/json", json);
    });

    server.on("/api/ultras", HTTP_GET, []() {
      if (gSelf == nullptr) {
        server.send(500, "application/json", "{\"erro\":\"server\"}");
        return;
      }

      unsigned long idade = 0;
      if (gSelf->ultraUltimoRxMs_ > 0) {
        idade = millis() - gSelf->ultraUltimoRxMs_;
      }

      bool valido = (gSelf->ultraUltimoRxMs_ > 0) && (idade < 800);

      String json;
      json.reserve(220);
      json += "{\"valido\":";
      json += valido ? "true" : "false";
      json += ",\"idade_ms\":";
      json += String(idade);
      json += ",\"uD_x10\":";
      json += String(gSelf->ultraDX10_);
      json += ",\"uE_x10\":";
      json += String(gSelf->ultraEX10_);
      json += ",\"uF_x10\":";
      json += String(gSelf->ultraFX10_);
      json += ",\"uT_x10\":";
      json += String(gSelf->ultraTX10_);
      json += "}";

      server.send(200, "application/json", json);
    });

    server.on("/api/bussola", HTTP_GET, []() {
      if (gSelf == nullptr) {
        server.send(500, "application/json", "{\"erro\":\"server\"}");
        return;
      }

      unsigned long idadeAtual = 0;
      if (gSelf->bussolaUltimoHeadingRxMs_ > 0) {
        idadeAtual = millis() - gSelf->bussolaUltimoHeadingRxMs_;
      }

      unsigned long idadeRef = 0;
      if (gSelf->bussolaUltimaRefRxMs_ > 0) {
        idadeRef = millis() - gSelf->bussolaUltimaRefRxMs_;
      }

      float atual = normalizar360(gSelf->bussolaAtualDeg_);
      float ref = normalizar360((float)gSelf->bussolaReferenciaDeg_);
      float erro = erroSignedGraus(ref, atual);

      String json;
      json.reserve(260);
      json += "{\"atual_deg\":";
      json += String(atual, 1);
      json += ",\"referencia_deg\":";
      json += String(ref, 1);
      json += ",\"erro_deg\":";
      json += String(erro, 1);
      json += ",\"atual_valido\":";
      json += gSelf->bussolaAtualValida_ ? "true" : "false";
      json += ",\"referencia_valida\":";
      json += gSelf->bussolaReferenciaValida_ ? "true" : "false";
      json += ",\"idade_atual_ms\":";
      json += String(idadeAtual);
      json += ",\"idade_ref_ms\":";
      json += String(idadeRef);
      json += "}";

      server.send(200, "application/json", json);
    });

    server.on("/api/status", HTTP_GET, []() {
      if (gSelf == nullptr) {
        server.send(500, "application/json", "{\"erro\":\"server\"}");
        return;
      }

      unsigned long idade = 0;
      if (gSelf->statusUpdatedMs_ > 0) {
        idade = millis() - gSelf->statusUpdatedMs_;
      }

      String json;
      json.reserve(220);
      json += "{\"papel\":\"";
      json += gSelf->statusAtacante_ ? "ATACANTE" : "DEFENSOR";
      json += "\",\"loop_fps\":";
      json += String(gSelf->statusLoopFps_, 1);
      json += ",\"wifi_dbm\":";
      json += String(gSelf->statusWifiRssiDbm_);
      json += ",\"wifi_ok\":";
      json += (gSelf->statusWifiRssiDbm_ > -120) ? "true" : "false";
      json += ",\"ap_clients\":";
      json += String(gSelf->statusApClients_);
      json += ",\"idade_ms\":";
      json += String(idade);
      json += "}";

      server.send(200, "application/json", json);
    });

    server.on("/api/posicionamento", HTTP_POST, []() {
      if (gSelf == nullptr) {
        server.send(500, "application/json", "{\"erro\":\"server\"}");
        return;
      }
      if (gSelf->positionTargetSender_ == nullptr) {
        server.send(503, "application/json", "{\"erro\":\"sender_indisponivel\"}");
        return;
      }

      String alvo = server.arg("alvo");
      float xCm = 0.0f;
      float yCm = 0.0f;
      if (!parseTargetPair(alvo, xCm, yCm)) {
        server.send(400, "application/json", "{\"erro\":\"formato_invalido\"}");
        return;
      }

      if (!gSelf->positionTargetSender_(xCm, yCm)) {
        server.send(409, "application/json", "{\"erro\":\"nao_encaminhado\"}");
        return;
      }

      String json;
      json.reserve(80);
      json += "{\"ok\":true,\"alvo\":\"";
      json += String(xCm, 1);
      json += "/";
      json += String(yCm, 1);
      json += "\"}";
      server.send(200, "application/json", json);
    });

    server.onNotFound([]() {
      server.send(404, "text/plain", "rota nao encontrada");
    });

    routeConfigured = true;
  }

  server.begin();
  running_ = true;

  Serial.print("[WEB] Servidor HTTP no ar em: http://");
  Serial.println(WiFi.softAPIP());
  return true;
}

void CabecaWebServer::handleClient() {
  if (!running_) {
    return;
  }
  server.handleClient();
}

bool CabecaWebServer::isRunning() const {
  return running_;
}

IPAddress CabecaWebServer::ipAddress() const {
  return WiFi.softAPIP();
}

void CabecaWebServer::updateMap32Snapshot(uint16_t seq,
                                          uint16_t limiar,
                                          const uint16_t* sensores,
                                          bool valido,
                                          unsigned long ultimoRxMs) {
  mapSeq_ = seq;
  mapLimiar_ = limiar;
  mapValido_ = valido;
  mapUltimoRxMs_ = ultimoRxMs;

  if (sensores == nullptr) {
    for (uint8_t i = 0; i < kSensorCount; i++) {
      mapSensores_[i] = 0;
    }
    return;
  }

  for (uint8_t i = 0; i < kSensorCount; i++) {
    mapSensores_[i] = sensores[i];
  }
}

void CabecaWebServer::updateUltrasSnapshot(int16_t ultraDX10,
                                           int16_t ultraEX10,
                                           int16_t ultraFX10,
                                           int16_t ultraTX10,
                                           unsigned long ultimoRxMs) {
  ultraDX10_ = ultraDX10;
  ultraEX10_ = ultraEX10;
  ultraFX10_ = ultraFX10;
  ultraTX10_ = ultraTX10;
  ultraUltimoRxMs_ = ultimoRxMs;
}

void CabecaWebServer::updateBussolaSnapshot(float headingAtualDeg,
                                            bool headingAtualValido,
                                            int16_t headingReferenciaDeg,
                                            bool headingReferenciaValida,
                                            unsigned long ultimoHeadingRxMs,
                                            unsigned long ultimoReferenciaRxMs) {
  bussolaAtualDeg_ = headingAtualDeg;
  bussolaAtualValida_ = headingAtualValido;
  bussolaReferenciaDeg_ = headingReferenciaDeg;
  bussolaReferenciaValida_ = headingReferenciaValida;
  bussolaUltimoHeadingRxMs_ = ultimoHeadingRxMs;
  bussolaUltimaRefRxMs_ = ultimoReferenciaRxMs;
}

void CabecaWebServer::updateRuntimeStatus(bool atacante,
                                          float loopFps,
                                          int32_t wifiRssiDbm,
                                          uint8_t apClients,
                                          unsigned long updatedMs) {
  statusAtacante_ = atacante;
  statusLoopFps_ = loopFps;
  statusWifiRssiDbm_ = wifiRssiDbm;
  statusApClients_ = apClients;
  statusUpdatedMs_ = updatedMs;
}
