#include "posicao_campo.hpp"

namespace PosicaoCampo {

namespace {

Config g_cfg;

float g_ultraEcm = -1.0f;
float g_ultraDcm = -1.0f;
float g_ultraFcm = -1.0f;
float g_ultraTcm = -1.0f;
bool g_ultrasValidos = false;

float g_posicaoXcm = 91.0f;
float g_posicaoYcm = 121.5f;
float g_confianca = 0.0f;
unsigned long g_ultimaEstimativaMs = 0;
bool g_posicaoValida = false;

bool ultraValido(float v) {
  return isfinite(v) && v > 1.0f && v < 350.0f;
}

float filtroComplementar(float anterior, float medicao, float confianca, float dtSec) {
  float tau = (confianca >= 0.8f) ? g_cfg.filtroTauRapido : g_cfg.filtroTauLento;
  if (tau < 0.02f) tau = 0.02f;
  float alpha = expf(-dtSec / tau);
  alpha = constrain(alpha + ((1.0f - confianca) * 0.18f), 0.08f, 0.96f);
  return (alpha * anterior) + ((1.0f - alpha) * medicao);
}

struct EixoResultado {
  float valor;
  float confianca;
};

EixoResultado estimarEixo(float leituraA,
                          float leituraB,
                          float campo,
                          float ultimo,
                          float dtSec) {
  const bool temA = ultraValido(leituraA);
  const bool temB = ultraValido(leituraB);
  const float minimo = g_cfg.roboRaioCm;
  const float maximo = campo - g_cfg.roboRaioCm;
  const float utilizavel = campo - (2.0f * g_cfg.roboRaioCm);

  float medida = ultimo;
  float confianca = 0.05f;

  if (temA && temB) {
    float diretoA = constrain(leituraA + g_cfg.roboRaioCm, minimo, maximo);
    float diretoB = constrain(campo - (leituraB + g_cfg.roboRaioCm), minimo, maximo);
    float soma = leituraA + leituraB;
    if (soma < 1.0f) soma = 1.0f;
    float residual = fabsf((leituraA + leituraB + (2.0f * g_cfg.roboRaioCm)) - campo);

    if (residual <= g_cfg.compToleranciaCm) {
      medida = 0.5f * (diretoA + diretoB);
      confianca = 0.95f;
    } else {
      float fracA = constrain(leituraA / soma, 0.0f, 1.0f);
      float fracB = constrain(leituraB / soma, 0.0f, 1.0f);
      float mapaA = minimo + (fracA * utilizavel);
      float mapaB = minimo + ((1.0f - fracB) * utilizavel);
      medida = 0.5f * (mapaA + mapaB);
      confianca = 0.56f;
    }
  } else if (temA) {
    medida = constrain(leituraA + g_cfg.roboRaioCm, minimo, maximo);
    confianca = 0.62f;
  } else if (temB) {
    medida = constrain(campo - (leituraB + g_cfg.roboRaioCm), minimo, maximo);
    confianca = 0.62f;
  }

  float salto = medida - ultimo;
  float medidaLimitada = ultimo + constrain(salto, -g_cfg.saltoMaximoCm, g_cfg.saltoMaximoCm);
  float filtrada = filtroComplementar(ultimo, medidaLimitada, confianca, dtSec);

  EixoResultado out;
  out.valor = constrain(filtrada, minimo, maximo);
  out.confianca = confianca;
  return out;
}

}  // namespace

void iniciar(const Config& config) {
  g_cfg = config;
  g_posicaoXcm = 0.5f * g_cfg.campoLarguraCm;
  g_posicaoYcm = 0.5f * g_cfg.campoAlturaCm;
  g_confianca = 0.0f;
  g_ultimaEstimativaMs = 0;
  g_posicaoValida = false;
}

void atualizarLeituras(float ultraEsquerdaCm,
                       float ultraDireitaCm,
                       float ultraFrenteCm,
                       float ultraTrasCm,
                       bool leiturasValidas) {
  g_ultraEcm = ultraEsquerdaCm;
  g_ultraDcm = ultraDireitaCm;
  g_ultraFcm = ultraFrenteCm;
  g_ultraTcm = ultraTrasCm;
  g_ultrasValidos = leiturasValidas;
  g_posicaoValida = false;
}

bool atualizar() {
  if (!g_ultrasValidos) {
    g_posicaoValida = false;
    return false;
  }

  unsigned long agora = millis();
  float dtSec = 0.08f;
  if (g_ultimaEstimativaMs > 0 && agora > g_ultimaEstimativaMs) {
    dtSec = (agora - g_ultimaEstimativaMs) / 1000.0f;
    if (dtSec < 0.05f) dtSec = 0.05f;
    if (dtSec > 0.8f) dtSec = 0.8f;
  }
  g_ultimaEstimativaMs = agora;

  EixoResultado eixoX = estimarEixo(g_ultraEcm, g_ultraDcm, g_cfg.campoLarguraCm, g_posicaoXcm, dtSec);
  EixoResultado eixoY = estimarEixo(g_ultraFcm, g_ultraTcm, g_cfg.campoAlturaCm, g_posicaoYcm, dtSec);

  g_posicaoXcm = eixoX.valor;
  g_posicaoYcm = eixoY.valor;
  g_confianca = 0.5f * (eixoX.confianca + eixoY.confianca);
  g_posicaoValida = (eixoX.confianca > 0.1f) && (eixoY.confianca > 0.1f);
  return g_posicaoValida;
}

float obterX() {
  return g_posicaoXcm;
}

float obterY() {
  return g_posicaoYcm;
}

float obterConfianca() {
  return g_confianca;
}

}  // namespace PosicaoCampo
