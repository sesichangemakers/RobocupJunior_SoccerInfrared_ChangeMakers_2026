#include "motores_movimentacao.hpp"
#include "posicao_campo.hpp"

namespace {
MotoresMovimentacaoConfig g_cfg;
bool g_inicializado = false;
uint8_t g_confirmacoesLinha = 0;

int g_v1Atual = 0;
int g_v2Atual = 0;
int g_v3Atual = 0;
int g_v4Atual = 0;

void motor1(int vel) {
  int pwm = constrain(abs(vel), 0, 255);
  ledcWrite(g_cfg.pwm_ch1, pwm);
  if (vel <= 0) {
    digitalWrite(g_cfg.in1_1_a, HIGH);
    digitalWrite(g_cfg.in2_1_a, LOW);
  } else {
    digitalWrite(g_cfg.in1_1_a, LOW);
    digitalWrite(g_cfg.in2_1_a, HIGH);
  }
}

void motor2(int vel) {
  int pwm = constrain(abs(vel), 0, 255);
  ledcWrite(g_cfg.pwm_ch2, pwm);
  if (vel <= 0) {
    digitalWrite(g_cfg.in1_2_a, HIGH);
    digitalWrite(g_cfg.in2_2_a, LOW);
  } else {
    digitalWrite(g_cfg.in1_2_a, LOW);
    digitalWrite(g_cfg.in2_2_a, HIGH);
  }
}

void motor4(int vel) {
  int pwm = constrain(abs(vel), 0, 255);
  ledcWrite(g_cfg.pwm_ch3, pwm);
  if (vel <= 0) {
    digitalWrite(g_cfg.in1_1_b, HIGH);
    digitalWrite(g_cfg.in2_1_b, LOW);
  } else {
    digitalWrite(g_cfg.in1_1_b, LOW);
    digitalWrite(g_cfg.in2_1_b, HIGH);
  }
}

void motor3(int vel) {
  int pwm = constrain(abs(vel), 0, 255);
  ledcWrite(g_cfg.pwm_ch4, pwm);
  if (vel <= 0) {
    digitalWrite(g_cfg.in1_2_b, HIGH);
    digitalWrite(g_cfg.in2_2_b, LOW);
  } else {
    digitalWrite(g_cfg.in1_2_b, LOW);
    digitalWrite(g_cfg.in2_2_b, HIGH);
  }
}

int aplicarRampaPwm(int alvo, int atual, int passoMaximo) {
  int delta = alvo - atual;
  if (delta > passoMaximo) return atual + passoMaximo;
  if (delta < -passoMaximo) return atual - passoMaximo;
  return alvo;
}

void aplicarComandoMotoresComRampa(int v1Alvo, int v2Alvo, int v3Alvo, int v4Alvo) {
  int alvo1 = constrain(v1Alvo, -255, 255);
  int alvo2 = constrain(v2Alvo, -255, 255);
  int alvo3 = constrain(v3Alvo, -255, 255);
  int alvo4 = constrain(v4Alvo, -255, 255);

  g_v1Atual = aplicarRampaPwm(alvo1, g_v1Atual, g_cfg.passoRampaPwm);
  g_v2Atual = aplicarRampaPwm(alvo2, g_v2Atual, g_cfg.passoRampaPwm);
  g_v3Atual = aplicarRampaPwm(alvo3, g_v3Atual, g_cfg.passoRampaPwm);
  g_v4Atual = aplicarRampaPwm(alvo4, g_v4Atual, g_cfg.passoRampaPwm);

  motor1(g_v1Atual);
  motor2(g_v2Atual);
  motor3(g_v3Atual);
  motor4(g_v4Atual);
}
}

void inicializarMotoresMovimentacao(const MotoresMovimentacaoConfig& config) {
  g_cfg = config;

  pinMode(g_cfg.in1_1_a, OUTPUT);
  pinMode(g_cfg.in2_1_a, OUTPUT);
  pinMode(g_cfg.in1_2_a, OUTPUT);
  pinMode(g_cfg.in2_2_a, OUTPUT);
  pinMode(g_cfg.in1_1_b, OUTPUT);
  pinMode(g_cfg.in2_1_b, OUTPUT);
  pinMode(g_cfg.in1_2_b, OUTPUT);
  pinMode(g_cfg.in2_2_b, OUTPUT);

  ledcSetup(g_cfg.pwm_ch1, g_cfg.pwm_freq, g_cfg.pwm_res);
  ledcAttachPin(g_cfg.pwm_1_a, g_cfg.pwm_ch1);
  ledcSetup(g_cfg.pwm_ch2, g_cfg.pwm_freq, g_cfg.pwm_res);
  ledcAttachPin(g_cfg.pwm_2_a, g_cfg.pwm_ch2);
  ledcSetup(g_cfg.pwm_ch3, g_cfg.pwm_freq, g_cfg.pwm_res);
  ledcAttachPin(g_cfg.pwm_1_b, g_cfg.pwm_ch3);
  ledcSetup(g_cfg.pwm_ch4, g_cfg.pwm_freq, g_cfg.pwm_res);
  ledcAttachPin(g_cfg.pwm_2_b, g_cfg.pwm_ch4);

  g_inicializado = true;
  pararMotores();
}

void pararMotores() {
  if (!g_inicializado) {
    return;
  }

  ledcWrite(g_cfg.pwm_ch1, 0);
  ledcWrite(g_cfg.pwm_ch2, 0);
  ledcWrite(g_cfg.pwm_ch3, 0);
  ledcWrite(g_cfg.pwm_ch4, 0);

  digitalWrite(g_cfg.in1_1_a, LOW);
  digitalWrite(g_cfg.in2_1_a, LOW);
  digitalWrite(g_cfg.in1_2_a, LOW);
  digitalWrite(g_cfg.in2_2_a, LOW);
  digitalWrite(g_cfg.in1_1_b, LOW);
  digitalWrite(g_cfg.in2_1_b, LOW);
  digitalWrite(g_cfg.in1_2_b, LOW);
  digitalWrite(g_cfg.in2_2_b, LOW);

  g_v1Atual = 0;
  g_v2Atual = 0;
  g_v3Atual = 0;
  g_v4Atual = 0;
}

void girarNoEixo(int velocidade) {
  if (!g_inicializado) {
    return;
  }

  int vel = constrain(velocidade, -g_cfg.velocidadeMaxima, g_cfg.velocidadeMaxima);
  aplicarComandoMotoresComRampa(-vel, -vel, -vel, -vel);
}

void seguirDirecaoPorAngulo(float anguloGraus, int velocidade) {
  if (!g_inicializado) {
    return;
  }

  int velocidadeAlvo = constrain(velocidade, 0, g_cfg.velocidadeMaxima);

  float theta = anguloGraus * PI / 180.0f;
  float vx = velocidadeAlvo * sinf(theta);
  float vy = velocidadeAlvo * cosf(theta);

  float theta1 = 45.0f * PI / 180.0f;
  float theta2 = 135.0f * PI / 180.0f;
  float theta3 = 225.0f * PI / 180.0f;
  float theta4 = 315.0f * PI / 180.0f;

  float v1 = vx * cosf(theta1) + vy * sinf(theta1);
  float v2 = vx * cosf(theta2) + vy * sinf(theta2);
  float v3 = vx * cosf(theta3) + vy * sinf(theta3);
  float v4 = vx * cosf(theta4) + vy * sinf(theta4);

  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
  if (maxVel > g_cfg.velocidadeMaxima) {
    float escala = (float)g_cfg.velocidadeMaxima / maxVel;
    v1 *= escala;
    v2 *= escala;
    v3 *= escala;
    v4 *= escala;
  }

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}

// --------- Função de movimento do atacante:

void seguirDirecaoComGiro(float anguloGraus, int velocidade, int cmdGiro) {
  if (!g_inicializado) {
    return;
  }

  int velocidadeAlvo = constrain(velocidade, 0, g_cfg.velocidadeMaxima);
  int cmdGiroLimitado = constrain(cmdGiro, -255, 255);
  float prioridadeGiro = (float)abs(cmdGiroLimitado) / 255.0f;
  float prioridadeTranslacao = 1.0f - prioridadeGiro;
  int sinalGiro = (cmdGiroLimitado > 0) ? 1 : ((cmdGiroLimitado < 0) ? -1 : 0);

  float theta = anguloGraus * PI / 180.0f;
  float vx = ((float)velocidadeAlvo * prioridadeTranslacao) * sinf(theta);
  float vy = ((float)velocidadeAlvo * prioridadeTranslacao) * cosf(theta);

  float theta1 = 45.0f * PI / 180.0f;
  float theta2 = 135.0f * PI / 180.0f;
  float theta3 = 225.0f * PI / 180.0f;
  float theta4 = 315.0f * PI / 180.0f;

  float v1 = vx * cosf(theta1) + vy * sinf(theta1);
  float v2 = vx * cosf(theta2) + vy * sinf(theta2);
  float v3 = vx * cosf(theta3) + vy * sinf(theta3);
  float v4 = vx * cosf(theta4) + vy * sinf(theta4);

  float termoGiro = (-g_cfg.ganhoGiroMisto * (float)velocidadeAlvo * prioridadeGiro * (float)sinalGiro) + 10;
  
  
  
  if((anguloGraus > 0) && (anguloGraus < 180)){
  v1 += termoGiro - 50; // 315
  v2 += termoGiro; // 225
  v3 += termoGiro; // 135
  v4 += termoGiro - 50; //45
  }else{
  v1 += termoGiro + 50; // 315
  v2 += termoGiro; // 225
  v3 += termoGiro; // 135
  v4 += termoGiro + 50; //45
  }


  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
  if (maxVel > g_cfg.velocidadeMaxima) {
    float escala = (float)g_cfg.velocidadeMaxima / maxVel;
    v1 *= escala;
    v2 *= escala;
    v3 *= escala;
    v4 *= escala;
  }

 // corrige certo para teste
 // aplicarComandoMotoresComRampa((int)termoGiro, (int)termoGiro, (int)termoGiro, (int)termoGiro);
    aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}

// --------- Função de movimento do defensor:

void seguirDirecaoComGiroLaterais(float anguloGraus, int velocidade, int cmdGiro) {
  if (!g_inicializado) {
    return;
  }

  int velocidadeAlvo = constrain(velocidade, 0, g_cfg.velocidadeMaxima);
  int cmdGiroLimitado = constrain(cmdGiro, -255, 255);
  float prioridadeGiro = (float)abs(cmdGiroLimitado) / 255.0f;
  float prioridadeTranslacao = 1.0f - prioridadeGiro;
  int sinalGiro = (cmdGiroLimitado > 0) ? 1 : ((cmdGiroLimitado < 0) ? -1 : 0);

  float theta = anguloGraus * PI / 180.0f;
  float vx = ((float)velocidadeAlvo * prioridadeTranslacao) * sinf(theta);
  float vy = ((float)velocidadeAlvo * prioridadeTranslacao) * cosf(theta);

  float theta1 = 45.0f * PI / 180.0f;
  float theta2 = 135.0f * PI / 180.0f;
  float theta3 = 225.0f * PI / 180.0f;
  float theta4 = 315.0f * PI / 180.0f;

  float v1 = vx * cosf(theta1) + vy * sinf(theta1);
  float v2 = vx * cosf(theta2) + vy * sinf(theta2);
  float v3 = vx * cosf(theta3) + vy * sinf(theta3);
  float v4 = vx * cosf(theta4) + vy * sinf(theta4);

  float termoGiro = (-g_cfg.ganhoGiroMisto * (float)velocidadeAlvo * prioridadeGiro * (float)sinalGiro) + 10;
  
  

  if((anguloGraus > 0) && (anguloGraus < 180)){
  v1 += termoGiro; // 315
  v2 += termoGiro + 10; // 225
  v3 += termoGiro + 10; // 135
  v4 += termoGiro; //45
  }else{
  v1 += termoGiro + 35; // 315
  v2 += termoGiro - 70; // 225
  v3 += termoGiro - 70; // 135
  v4 += termoGiro + 35; //45
  }

  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
  if (maxVel > g_cfg.velocidadeMaxima) {
    float escala = (float)g_cfg.velocidadeMaxima / maxVel;
    v1 *= escala;
    v2 *= escala;
    v3 *= escala;
    v4 *= escala;
  }

 // corrige certo para teste
 // aplicarComandoMotoresComRampa((int)termoGiro, (int)termoGiro, (int)termoGiro, (int)termoGiro);
    aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}















void moverFrenteComGiro(int velocidadePwm, int cmdGiro) {
  moverFrenteComGiro(velocidadePwm, cmdGiro, g_cfg.ganhoGiroMisto);
}

void moverFrenteComGiro(int velocidadePwm, int cmdGiro, float ganhoGiroMisto) {
  if (!g_inicializado) {
    return;
  }

  int pwmBase = constrain(velocidadePwm, 0, 255);

  float v1 = (float)pwmBase;
  float v2 = (float)pwmBase;
  float v3 = -(float)pwmBase;
  float v4 = -(float)pwmBase;

  float termoGiro = -ganhoGiroMisto * (float)cmdGiro;
  v1 += termoGiro;
  v2 += termoGiro;
  v3 += termoGiro;
  v4 += termoGiro;

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}

bool g_fugindoDaLinha = false;
float g_anguloFugaTravado = 0.0f;
unsigned long g_ultimaLinhaDetectadaMs = 0;

const unsigned long TEMPO_PERDA_LINHA_MS = 100;


bool sairDaLinha(bool linhaDetectada, float anguloLinhaGraus, int velocidadePwm,
                 float* anguloComandoSaida) {

  if (!g_inicializado) {
    return false;
  }

  const bool linhaAtiva = linhaDetectada && (anguloLinhaGraus >= 0.0f);

  // IMPORTANTE:
  // No modo ATACANTE, a placa Pe ja envia o angulo de REPULSAO da linha.
  // Portanto, este modulo NAO deve somar 180 graus nem inverter o vetor.
  // O angulo recebido e usado diretamente como direcao de fuga.
  if (linhaAtiva) {
    float anguloRepulsao = anguloLinhaGraus;

    while (anguloRepulsao >= 360.0f) anguloRepulsao -= 360.0f;
    while (anguloRepulsao < 0.0f)    anguloRepulsao += 360.0f;

    // Atualiza a direcao enquanto a linha estiver sendo lida.
    // Quando a leitura desaparecer, esta ultima direcao sera mantida
    // por TEMPO_PERDA_LINHA_MS para evitar uma quebra instantanea da fuga.
    g_anguloFugaTravado = anguloRepulsao;
    g_fugindoDaLinha = true;
    g_ultimaLinhaDetectadaMs = millis();

    if (anguloComandoSaida != nullptr) {
      *anguloComandoSaida = g_anguloFugaTravado;
    }

    seguirDirecaoPorAngulo(g_anguloFugaTravado, velocidadePwm);
    return true;
  }

  // Pequena retencao apos perder a linha: mantem o ultimo vetor seguro
  // por 100 ms (TEMPO_PERDA_LINHA_MS) antes de devolver o controle.
  if (g_fugindoDaLinha) {
    const unsigned long tempoSemLinha = millis() - g_ultimaLinhaDetectadaMs;

    if (tempoSemLinha < TEMPO_PERDA_LINHA_MS) {
      if (anguloComandoSaida != nullptr) {
        *anguloComandoSaida = g_anguloFugaTravado;
      }

      seguirDirecaoPorAngulo(g_anguloFugaTravado, velocidadePwm);
      return true;
    }
  }

  // Fim real da fuga.
  g_fugindoDaLinha = false;
  g_anguloFugaTravado = 0.0f;
  g_ultimaLinhaDetectadaMs = 0;

  if (anguloComandoSaida != nullptr) {
    *anguloComandoSaida = 0.0f;
  }

  return false;
}

void resetSairDaLinha() {
  g_confirmacoesLinha = 0;
  g_fugindoDaLinha = false;
  g_anguloFugaTravado = 0.0f;
  g_ultimaLinhaDetectadaMs = 0;
}

namespace {
MotoresPosicionamentoConfig g_posCfg;
MotoresPosicionamentoOrientacaoHooks g_posHooks;

float g_anguloAlvoGraus = 0.0f;

float normalizarAngulo360Pos(float ang) {
  while (ang >= 360.0f) ang -= 360.0f;
  while (ang < 0.0f) ang += 360.0f;
  return ang;
}

float mapearFaixaClampedPos(float valor,
                            float entradaMin,
                            float entradaMax,
                            float saidaMin,
                            float saidaMax) {
  float denominador = entradaMax - entradaMin;
  if (fabsf(denominador) < 0.0001f) {
    return saidaMin;
  }
  float proporcao = (valor - entradaMin) / denominador;
  if (proporcao < 0.0f) proporcao = 0.0f;
  if (proporcao > 1.0f) proporcao = 1.0f;
  return saidaMin + ((saidaMax - saidaMin) * proporcao);
}

int calcularVelocidadePosicionamento(float distanciaCm) {
  float velMapeada = mapearFaixaClampedPos(distanciaCm,
                                           0.0f,
                                           g_posCfg.distanciaRampaCm,
                                           (float)g_posCfg.velocidadePwmMin,
                                           (float)g_posCfg.velocidadePwmMax);
  return constrain((int)roundf(velMapeada), g_posCfg.velocidadePwmMin, g_posCfg.velocidadePwmMax);
}

bool usarFaixaNormalMovimento(float anguloMov) {
  return (anguloMov >= 315.0f || anguloMov <= 45.0f ||
          (anguloMov >= 135.0f && anguloMov <= 225.0f));
}

bool hooksOrientacaoValidos() {
  return g_posHooks.temReferenciaOrientacao != nullptr &&
         g_posHooks.obterErroOrientacaoGraus != nullptr &&
         g_posHooks.calcularComandoGiro != nullptr;
}

bool preencherVetorParaAlvo(float alvoX,
                            float alvoY,
                            float& erroX,
                            float& erroY,
                            float& distanciaCm,
                            float& anguloGraus) {
  if (!atualizarPosicaoAtual()) {
    return false;
  }

  const float atualX = obterPosicaoX();
  const float atualY = obterPosicaoY();
  erroX = alvoX - atualX;
  erroY = alvoY - atualY;
  distanciaCm = sqrtf((erroX * erroX) + (erroY * erroY));
  anguloGraus = normalizarAngulo360Pos(atan2f(erroX, -erroY) * 180.0f / PI);
  g_anguloAlvoGraus = anguloGraus;
  return true;
}
}

void inicializarMotoresPosicionamento(const MotoresPosicionamentoConfig& config) {
  g_posCfg = config;
  g_posHooks.temReferenciaOrientacao = config.hookTemReferenciaOrientacao;
  g_posHooks.obterErroOrientacaoGraus = config.hookObterErroOrientacaoGraus;
  g_posHooks.calcularComandoGiro = config.hookCalcularComandoGiro;
  g_anguloAlvoGraus = 0.0f;

  PosicaoCampo::Config posicaoCfg;
  posicaoCfg.campoLarguraCm = config.campoLarguraCm;
  posicaoCfg.campoAlturaCm = config.campoAlturaCm;
  posicaoCfg.roboRaioCm = config.roboRaioCm;
  posicaoCfg.compToleranciaCm = config.compToleranciaCm;
  posicaoCfg.filtroTauRapido = config.filtroTauRapido;
  posicaoCfg.filtroTauLento = config.filtroTauLento;
  posicaoCfg.saltoMaximoCm = config.saltoMaximoCm;
  PosicaoCampo::iniciar(posicaoCfg);
}

void configurarHooksOrientacaoPosicionamento(const MotoresPosicionamentoOrientacaoHooks& hooks) {
  g_posHooks = hooks;
}

void atualizarLeiturasPosicionamento(float ultraEsquerdaCm,
                                     float ultraDireitaCm,
                                     float ultraFrenteCm,
                                     float ultraTrasCm,
                                     bool leiturasValidas) {
  PosicaoCampo::atualizarLeituras(ultraEsquerdaCm, ultraDireitaCm, ultraFrenteCm, ultraTrasCm, leiturasValidas);
}

bool atualizarPosicaoAtual() {
  return PosicaoCampo::atualizar();
}

float obterPosicaoX() {
  return PosicaoCampo::obterX();
}

float obterPosicaoY() {
  return PosicaoCampo::obterY();
}

float obterConfiancaPosicao() {
  return PosicaoCampo::obterConfianca();
}

float obterAnguloAlvoPosicionamentoGraus() {
  return g_anguloAlvoGraus;
}

bool moverParaSemGiro(float xCm, float yCm) {
  float erroX = 0.0f;
  float erroY = 0.0f;
  float distanciaCm = 0.0f;
  float anguloMov = 0.0f;
  if (!preencherVetorParaAlvo(xCm, yCm, erroX, erroY, distanciaCm, anguloMov)) {
    return false;
  }

  int velocidade = calcularVelocidadePosicionamento(distanciaCm);
  seguirDirecaoPorAngulo(anguloMov, velocidade);
  return true;
}

bool moverParaComGiro(float xCm, float yCm) {
  float erroX = 0.0f;
  float erroY = 0.0f;
  float distanciaCm = 0.0f;
  float anguloMov = 0.0f;
  if (!preencherVetorParaAlvo(xCm, yCm, erroX, erroY, distanciaCm, anguloMov)) {
    return false;
  }

  //int velocidade = calcularVelocidadePosicionamento(distanciaCm);
  int velocidade = 220;
  if (!hooksOrientacaoValidos() || !g_posHooks.temReferenciaOrientacao()) {
    seguirDirecaoPorAngulo(anguloMov, velocidade);
    return true;
  }

  float erroOrientacao = g_posHooks.obterErroOrientacaoGraus();
  int cmdGiro = g_posCfg.sinalGiro * g_posHooks.calcularComandoGiro(erroOrientacao);

  if (fabsf(erroOrientacao) > g_posCfg.giroSomenteEixoGraus) {
    int cmdEixo = constrain(cmdGiro, -g_posCfg.velocidadeGiroEixoPwm, g_posCfg.velocidadeGiroEixoPwm);
    girarNoEixo(cmdEixo);
    return true;
  }

  bool habilitarGiro = fabsf(erroOrientacao) > g_posCfg.erroMinimoAtivarGiroGraus;
  if (!habilitarGiro) {
    seguirDirecaoPorAngulo(anguloMov, velocidade);
    return true;
  }

  if (usarFaixaNormalMovimento(anguloMov)) {
    seguirDirecaoComGiro(anguloMov, velocidade, cmdGiro);
  } else {
    seguirDirecaoComGiroLaterais(anguloMov, velocidade, cmdGiro);
  }
  return true;
}