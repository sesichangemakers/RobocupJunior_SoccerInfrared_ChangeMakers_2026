#pragma once

#include <Arduino.h>

/*
 Biblioteca: motores_movimentacao

 Guia rapido de uso (API de movimentacao):
 - inicializarMotoresMovimentacao(config):
   Inicializa pinos da ponte H, canais PWM e parametros globais da locomocao.
   Parametros principais em config:
   - velocidadeMaxima: limite global de velocidade das rodas (0..255).
   - passoRampaPwm: passo maximo por ciclo na rampa de aceleracao/frenagem.
   - ganhoGiroMisto: ganho de mistura entre translacao e giro (cmdGiro).

 - pararMotores():
   Zera PWM e coloca as pontes em estado neutro.

 - girarNoEixo(velocidade):
   Gira o robo no proprio eixo.
   Parametros:
   - velocidade: comando assinado de giro. Sinal define o sentido.

 - seguirDirecaoPorAngulo(anguloGraus, velocidade):
   Realiza translacao no angulo desejado (base mecanum/omni).
   Parametros:
   - anguloGraus: direcao de movimento no referencial local (0..360).
   - velocidade: modulo da translacao (0..velocidadeMaxima).

 - seguirDirecaoComGiro(anguloGraus, velocidade, cmdGiro):
   Translacao com correcao de orientacao durante o deslocamento.
   Parametros:
   - anguloGraus: direcao de translacao (0..360).
   - velocidade: modulo da translacao (0..velocidadeMaxima).
   - cmdGiro: prioridade/sentido de giro. Modulo define prioridade
     (0 = so translacao, 255 = so giro) e sinal define o sentido.

 - seguirDirecaoComGiroLaterais(anguloGraus, velocidade, cmdGiro):
   Variante para deslocamentos laterais, com distribuicao de giro otimizada.
   Parametros:
   - anguloGraus: direcao de translacao (0..360).
   - velocidade: modulo da translacao (0..velocidadeMaxima).
   - cmdGiro: prioridade/sentido de giro. Modulo define prioridade
     (0 = so translacao, 255 = so giro) e sinal define o sentido.

 - moverFrenteComGiro(velocidadePwm, cmdGiro):
   Comando direto de avancar com compensacao de giro.
   Parametros:
   - velocidadePwm: PWM base de avancar (0..255).
   - cmdGiro: comando de giro assinado para correcao angular.

 - moverFrenteComGiro(velocidadePwm, cmdGiro, ganhoGiroMisto):
   Mesmo comando acima, mas com ganho de giro por chamada.
   Parametros:
   - ganhoGiroMisto: fator de mistura do giro para esta chamada.

 - inicializarMotoresPosicionamento(config):
   Inicializa o modulo de posicionamento por coordenadas (estimativa e navegacao).
   Parametros principais em config:
   - campoLarguraCm/campoAlturaCm: dimensoes do campo em cm.
   - roboRaioCm: raio do robo para compensacao nas leituras dos ultras.
   - velocidadePwmMin/velocidadePwmMax: faixa de velocidade para deslocamento ao alvo.
   - hookTemReferenciaOrientacao/hookObterErroOrientacaoGraus/hookCalcularComandoGiro:
     hooks opcionais de orientacao usados no modo com giro.

 - atualizarLeiturasPosicionamento(ultraE, ultraD, ultraF, ultraT, validas):
   Atualiza as leituras ultrassonicas da iteracao atual.

 - atualizarPosicaoAtual():
   Recalcula a posicao filtrada do robo no campo com base nos ultras.
   Retorno: true se a posicao atual ficou valida.

 - obterPosicaoX() / obterPosicaoY():
   Retornam as coordenadas atuais estimadas do robo (em cm).

 - moverParaComGiro(x, y):
   Move ate a coordenada alvo com controle de orientacao.
   Comportamento: pode girar no proprio eixo quando erro angular e alto.

 - moverParaSemGiro(x, y):
   Move vetorialmente ate a coordenada alvo mantendo a orientacao atual.

//==========================================================================//
    float anguloFuga = 0.0f;
    if (sairDaLinha(linhaDetectada, anguloLinhaPe,
            aplicarFreioUltrassonicoAtacante(VELOCIDADE_FUGA_LINHA),
            &anguloFuga)) {
    fugindoLinhaAgora = true;
    anguloFugaLinhaCmd = anguloFuga;
    return;
    }
//==========================================================================//
*/

struct MotoresMovimentacaoConfig {
  uint8_t in1_1_a = 5;
  uint8_t in2_1_a = 6;
  uint8_t pwm_1_a = 4;

  uint8_t in1_2_a = 3;
  uint8_t in2_2_a = 46;
  uint8_t pwm_2_a = 7;

  uint8_t in1_1_b = 11;
  uint8_t in2_1_b = 12;
  uint8_t pwm_1_b = 10;

  uint8_t in1_2_b = 13;
  uint8_t in2_2_b = 14;
  uint8_t pwm_2_b = 47;

  uint8_t pwm_ch1 = 0;
  uint8_t pwm_ch2 = 1;
  uint8_t pwm_ch3 = 2;
  uint8_t pwm_ch4 = 3;

  uint32_t pwm_freq = 20000;
  uint8_t pwm_res = 8;

  int velocidadeMaxima = 255;
  int passoRampaPwm = 16;
  float ganhoGiroMisto = 0.7f;
};

struct MotoresPosicionamentoConfig {
  // Dimensoes do campo e raio do robo usados para converter ultras -> posicao X/Y.
  float campoLarguraCm = 182.0f;
  float campoAlturaCm = 243.0f;
  float roboRaioCm = 10.5f;

  // Parametros da estimativa: tolerancia de consistencia e filtro temporal.
  float compToleranciaCm = 22.0f;
  float filtroTauRapido = 0.26f;
  float filtroTauLento = 0.48f;
  float saltoMaximoCm = 35.0f;

  // Faixa de PWM usada durante o deslocamento ate o alvo.
  int velocidadePwmMax = 255;
  int velocidadePwmMin = 200;
  float distanciaRampaCm = 80.0f;

  // Regras de orientacao para o modo com giro.
  float toleranciaGiroGraus = 6.0f;
  float giroSomenteEixoGraus = 35.0f;
  int velocidadeGiroEixoPwm = 180;
  int sinalGiro = -1;
  float erroMinimoAtivarGiroGraus = 5.0f;

  // Hooks opcionais para orientacao no modo com giro.
  bool (*hookTemReferenciaOrientacao)() = nullptr;
  float (*hookObterErroOrientacaoGraus)() = nullptr;
  int (*hookCalcularComandoGiro)(float erroGraus) = nullptr;
};

// Hooks opcionais para reaproveitar qualquer fonte de orientacao (ex.: bussola + PID).
// - temReferenciaOrientacao(): informa se existe referencia angular valida.
// - obterErroOrientacaoGraus(): retorna o erro angular atual em graus.
// - calcularComandoGiro(erro): converte erro angular em comando de giro (tipicamente PID).
// Quando os hooks nao sao configurados, moverParaComGiro() cai automaticamente
// para translacao sem giro para manter o robo operacional.
struct MotoresPosicionamentoOrientacaoHooks {
  bool (*temReferenciaOrientacao)() = nullptr;
  float (*obterErroOrientacaoGraus)() = nullptr;
  int (*calcularComandoGiro)(float erroGraus) = nullptr;
};

void inicializarMotoresMovimentacao(const MotoresMovimentacaoConfig& config);
void pararMotores();
void girarNoEixo(int velocidade);
void seguirDirecaoPorAngulo(float anguloGraus, int velocidade);
void seguirDirecaoComGiro(float anguloGraus, int velocidade, int cmdGiro);
void seguirDirecaoComGiroLaterais(float anguloGraus, int velocidade, int cmdGiro);
void seguirDirecaoComGiroLateraisDefensor(float anguloGraus, int velocidade, int cmdGiro);
void moverFrenteComGiro(int velocidadePwm, int cmdGiro);
void moverFrenteComGiro(int velocidadePwm, int cmdGiro, float ganhoGiroMisto);

// Inicializa o modulo de posicionamento (estimativa + movimentacao por coordenadas).
// Deve ser chamado no setup, apos inicializarMotoresMovimentacao().
// Se os hooks de orientacao vierem preenchidos em config, o modo com giro
// ja fica ativo automaticamente, sem chamada separada.
void inicializarMotoresPosicionamento(const MotoresPosicionamentoConfig& config = MotoresPosicionamentoConfig());

// Registra callbacks de orientacao usados por moverParaComGiro().
void configurarHooksOrientacaoPosicionamento(const MotoresPosicionamentoOrientacaoHooks& hooks);

// Atualiza as leituras ultrassonicas da iteracao atual.
// Recomenda-se chamar antes de atualizarPosicaoAtual() no loop.
void atualizarLeiturasPosicionamento(float ultraEsquerdaCm,
                                     float ultraDireitaCm,
                                     float ultraFrenteCm,
                                     float ultraTrasCm,
                                     bool leiturasValidas);

// Executa uma nova estimativa filtrada de posicao; retorna false sem dados validos.
bool atualizarPosicaoAtual();

// API padrao de leitura para interface HTML e logica de navegacao.
float obterPosicaoX();
float obterPosicaoY();
float obterConfiancaPosicao();
float obterAnguloAlvoPosicionamentoGraus();

// Vai para (x,y) orientando a frente do robo para a direcao de deslocamento.
bool moverParaComGiro(float xCm, float yCm);

// Vai para (x,y) mantendo a orientacao atual (deslocamento vetorial/omnidirecional).
bool moverParaSemGiro(float xCm, float yCm);

// Detecta linha e executa fuga imediatamente quando confirmada.
// Retorna true quando o comando de fuga foi aplicado.
bool sairDaLinha(bool linhaDetectada, float anguloLinhaGraus, int velocidadePwm,
                 float* anguloComandoSaida = nullptr);

// Reseta estado interno de confirmacao da rotina sairDaLinha.
void resetSairDaLinha();
