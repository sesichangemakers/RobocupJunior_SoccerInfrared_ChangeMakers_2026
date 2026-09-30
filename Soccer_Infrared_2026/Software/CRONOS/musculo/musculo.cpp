


// =============================================================================
// MUSCULO.CPP — Placa de Atuadores e Estratégias do Cronos
// =============================================================================
// Responsabilidades:
//   • Controle de movimento (motores DC via ponte H)
//   • Menu de navegação no display OLED
//   • Calibração de gol (cor), bússola e IR
//   • Alinhamento e correção angular via PID
//   • Lógica do kicker (solenoide)
//   • Estratégias de ATACANTE e DEFENSOR
//
// Entradas:
//   • Mensagens da Cabeça via Serial1 (IR, bússola, linha, ultrasson, câmera, botões)
//   • Sensores de linha/gol e botões físicos
//
// Saídas:
//   • Comandos PWM para 4 motores (rodas omnidirecionais/mecanum)
//   • Pulso do kicker (solenoide)
//   • Telas de status no display OLED 128x64
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// =============================================================================
// SECAO 1 — DEFINICOES GERAIS DE HARDWARE
// =============================================================================

// --- Papel atual do robô recebido da Cabeça ---
// true = atacante | false = defensor
bool papelAtacante = false;
bool papelAtacanteAnterior = false;  // Detecta mudanças de papel entre ciclos

// --- Pinos da Serial com a Cabeça ---
#define RX_CABECA 17
#define TX_CABECA 18

// --- Pinos do barramento I2C e dimensões do display OLED ---
#define SDA_PIN 8
#define SCL_PIN 9
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR     0x3C

// --- Pinos da Ponte H — Conjunto A (Motores 1 e 2) ---
#define IN1_1_A  5
#define IN2_1_A  6
#define PWM_1_A  4

#define IN1_2_A  3
#define IN2_2_A 46
#define PWM_2_A  7

// --- Pinos da Ponte H — Conjunto B (Motores 3 e 4) ---
#define IN1_1_B 11
#define IN2_1_B 12
#define PWM_1_B 10

#define IN1_2_B 13
#define IN2_2_B 14
#define PWM_2_B 47

// --- Canais PWM do ESP32 para cada motor ---
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

// --- Configuração global do PWM dos motores ---
#define PWM_FREQ 20000   // Frequência: 20 kHz (fora da faixa audível)
#define PWM_RES     8    // Resolução: 8 bits (0–255)

// --- Pino e temporização do solenoide (kicker) ---
constexpr uint8_t  KICKER_PIN = 21;
constexpr unsigned long KICK_PULSE_MS = 100;   // Duração do pulso de chute (ms)
constexpr unsigned long KICK_INTERVAL_MS = 1000;  // Intervalo mínimo entre chutes (ms)

// --- Instância do display OLED ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// =============================================================================
// SECAO 2 — VARIAVEIS DE COMUNICACAO E ESTADO GERAL
// =============================================================================

// --- Estado da comunicação com a Cabeça ---
bool   comunicacaoCabecaOK  = false;
String bufferSerial         = "";
String mensagemBotao        = "NENHUM";

// --- Temporização de handshake e timeout ---
unsigned long ultimoEnvioOi    = 0;
unsigned long ultimoRxCabeca   = 0;
unsigned long mostrarStatusAte = 0;

// --- Status das placas auxiliares (Olho e Pé) ---
bool olhoOK = false;
bool peOK   = false;

// --- Estado do jogo enviado à Cabeça ---
bool          estadoJogoCabecaEnviado         = false;
unsigned long ultimoEnvioEstadoJogoCabecaMs   = 0;

// --- Temporização e limites gerais de comunicação ---
const unsigned long INTERVALO_OI_MS               = 1000;
const unsigned long TIMEOUT_COM_MS                = 5000;
const unsigned long INTERVALO_ENVIO_ESTADO_JOGO_MS = 500;

// --- Botão de ação longa (pino físico) ---
const uint8_t BOTAO_MEIO_LONGO = 23;


// =============================================================================
// SECAO 3 — VARIAVEIS DE SENSORES E TELEMETRIA (recebidos da Cabeça)
// =============================================================================

// --- Bússola ---
bool   bussolaValida       = false;
int    headingBussolaTeste = 0;    // Leitura atual da bússola (°)
int    headingBussolaSalvo = 0;    // Referência calibrada salva em EEPROM
unsigned long ultimoRxBussolaMs = 0;
const unsigned long TIMEOUT_BUSSOLA_MS = 800;

// --- Sensor IR (detecção de bola) ---
float  anguloIr              = -1.0;    // Ângulo atual do IR (° ou -1 se sem bola)
bool   irDetectado           = false;
float  ultimoAnguloIrValido  = -1.0f;
unsigned long ultimoIrValidoMs = 0;

// --- Câmera (bola + 2 gols: azul e amarelo) ---
bool   cameraOK          = false;   // Câmera se comunicando com o Olho
bool   cameraDadosValidos = false;
unsigned long ultimoRxCameraMs              = 0;
unsigned long cameraUltimaVezBolaDetetadaMs = 0;
unsigned long cameraSemBolaBrutaInicioMs    = 0;
const unsigned long TIMEOUT_CAMERA_MS       = 1000;

int16_t  cameraBallAngle   = -999;
uint16_t cameraBallDist    = 0;
int16_t  cameraBlueAngle   = -999;
uint16_t cameraBlueDist    = 0;
int16_t  cameraYellowAngle = -999;
uint16_t cameraYellowDist  = 0;

// Gol selecionado (calculado a partir de corGolAzul)
int16_t  cameraGolSelecionadoAngle = -999;
uint16_t cameraGolSelecionadoDist  = 0;
bool     cameraGolSelecionadoValido = false;
bool     cameraGolSelecionadoAzul   = false;

// Buffer circular de ângulos da câmera para filtragem
const uint8_t CAMERA_BOLA_BUFFER_TAM                  = 3;
const float   CAMERA_BOLA_PESO_PREVISAO                = 0.65f;
const float   CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS     = 35.0f;
float    cameraBallBufferAngulos[CAMERA_BOLA_BUFFER_TAM] = {0.0f, 0.0f, 0.0f};
uint8_t  cameraBallBufferIndice    = 0;
uint8_t  cameraBallBufferQuantidade = 0;

// --- Ultrassônicos locais (D=direita, E=esquerda, F=frente, T=trás) ---
float ultraDcm = -1.0f;
float ultraEcm = -1.0f;
float ultraFcm = -1.0f;
float ultraTcm = -1.0f;
bool  ultrasValidos    = false;
unsigned long ultimoRxUltraMs = 0;
const unsigned long TIMEOUT_ULTRA_MS = 1000;

// --- Ultrassônicos remotos (do outro robô, via ESP-NOW) ---
float ultraRemotoDcm = -1.0f;
float ultraRemotoEcm = -1.0f;
float ultraRemotoFcm = -1.0f;
float ultraRemotoTcm = -1.0f;
bool  ultrasRemotosValidos    = false;
unsigned long ultimoRxUltraRemotoMs = 0;

// --- Sensores brutos de linha (LDR do Pé) ---
int sensorBruto1  = -1;
int sensorBruto9  = -1;
int sensorBruto17 = -1;
int sensorBruto25 = -1;
unsigned long ultimoRxSensoresBrutosMs   = 0;
unsigned long ultimoReqSensoresBrutosMs  = 0;
const unsigned long TIMEOUT_SENSORES_BRUTOS_MS        = 1200;
const unsigned long INTERVALO_REQ_SENSORES_BRUTOS_MS  = 250;

// --- Limiar de linha (compartilhado entre Músculo e Pé) ---
int  limiarLinhaEditado          = 2500;
bool limiarLinhaSincronizado     = false;
unsigned long ultimoReqLimiarLinhaMs     = 0;
unsigned long entradaTelaLimiarMs        = 0;
const int  LIMIAR_LINHA_MIN              = 100;
const int  LIMIAR_LINHA_MAX              = 4000;
const int  LIMIAR_LINHA_PASSO            = 100;
const unsigned long INTERVALO_REQ_LIMIAR_LINHA_MS = 400;

// --- Linha (atacante: ângulo único; defensor: zonas A e B) ---
float  anguloLinhaPe        = -1.0f;
bool   linhaDetectada       = false;
float  anguloLinhaZonaA     = -1.0f;
float  anguloLinhaZonaB     = -1.0f;
bool   linhaZonaAValida     = false;
bool   linhaZonaBValida     = false;
unsigned long ultimoRxLinhaMs             = 0;
unsigned long ultimoComandoLinhaMs        = 0;
float  ultimoAnguloLinhaValido            = -1.0f;
float  ultimoAnguloLinhaZonaAValido       = -1.0f;
float  ultimoAnguloLinhaZonaBValido       = -1.0f;
unsigned long ultimoRxLinhaZonaAMs        = 0;
unsigned long ultimoRxLinhaZonaBMs        = 0;
const unsigned long TIMEOUT_LINHA_MS      = 50;

// --- Cor do gol de referência e envio pendente à Cabeça ---
bool corGolAzul             = false;
bool corGolPendenteEnvio    = true;
unsigned long ultimoEnvioCorGolMs         = 0;
const unsigned long INTERVALO_ENVIO_COR_GOL_MS = 500;

// --- Kicker ---
bool kickerRecebido         = false;
bool kickerAtivado          = false;   // true quando chave acionada (valor 0 vindo da Cabeça)
bool pulsoKickerAtivo       = false;
bool pulsoKickerManualAtivo = false;
bool pedidoChuteManual      = false;
unsigned long inicioPulsoKickerMs   = 0;
unsigned long ultimoDisparoKickerMs = 0;

// --- ESP-NOW (comunicação entre as Cabeças) ---
bool espnowOK = false;
bool sozinho  = true;
unsigned long ultimoRxEspnowMs = 0;

// Papel automático: quem está mais perto da bola assume o ataque
constexpr bool  PAPEL_AUTO_DESEMPATE_ATACANTE = true;
constexpr float PAPEL_AUTO_JANELA_EMPATE_CM   = 0.5f;


// =============================================================================
// SECAO 4 — EEPROM: ENDERECOS E FUNCOES DE PERSISTENCIA
// =============================================================================

const int EEPROM_SIZE             = 64;
const int EEPROM_ADDR_BUSSOLA     = 0;
const int EEPROM_ADDR_PAPEL_CONFIG = EEPROM_ADDR_BUSSOLA + (int)sizeof(int);
const int EEPROM_ADDR_COR_GOL     = EEPROM_ADDR_PAPEL_CONFIG + (int)sizeof(uint8_t);

// Enums de configuração de papel e cor de gol (persistidos em EEPROM)
enum PapelConfigurado { PAPEL_CONFIG_ATACANTE, PAPEL_CONFIG_DEFENSOR, PAPEL_CONFIG_AUTO };
PapelConfigurado papelConfiguradoMenu = PAPEL_CONFIG_AUTO;

// Aplica o papel configurado localmente (ignora a Cabeça quando fixo)
void aplicarPapelConfiguradoLocal() {
  if (papelConfiguradoMenu == PAPEL_CONFIG_ATACANTE) {
    papelAtacante = true;
    papelAtacanteAnterior = true;
  } else if (papelConfiguradoMenu == PAPEL_CONFIG_DEFENSOR) {
    papelAtacante = false;
    papelAtacanteAnterior = false;
  }
}

// Salva papel configurado na EEPROM; retorna true se OK
bool salvarPapelConfiguradoEEPROM() {
  uint8_t papelSalvo = (uint8_t)papelConfiguradoMenu;
  EEPROM.put(EEPROM_ADDR_PAPEL_CONFIG, papelSalvo);
  return EEPROM.commit();
}

// Salva cor do gol na EEPROM; retorna true se OK
bool salvarCorGolEEPROM() {
  uint8_t corGolSalva = corGolAzul ? 1 : 0;
  EEPROM.put(EEPROM_ADDR_COR_GOL, corGolSalva);
  return EEPROM.commit();
}

// Carrega papel configurado da EEPROM e aplica localmente
void carregarPapelConfiguradoEEPROM() {
  uint8_t papelSalvo = (uint8_t)PAPEL_CONFIG_AUTO;
  EEPROM.get(EEPROM_ADDR_PAPEL_CONFIG, papelSalvo);
  if (papelSalvo > (uint8_t)PAPEL_CONFIG_AUTO) {
    papelSalvo = (uint8_t)PAPEL_CONFIG_AUTO;
  }
  papelConfiguradoMenu = (PapelConfigurado)papelSalvo;
  aplicarPapelConfiguradoLocal();
}

// Carrega cor do gol da EEPROM
void carregarCorGolEEPROM() {
  uint8_t corGolSalva = 0;
  EEPROM.get(EEPROM_ADDR_COR_GOL, corGolSalva);
  if (corGolSalva > 1) {
    corGolSalva = 0;
  }
  corGolAzul = (corGolSalva == 1);
}


// =============================================================================
// SECAO 5 — ESTADOS DE INTERFACE (MENU, CALIBRACAO, FUNCAO, INICIAR)
// =============================================================================

enum Estado { MENU, CALIBRACAO, FUNCAO, INICIAR };
Estado estadoAtual   = MENU;
int    itemSelecionado = 0;

// Submenus da calibração
enum SubMenuCalibracao {
  SUBMENU_PRINCIPAL,
  SUBMENU_GOL,
  SUBMENU_BUSSOLA,
  SUBMENU_IR,
  SUBMENU_ULTRA,
  SUBMENU_CAMERA,
  SUBMENU_ESPNOW
};
SubMenuCalibracao subMenuCalibracao = SUBMENU_PRINCIPAL;
int itemSubMenu = 0;

// Submenus de função
enum SubMenuFuncao {
  SUBFUNCAO_PRINCIPAL,
  SUBFUNCAO_PAPEIS,
  SUBFUNCAO_SENSORES,
  SUBFUNCAO_LIMIAR_LINHA,
  SUBFUNCAO_KICKER
};
SubMenuFuncao subMenuFuncao     = SUBFUNCAO_PRINCIPAL;
int           itemSubMenuFuncao = 0;


// =============================================================================
// SECAO 6 — CONTROLE DE MOVIMENTO: PARAMETROS GERAIS
// =============================================================================

// --- Velocidade máxima global dos motores (0–255) ---
// *** AJUSTE AQUI para alterar a velocidade máxima do robô ***
const int velocidade_maxima = 180;

// --- Habilita/desabilita o movimento baseado em bola (teste) ---
const bool MOVIMENTO_BOLA_HABILITADO = false;

// --- Rampa de aceleração PWM (suaviza variações bruscas de comando) ---
const int PASSO_RAMPA_PWM = 16;

// --- Configuração de giro no eixo (sentido e velocidade padrão) ---
// *** AJUSTE AQUI para corrigir o sentido ou velocidade de giro de alinhamento ***
#define VELOCIDADE_GIRO  180
#define SINAL_GIRO       -1

// --- Ganho de mistura translação + rotação ---
const float GANHO_GIRO_MISTO = 0.7f;

// --- Temporização de transição angular do IR (rampa suave entre faixas) ---
const unsigned long TRANSICAO_ANGULO_IR_MIN_MS    = 50;
const unsigned long TRANSICAO_ANGULO_IR_MAX_MS    = 100;
const float         TRANSICAO_ANGULO_IR_MS_POR_GRAU = 2.0f;
const float         PASSO_ANGULO_IR_GRAUS          = 5.0f;

// Estado interno da rampa angular do IR
float        anguloIrSuaveAtual        = 0.0f;
float        anguloIrSuaveInicio       = 0.0f;
float        anguloIrSuaveAlvo         = 0.0f;
unsigned long inicioTransicaoIrMs       = 0;
unsigned long duracaoTransicaoIrMs      = TRANSICAO_ANGULO_IR_MIN_MS;
bool         anguloIrSuaveInicializado  = false;

// --- Variáveis de estado de fuga de linha e alinhamento ---
float  erroGolGraus       = 0.0f;   // Erro angular atual em relação ao gol
bool   golDetectado       = false;
uint16_t golPixels        = 0;
float  erroAlinhamentoGraus = 0.0f; // Erro de alinhamento com o gol (°)
bool   alinhandoAgora     = false;  // true quando executando giro de alinhamento
bool   fugindoLinhaAgora  = false;  // true quando executando fuga de linha
float  anguloFugaLinhaCmd = 0.0f;   // Ângulo do comando de fuga enviado aos motores
const int VELOCIDADE_FUGA_LINHA = 255;

// --- Tolerâncias de alinhamento ---
// *** AJUSTE AQUI para afinar a janela de alinhamento com o gol ***
const float TOLERANCIA_ALINHAMENTO_GRAUS   = 8.0f;
const float JANELA_FUZZY_ALINHAMENTO_GRAUS = 30.0f;
const int   VELOCIDADE_GIRO_ALINHAMENTO    = VELOCIDADE_GIRO;
const int   SINAL_GIRO_PID                 = SINAL_GIRO;

// --- Tempo de retenção de dados de linha e zona (evita perda por frame único) ---
const unsigned long RETENCAO_FUGA_LINHA_MS          = 250;
const unsigned long RETENCAO_ZONA_LINHA_DEFENSOR_MS  = 300;
const unsigned long TEMPO_CAMERA_SEM_IR_PARA_IGNORAR_LINHA_MS = 1000;


// =============================================================================
// SECAO 7 — PID GERAL DE ALINHAMENTO (BUSSOLA)
//            Usado por ATACANTE e DEFENSOR para alinhar com o gol via câmera
// =============================================================================

// Ganhos e saturações do PID de alinhamento bússola/câmera
// *** AJUSTE AQUI para afinação do giro de alinhamento com o gol ***
const float PID_BUS_KP            = 0.8f;
const float PID_BUS_KI            = 0.01f;
const float PID_BUS_KD            = 0.5f;
const float PID_BUS_INTEGRAL_MAX  = 120.0f;
const int   PID_BUS_SAIDA_MIN     = 30;
const int   PID_BUS_SAIDA_MAX     = 180;

// Estado interno do PID de bússola (entre iterações do loop)
float        pidBusIntegral      = 0.0f;
float        pidBusErroAnterior  = 0.0f;
unsigned long pidBusUltimoMs     = 0;

// --- Zera o PID de bússola ---
void resetPidBussola() {
  pidBusIntegral     = 0.0f;
  pidBusErroAnterior = 0.0f;
  pidBusUltimoMs     = 0;
}

// --- Calcula saída assinada do PID de bússola com anti-windup ---
int calcularSaidaPidBussola(float erroGraus) {
  unsigned long agora = millis();
  float dt = 0.02f;

  // Calcula dt real e limita extremos para robustez
  if (pidBusUltimoMs != 0) {
    dt = (agora - pidBusUltimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f)   dt = 0.2f;
  }
  pidBusUltimoMs = agora;

  // Termo integral com anti-windup por saturação simples
  pidBusIntegral += erroGraus * dt;
  if (pidBusIntegral >  PID_BUS_INTEGRAL_MAX) pidBusIntegral =  PID_BUS_INTEGRAL_MAX;
  if (pidBusIntegral < -PID_BUS_INTEGRAL_MAX) pidBusIntegral = -PID_BUS_INTEGRAL_MAX;

  // Derivada discreta
  float derivada = (erroGraus - pidBusErroAnterior) / dt;
  pidBusErroAnterior = erroGraus;

  // Ação de controle, módulo e saturações
  float u    = PID_BUS_KP * erroGraus + PID_BUS_KI * pidBusIntegral + PID_BUS_KD * derivada;
  int   saida = (int)fabsf(u);
  if (saida < PID_BUS_SAIDA_MIN)            saida = PID_BUS_SAIDA_MIN;
  if (saida > PID_BUS_SAIDA_MAX)            saida = PID_BUS_SAIDA_MAX;
  if (saida > VELOCIDADE_GIRO_ALINHAMENTO)  saida = VELOCIDADE_GIRO_ALINHAMENTO;

  // Sinal preserva o sentido do erro angular
  return (u >= 0.0f) ? saida : -saida;
}


// =============================================================================
// SECAO 8 — FUNCOES MATEMATICAS UTILITARIAS
// =============================================================================

// Normaliza erro angular para a faixa [-180, 180]
float normalizarErro180(float erro) {
  while (erro >  180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

// Normaliza ângulo absoluto para a faixa [0, 360)
float normalizarAngulo360(float ang) {
  while (ang >= 360.0f) ang -= 360.0f;
  while (ang <    0.0f) ang += 360.0f;
  return ang;
}

// Quantiza ângulo em passos fixos para transições curtas e previsíveis
float quantizarAnguloPasso(float anguloGraus, float passoGraus) {
  if (passoGraus <= 0.0f) {
    return normalizarAngulo360(anguloGraus);
  }
  float ang       = normalizarAngulo360(anguloGraus);
  float quantizado = roundf(ang / passoGraus) * passoGraus;
  return normalizarAngulo360(quantizado);
}

// Valida payload numérico (evita toFloat() silencioso em 0)
bool payloadNumericoValido(const String &texto) {
  if (texto.length() == 0) {
    return false;
  }
  bool encontrouDigito = false;
  bool encontrouPonto  = false;
  for (size_t i = 0; i < texto.length(); i++) {
    char c = texto.charAt(i);
    if (c >= '0' && c <= '9')             { encontrouDigito = true; continue; }
    if (c == '.' && !encontrouPonto)       { encontrouPonto  = true; continue; }
    if ((c == '+' || c == '-') && i == 0) { continue; }
    return false;
  }
  return encontrouDigito;
}

// Mapeia um valor de entrada para saída, com clamping nas bordas
float mapearFaixaClamped(float valor, float entradaMin, float entradaMax,
                         float saidaMin, float saidaMax) {
  float denominador = entradaMax - entradaMin;
  if (fabsf(denominador) < 0.0001f) {
    return saidaMin;
  }
  float proporcao = (valor - entradaMin) / denominador;
  if (proporcao < 0.0f) proporcao = 0.0f;
  if (proporcao > 1.0f) proporcao = 1.0f;
  return saidaMin + ((saidaMax - saidaMin) * proporcao);
}


// =============================================================================
// SECAO 9 — FUNCOES DE CONTROLE DE MOTORES
// =============================================================================

// Aciona o Motor 1 com sentido e PWM conforme velocidade assinada
void Motor_1(int vel1) {
  int pwm1 = constrain(abs(vel1), 0, 255);
  ledcWrite(PWM_CH1, pwm1);
  if (vel1 <= 0) { digitalWrite(IN1_1_A, HIGH); digitalWrite(IN2_1_A, LOW); }
  else           { digitalWrite(IN1_1_A, LOW);  digitalWrite(IN2_1_A, HIGH); }
}

// Aciona o Motor 2 com sentido e PWM conforme velocidade assinada
void Motor_2(int vel2) {
  int pwm2 = constrain(abs(vel2), 0, 255);
  ledcWrite(PWM_CH2, pwm2);
  if (vel2 <= 0) { digitalWrite(IN1_2_A, HIGH); digitalWrite(IN2_2_A, LOW); }
  else           { digitalWrite(IN1_2_A, LOW);  digitalWrite(IN2_2_A, HIGH); }
}

// Aciona o Motor 4 com sentido e PWM conforme velocidade assinada
void Motor_4(int vel3) {
  int pwm3 = constrain(abs(vel3), 0, 255);
  ledcWrite(PWM_CH3, pwm3);
  if (vel3 <= 0) { digitalWrite(IN1_1_B, HIGH); digitalWrite(IN2_1_B, LOW); }
  else           { digitalWrite(IN1_1_B, LOW);  digitalWrite(IN2_1_B, HIGH); }
}

// Aciona o Motor 3 com sentido e PWM conforme velocidade assinada
void Motor_3(int vel4) {
  int pwm4 = constrain(abs(vel4), 0, 255);
  ledcWrite(PWM_CH4, pwm4);
  if (vel4 <= 0) { digitalWrite(IN1_2_B, HIGH); digitalWrite(IN2_2_B, LOW); }
  else           { digitalWrite(IN1_2_B, LOW);  digitalWrite(IN2_2_B, HIGH); }
}

// Para todos os motores, desligando PWM e deixando pontes em estado neutro
void pararMotores() {
  ledcWrite(PWM_CH1, 0);
  ledcWrite(PWM_CH2, 0);
  ledcWrite(PWM_CH3, 0);
  ledcWrite(PWM_CH4, 0);
  digitalWrite(IN1_1_A, LOW); digitalWrite(IN2_1_A, LOW);
  digitalWrite(IN1_2_A, LOW); digitalWrite(IN2_2_A, LOW);
  digitalWrite(IN1_1_B, LOW); digitalWrite(IN2_1_B, LOW);
  digitalWrite(IN1_2_B, LOW); digitalWrite(IN2_2_B, LOW);
}

// Aplica rampa PWM em um único motor para suavizar variações
int aplicarRampaPwm(int alvo, int atual, int passoMaximo) {
  int delta = alvo - atual;
  if (delta >  passoMaximo) return atual + passoMaximo;
  if (delta < -passoMaximo) return atual - passoMaximo;
  return alvo;
}

// Aplica rampa em todos os 4 motores antes de enviar aos drivers
void aplicarComandoMotoresComRampa(int v1Alvo, int v2Alvo, int v3Alvo, int v4Alvo) {
  static int v1Atual = 0;
  static int v2Atual = 0;
  static int v3Atual = 0;
  static int v4Atual = 0;

  int alvo1 = constrain(v1Alvo, -255, 255);
  int alvo2 = constrain(v2Alvo, -255, 255);
  int alvo3 = constrain(v3Alvo, -255, 255);
  int alvo4 = constrain(v4Alvo, -255, 255);

  v1Atual = aplicarRampaPwm(alvo1, v1Atual, PASSO_RAMPA_PWM);
  v2Atual = aplicarRampaPwm(alvo2, v2Atual, PASSO_RAMPA_PWM);
  v3Atual = aplicarRampaPwm(alvo3, v3Atual, PASSO_RAMPA_PWM);
  v4Atual = aplicarRampaPwm(alvo4, v4Atual, PASSO_RAMPA_PWM);

  Motor_1(v1Atual);
  Motor_2(v2Atual);
  Motor_3(v3Atual);
  Motor_4(v4Atual);
}


// Gira o robô no próprio eixo com velocidade assinada
void girarNoEixo(int velocidade) {
  int vel = constrain(velocidade, -velocidade_maxima, velocidade_maxima);
  // Horário: M1/M2 frente e M3/M4 trás; anti-horário: inverso
  aplicarComandoMotoresComRampa(-vel, -vel, -vel, -vel);
}

// Converte ângulo de translação (polar) em velocidades individuais de roda (mecanum/omni)
void seguirDirecaoPorAngulo(float anguloGraus, int velocidade) {
  int velocidadeAlvo = constrain(velocidade, 0, velocidade_maxima);

  // Vetor translacional no referencial do robô
  float theta = anguloGraus * PI / 180.0;
  float vx = velocidadeAlvo * sin(theta);
  float vy = velocidadeAlvo * cos(theta);

  // Ângulos físicos das rodas omnidirecionais (45°, 135°, 225°, 315°)
  float theta1 = 45.0  * PI / 180.0;
  float theta2 = 135.0 * PI / 180.0;
  float theta3 = 225.0 * PI / 180.0;
  float theta4 = 315.0 * PI / 180.0;

  // Projeção do vetor em cada roda
  float v1 = vx * cos(theta1) + vy * sin(theta1);
  float v2 = vx * cos(theta2) + vy * sin(theta2);
  float v3 = vx * cos(theta3) + vy * sin(theta3);
  float v4 = vx * cos(theta4) + vy * sin(theta4);

  // Renormaliza para manter a maior roda dentro do limite
  float maxVel = max(max(abs(v1), abs(v2)), max(abs(v3), abs(v4)));
  if (maxVel > velocidade_maxima) {
    float escala = (float)velocidade_maxima / maxVel;
    v1 *= escala; v2 *= escala; v3 *= escala; v4 *= escala;
  }

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}

// Move por ângulo e soma termo de giro para corrigir orientação durante o deslocamento
void seguirDirecaoComGiro(float anguloGraus, int velocidade, int cmdGiro) {
  int velocidadeAlvo = constrain(velocidade, 0, velocidade_maxima);

  float theta = anguloGraus * PI / 180.0f;
  float vx = velocidadeAlvo * sinf(theta);
  float vy = velocidadeAlvo * cosf(theta);

  float theta1 = 45.0f  * PI / 180.0f;
  float theta2 = 135.0f * PI / 180.0f;
  float theta3 = 225.0f * PI / 180.0f;
  float theta4 = 315.0f * PI / 180.0f;

  float v1 = vx * cosf(theta1) + vy * sinf(theta1);
  float v2 = vx * cosf(theta2) + vy * sinf(theta2);
  float v3 = vx * cosf(theta3) + vy * sinf(theta3);
  float v4 = vx * cosf(theta4) + vy * sinf(theta4);

  // Sobrepõe o termo de rotação à translação (mesmo sentido de girarNoEixo)
  float termoGiro = -GANHO_GIRO_MISTO * (float)cmdGiro;
  v1 += termoGiro; v2 += termoGiro; v3 += termoGiro; v4 += termoGiro;

  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
  if (maxVel > velocidade_maxima) {
    float escala = (float)velocidade_maxima / maxVel;
    v1 *= escala; v2 *= escala; v3 *= escala; v4 *= escala;
  }

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}

// Move para frente com PWM fixo nas rodas, mantendo correção de giro
void moverFrenteComGiro(int velocidadePwm, int cmdGiro) {
  int pwmBase = constrain(velocidadePwm, 0, 255);

  // Padrão equivalente a avançar para frente no referencial atual
  float v1 =  (float)pwmBase;
  float v2 =  (float)pwmBase;
  float v3 = -(float)pwmBase;
  float v4 = -(float)pwmBase;

  // Mesmo sentido de compensação de giro do restante da locomoção
  float termoGiro = -GANHO_GIRO_MISTO * (float)cmdGiro;
  v1 += termoGiro; v2 += termoGiro; v3 += termoGiro; v4 += termoGiro;

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}


// =============================================================================
// SECAO 10 — KICKER (SOLENOIDE)
// =============================================================================

// Solicita chute manual pelo submenu kicker
void solicitarChuteManualkicker() {
  pedidoChuteManual = true;
}

// Controla o pulso do kicker com tempo mínimo e intervalo entre disparos
void atualizarKicker() {
  unsigned long agora = millis();

  // Finaliza o pulso após o tempo mínimo energizado
  if (pulsoKickerAtivo && (agora - inicioPulsoKickerMs) >= KICK_PULSE_MS) {
    digitalWrite(KICKER_PIN, LOW);
    pulsoKickerAtivo       = false;
    pulsoKickerManualAtivo = false;
  }

  // Disparo manual solicitado pelo submenu kicker
  if (pedidoChuteManual && !pulsoKickerAtivo) {
    digitalWrite(KICKER_PIN, HIGH);
    inicioPulsoKickerMs    = agora;
    ultimoDisparoKickerMs  = agora;
    pulsoKickerAtivo       = true;
    pulsoKickerManualAtivo = true;
    pedidoChuteManual      = false;
    Serial.println("Chute manual");
    return;
  }
  pedidoChuteManual = false;

  // Só permite chute automático durante o jogo, com comunicação válida e chave acionada
  bool podeChutar = (estadoAtual == INICIAR) && comunicacaoCabecaOK && kickerRecebido && kickerAtivado;
  if (!podeChutar) {
    // Desliga imediatamente apenas o pulso automático
    if (pulsoKickerAtivo && !pulsoKickerManualAtivo) {
      digitalWrite(KICKER_PIN, LOW);
      pulsoKickerAtivo = false;
    }
    return;
  }

  // Dispara novo pulso respeitando o intervalo mínimo entre chutes
  if (!pulsoKickerAtivo && (agora - ultimoDisparoKickerMs) >= KICK_INTERVAL_MS) {
    digitalWrite(KICKER_PIN, HIGH);
    inicioPulsoKickerMs   = agora;
    ultimoDisparoKickerMs = agora;
    pulsoKickerAtivo      = true;
    pulsoKickerManualAtivo = false;
    Serial.println("Chutei");
  }
}


// =============================================================================
// SECAO 11 — FUNCOES DE GOL, BUSSOLA E ANGULO DE CAMPO (compartilhadas)
// =============================================================================

// Retorna true se o gol selecionado é o azul
bool golReferenciaAzulEfetiva() {
  return corGolAzul;
}

// Converte ângulo de gol para o referencial do defensor (inversão de 180°)
// Ex.: 30 -> 150, 200 -> 340
float converterAnguloGolParaDefensor(float anguloGolGraus) {
  float invertido = normalizarAngulo360(anguloGolGraus + 180.0f);
  return normalizarAngulo360(360.0f - invertido);
}

// Calcula erro de alinhamento com o gol no referencial do defensor (espelhado)
float calcularErroGolEspelhadoDefensor(float anguloGolGraus) {
  return normalizarErro180(anguloGolGraus - 180.0f);
}

// Retorna o erro de alinhamento com o gol conforme o papel atual
float calcularErroGolPorPapel(float anguloGolGraus) {
  if (papelAtacante) {
    return normalizarErro180(anguloGolGraus);
  }
  return calcularErroGolEspelhadoDefensor(anguloGolGraus);
}

// Calcula o erro angular atual em relação à referência salva da bússola
float calcularErroReferenciaBussola() {
  if (!bussolaValida) {
    return 0.0f;
  }
  return normalizarErro180((float)headingBussolaSalvo - (float)headingBussolaTeste);
}

// Gera ângulo local de translação para voltar ao gol pela bússola
float calcularAnguloRetornoGolPorBussola() {
  return normalizarAngulo360(180.0f + calcularErroReferenciaBussola());
}

// Retorna true se a bússola possui referência válida
bool bussolaTemReferenciaValida() {
  return bussolaValida;
}

// Atualiza validade da bússola por timeout
void atualizarValidadeBussola() {
  if (bussolaValida && (millis() - ultimoRxBussolaMs) > TIMEOUT_BUSSOLA_MS) {
    bussolaValida = false;
  }
}

// Calcula o erro do campo (mesma base do gol invertido)
float calcularErroAngularCampo() {
  return erroAlinhamentoGraus;
}

// Corrige ângulo local para o referencial do campo
float corrigirAnguloParaCampo(float anguloLocalGraus, float erroAngularGraus) {
  return normalizarAngulo360(anguloLocalGraus + erroAngularGraus);
}

// Regras de deslocamento lateral pela bola corrigida no campo
// Direita real (20..120) -> 120 - erroAngular
// Esquerda real (240..340) -> 340 + erroAngular
bool calcularComandoLateralPorBolaCorrigida(float anguloBolaCorrigido,
                                            float erroAngular,
                                            float &anguloComando) {
  float ang = normalizarAngulo360(anguloBolaCorrigido);
  if (ang >= 20.0f && ang <= 120.0f) {
    anguloComando = normalizarAngulo360(120.0f - erroAngular);
    return true;
  }
  if (ang >= 240.0f && ang <= 340.0f) {
    anguloComando = normalizarAngulo360(340.0f + erroAngular);
    return true;
  }
  return false;
}


// =============================================================================
// SECAO 12 — FUNCOES DE CAMERA (compartilhadas)
// =============================================================================

// Retorna true se o pacote de câmera está dentro do timeout
bool cameraPacoteRecente() {
  return (ultimoRxCameraMs > 0) && ((millis() - ultimoRxCameraMs) <= TIMEOUT_CAMERA_MS);
}

// Atualiza flags de validade e seleciona gol conforme corGolAzul
void atualizarValidadeCamera() {
  cameraDadosValidos          = cameraPacoteRecente();
  cameraGolSelecionadoAzul    = corGolAzul;
  cameraGolSelecionadoAngle   = cameraGolSelecionadoAzul ? cameraBlueAngle  : cameraYellowAngle;
  cameraGolSelecionadoDist    = cameraGolSelecionadoAzul ? cameraBlueDist   : cameraYellowDist;
  cameraGolSelecionadoValido  = cameraPacoteRecente() &&
                                (cameraGolSelecionadoAngle != -999) &&
                                (cameraGolSelecionadoDist > 0);
}

// Retorna true se a câmera está vendo a bola com dados válidos
bool cameraTemBolaValida() {
  return cameraPacoteRecente() &&
         (cameraBallAngle != -999) &&
         (cameraBallDist > 0) &&
         !((cameraBallAngle == 0) && (cameraBallDist == 0));
}

// Adiciona leitura de ângulo de bola ao buffer circular
void adicionarLeituraCameraNoBuffer(float anguloGraus) {
  cameraBallBufferAngulos[cameraBallBufferIndice] = normalizarAngulo360(anguloGraus);
  cameraBallBufferIndice = (cameraBallBufferIndice + 1) % CAMERA_BOLA_BUFFER_TAM;
  if (cameraBallBufferQuantidade < CAMERA_BOLA_BUFFER_TAM) {
    cameraBallBufferQuantidade++;
  }
}

// Filtra o ângulo da bola pelo buffer (média circular + predição)
bool obterAnguloCameraBolaFiltrado(float &anguloFiltrado) {
  if (!cameraPacoteRecente() || cameraBallBufferQuantidade == 0) {
    return false;
  }
  float somaSin = 0.0f;
  float somaCos = 0.0f;
  for (uint8_t i = 0; i < cameraBallBufferQuantidade; i++) {
    float rad = cameraBallBufferAngulos[i] * PI / 180.0f;
    somaSin += sinf(rad);
    somaCos += cosf(rad);
  }
  float mediaCircular = normalizarAngulo360(atan2f(somaSin, somaCos) * 180.0f / PI);
  int idxUltimo = (cameraBallBufferIndice + CAMERA_BOLA_BUFFER_TAM - 1) % CAMERA_BOLA_BUFFER_TAM;
  float anguloPrevisto = cameraBallBufferAngulos[idxUltimo];

  // Predição de um passo à frente pelo delta angular mais recente
  if (cameraBallBufferQuantidade >= 2) {
    int idxPenultimo = (idxUltimo + CAMERA_BOLA_BUFFER_TAM - 1) % CAMERA_BOLA_BUFFER_TAM;
    float deltaPrev = normalizarErro180(cameraBallBufferAngulos[idxUltimo] - cameraBallBufferAngulos[idxPenultimo]);
    if (deltaPrev >  CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS) deltaPrev =  CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS;
    if (deltaPrev < -CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS) deltaPrev = -CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS;
    anguloPrevisto = normalizarAngulo360(cameraBallBufferAngulos[idxUltimo] + deltaPrev);
  }

  float erroPrevMedia = normalizarErro180(anguloPrevisto - mediaCircular);
  anguloFiltrado = normalizarAngulo360(mediaCircular + (erroPrevMedia * CAMERA_BOLA_PESO_PREVISAO));
  return true;
}

// Lê gol selecionado (para o menu de calibração)
bool cameraLerGolSelecionadoMenu(int16_t &anguloGol, uint16_t &distGol) {
  anguloGol = cameraGolSelecionadoAngle;
  distGol   = cameraGolSelecionadoDist;
  return cameraGolSelecionadoValido;
}

// Retorna true e ângulo do gol selecionado se visível
bool cameraTemGolSelecionadoValido(int16_t &anguloGol) {
  anguloGol = cameraGolSelecionadoAngle;
  return cameraGolSelecionadoValido;
}

// Retorna true e ângulo do gol de retorno do defensor (gol adversário)
bool cameraTemGolRetornoDefensorValido(int16_t &anguloGol) {
  bool     usarGolAzul = !corGolAzul;
  uint16_t distGol     = usarGolAzul ? cameraBlueDist    : cameraYellowDist;
  anguloGol             = usarGolAzul ? cameraBlueAngle   : cameraYellowAngle;
  return cameraPacoteRecente() && (anguloGol != -999) && (distGol > 0);
}


// =============================================================================
// SECAO 13 — FUNCOES DE LINHA (compartilhadas)
// =============================================================================

// Declarações antecipadas para uso interno nas funções de linha
extern bool  linhaDetectada;
extern float anguloLinhaPe;

// Descarta leituras antigas para não manter fuga ativa com dado obsoleto
void atualizarValidadeLinha() {
  if (linhaDetectada && (millis() - ultimoRxLinhaMs) > TIMEOUT_LINHA_MS) {
    linhaDetectada = false;
    anguloLinhaPe  = -1.0f;
  }
  if ((millis() - ultimoRxLinhaMs) > TIMEOUT_LINHA_MS) {
    linhaZonaAValida = false;
    linhaZonaBValida = false;
    anguloLinhaZonaA = -1.0f;
    anguloLinhaZonaB = -1.0f;
  }
}

// Calcula direção de centralização para teste de linha (teste de centro)
float calcularDirecaoCentroLinhaTeste() {
  if (!linhaDetectada || anguloLinhaPe < 0.0f) {
    return -1.0f;
  }
  // Exibe o comando puro da linha, sem inversão de repulsão
  return normalizarAngulo360(anguloLinhaPe);
}


// =============================================================================
// SECAO 14 — FUNCOES ESP-NOW E PAPEL AUTOMATICO
// =============================================================================

// Retorna true se ESP-NOW está conectado e com pacote recente
bool espnowConectadoRecente() {
  return espnowOK && (ultimoRxEspnowMs > 0) && ((millis() - ultimoRxEspnowMs) <= TIMEOUT_COM_MS);
}

// Atualiza flag "sozinho" conforme estado do ESP-NOW durante o jogo
void atualizarSozinhoLocal() {
  if (estadoAtual == INICIAR) {
    sozinho = !espnowConectadoRecente();
  }
}

// Envia estado de jogo (RUN:1/0) para a Cabeça
void enviarEstadoJogoParaCabeca(bool forcar = false) {
  bool emJogo = (estadoAtual == INICIAR);
  unsigned long agora = millis();
  if (!forcar &&
      emJogo == estadoJogoCabecaEnviado &&
      (agora - ultimoEnvioEstadoJogoCabecaMs) < INTERVALO_ENVIO_ESTADO_JOGO_MS) {
    return;
  }
  Serial1.print("RUN:");
  Serial1.println(emJogo ? 1 : 0);
  estadoJogoCabecaEnviado        = emJogo;
  ultimoEnvioEstadoJogoCabecaMs  = agora;
}

// Envia papel atual (ATCFB:1/0) para a Cabeça ressincronizar o Pé
void enviarPapelAtualParaCabeca() {
  Serial1.print("ATCFB:");
  Serial1.println(papelAtacante ? 1 : 0);
}

// Retorna true se ultrassônicos locais estão recentes (para decisão de papel)
bool ultrasLocaisRecentesParaPapel() {
  return ultrasValidos && (ultimoRxUltraMs > 0) && ((millis() - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);
}

// Retorna true se ultrassônicos remotos estão recentes (para decisão de papel)
bool ultrasRemotosRecentesParaPapel() {
  return ultrasRemotosValidos && (ultimoRxUltraRemotoMs > 0) && ((millis() - ultimoRxUltraRemotoMs) <= TIMEOUT_ULTRA_MS);
}

// Atualiza o papel automático com base na parceria (ESP-NOW)
void atualizarPapelAutomaticoPorParceria() {
  if (papelConfiguradoMenu != PAPEL_CONFIG_AUTO) {
    return;
  }
  bool novoPapelAtacante = true;
  if (novoPapelAtacante != papelAtacante) {
    papelAtacante        = novoPapelAtacante;
    papelAtacanteAnterior = novoPapelAtacante;
    enviarPapelAtualParaCabeca();
  }
}


// =============================================================================
// SECAO 15 — FUNCOES DE ENVIO PARA A CABECA
// =============================================================================

// Envia periodicamente a cor de gol selecionada para a Cabeça
void enviarCorGolParaCabeca() {
  if (!corGolPendenteEnvio) {
    return;
  }
  if ((millis() - ultimoEnvioCorGolMs) < INTERVALO_ENVIO_COR_GOL_MS) {
    return;
  }
  // A Cabeça confirma com "CFG:GOL:OK" quando assumir a configuração
  Serial1.print("CFG:GOL:");
  Serial1.println(corGolAzul ? "1" : "0");
  ultimoEnvioCorGolMs = millis();
}

// Solicita leituras brutas de sensor ao Pé (somente na tela de sensores)
void solicitarSensoresBrutosPe(bool forcar = false) {
  if (estadoAtual != FUNCAO || subMenuFuncao != SUBFUNCAO_SENSORES) {
    return;
  }
  if (!forcar && (millis() - ultimoReqSensoresBrutosMs) < INTERVALO_REQ_SENSORES_BRUTOS_MS) {
    return;
  }
  Serial1.println("REQ:SENS");
  ultimoReqSensoresBrutosMs = millis();
}

// Solicita limiar de linha ao Pé (somente na tela de ajuste de limiar)
void solicitarLimiarLinhaPe(bool forcar = false) {
  if (estadoAtual != FUNCAO || subMenuFuncao != SUBFUNCAO_LIMIAR_LINHA) {
    return;
  }
  if (!forcar && (millis() - ultimoReqLimiarLinhaMs) < INTERVALO_REQ_LIMIAR_LINHA_MS) {
    return;
  }
  Serial1.println("REQ:LIM");
  ultimoReqLimiarLinhaMs = millis();
}

// Envia limiar de linha editado para o Pé aplicar
void enviarLimiarLinhaParaPe() {
  Serial1.print("SETLIM:");
  Serial1.println(limiarLinhaEditado);
}


// =============================================================================
// SECAO 16 — INTERFACE OLED: TELAS DE MENU E CALIBRACAO
// =============================================================================

// Desenha a tela principal de menu
void desenharMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("==== MENU ====");
  display.println();
  int16_t anguloGolMenu = -999;
  bool golVisivelMenu = cameraTemGolSelecionadoValido(anguloGolMenu);

  // Item 0: CALIBRACAO
  if (itemSelecionado == 0) {
    display.fillRect(0, 16, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 18);
    display.println("CALIBRACAO");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(4, 18);
    display.println("CALIBRACAO");
  }

  // Item 1: FUNCAO
  if (itemSelecionado == 1) {
    display.fillRect(0, 32, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 34);
    display.println("FUNCAO");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(4, 34);
    display.println("FUNCAO");
  }

  // Item 2: INICIAR
  if (itemSelecionado == 2) {
    display.fillRect(0, 44, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 46);
    display.println("INICIAR");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(4, 46);
    display.println("INICIAR");
  }

  // Rodapé: papel atual e status do ESP-NOW
  display.setCursor(0, 56);
  display.print("P:");
  display.print(papelAtacante ? "ATC" : "DEF");
  display.print(" ESN:");
  display.print(espnowConectadoRecente() ? "ON" : "OFF");
  display.display();
}

// Desenha as telas do submenu de função (papéis, sensores, limiar, kicker)
void desenharSubmenuFuncao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== FUNCAO ===");
  display.println();

  // --- Nível principal: escolhe entre PAPEIS, SENSORES, KICKER ou VOLTA ---
  if (subMenuFuncao == SUBFUNCAO_PRINCIPAL) {
    if (itemSubMenuFuncao == 0) {
      display.fillRect(0, 16, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 16);
      display.println("PAPEIS");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 16);
      display.println("PAPEIS");
    }
    if (itemSubMenuFuncao == 1) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 24);
      display.println("SENSORES");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 24);
      display.println("SENSORES");
    }
    if (itemSubMenuFuncao == 2) {
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32);
      display.println("KICKER");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 32);
      display.println("KICKER");
    }
    if (itemSubMenuFuncao == 3) {
      display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 40);
      display.println("VOLTA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 40);
      display.println("VOLTA");
    }
    display.setCursor(0, 48);
    display.print("MODO: ");
    if      (papelConfiguradoMenu == PAPEL_CONFIG_ATACANTE) display.println("ATACANTE");
    else if (papelConfiguradoMenu == PAPEL_CONFIG_DEFENSOR) display.println("DEFENSOR");
    else                                                    display.println("AUTO");
    display.setCursor(0, 56);
    display.print("PAPEL: ");
    display.print(papelAtacante ? "ATC" : "DEF");
    display.print(" ");
    display.println(espnowConectadoRecente() ? "ON" : "OFF");

  // --- Nível papeis: ATACANTE, DEFENSOR, AUTO, VOLTA ---
  } else if (subMenuFuncao == SUBFUNCAO_PAPEIS) {
    if (itemSubMenuFuncao == 0) {
      display.fillRect(0, 16, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 16);
      display.println("ATACANTE");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 16);
      display.println("ATACANTE");
    }
    if (itemSubMenuFuncao == 1) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 24);
      display.println("DEFENSOR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 24);
      display.println("DEFENSOR");
    }
    if (itemSubMenuFuncao == 2) {
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32);
      display.println("AUTO");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 32);
      display.println("AUTO");
    }
    if (itemSubMenuFuncao == 3) {
      display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 40);
      display.println("VOLTA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 40);
      display.println("VOLTA");
    }
    display.setCursor(0, 56);
    display.println("BTN3 CONFIRMA");

  // --- Nível sensores: exibe leituras brutas dos LDRs ---
  } else if (subMenuFuncao == SUBFUNCAO_SENSORES) {
    bool sensoresRecentes = (ultimoRxSensoresBrutosMs > 0) &&
                            ((millis() - ultimoRxSensoresBrutosMs) <= TIMEOUT_SENSORES_BRUTOS_MS);
    display.setCursor(0, 8);
    display.println("LDR BRUTO PE");
    display.setCursor(0, 20);
    display.print("S1 :"); display.print(sensoresRecentes ? sensorBruto1  : -1);
    display.setCursor(66, 20);
    display.print("S9 :"); display.print(sensoresRecentes ? sensorBruto9  : -1);
    display.setCursor(0, 34);
    display.print("S17:"); display.print(sensoresRecentes ? sensorBruto17 : -1);
    display.setCursor(66, 34);
    display.print("S25:"); display.print(sensoresRecentes ? sensorBruto25 : -1);
    display.setCursor(0, 46);
    display.println(sensoresRecentes ? "DADOS: OK" : "DADOS: AGUARDANDO");
    display.setCursor(0, 56);
    display.println("BTN3 ABRE LIMIAR");

  // --- Nível limiar: ajuste manual do limiar de linha ---
  } else if (subMenuFuncao == SUBFUNCAO_LIMIAR_LINHA) {
    display.setCursor(0, 0);
    display.println("AJUSTE LIMIAR LINHA");
    display.setCursor(61, 16);
    display.println("^");
    display.setTextSize(2);
    display.setCursor(24, 26);
    display.println(limiarLinhaEditado);
    display.setTextSize(1);
    display.setCursor(61, 48);
    display.println("v");
    display.setCursor(0, 56);
    display.println("BTN3 SALVA E VOLTA");

  // --- Nível kicker: CHUTAR ou VOLTAR ---
  } else if (subMenuFuncao == SUBFUNCAO_KICKER) {
    if (itemSubMenuFuncao == 0) {
      display.fillRect(0, 18, 128, 12, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(24, 20);
      display.println("CHUTAR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(24, 20);
      display.println("CHUTAR");
    }
    if (itemSubMenuFuncao == 1) {
      display.fillRect(0, 36, 128, 12, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(24, 38);
      display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(24, 38);
      display.println("VOLTAR");
    }
    display.setCursor(0, 56);
    display.println("BTN3 CONFIRMA");
  }

  display.display();
}

// Desenha as telas do submenu de calibração (gol, bússola, IR, ultra, câmera, linha)
void desenharSubmenuCalibracao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("== CALIBRACAO ==");
  display.println();

  // --- Nível principal: lista as opções de calibração ---
  if (subMenuCalibracao == SUBMENU_PRINCIPAL) {
    if (itemSubMenu == 0) {
      display.fillRect(0, 16, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 16); display.println("GOL");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 16); display.println("GOL"); }

    if (itemSubMenu == 1) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 24); display.println("BUSSOLA");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 24); display.println("BUSSOLA"); }

    if (itemSubMenu == 2) {
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32); display.println("IR");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 32); display.println("IR"); }

    if (itemSubMenu == 3) {
      display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 40); display.println("ULTRA");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 40); display.println("ULTRA"); }

    if (itemSubMenu == 4) {
      display.fillRect(0, 48, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 48); display.println("CAM");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 48); display.println("CAM"); }

    if (itemSubMenu == 5) {
      display.fillRect(0, 56, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 56); display.println("LINHA CTR");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 56); display.println("LINHA CTR"); }

    if (itemSubMenu == 6) {
      display.fillRect(96, 0, 32, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(98, 2); display.println("VOLTA");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(98, 2); display.println("VOLTA"); }

  // --- Seleção de cor do gol (amarelo ou azul) ---
  } else if (subMenuCalibracao == SUBMENU_GOL) {
    display.println("Selecione cor do gol");
    if (itemSubMenu == 0) {
      display.fillRect(0, 24, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 26); display.println("AMARELO");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 26); display.println("AMARELO"); }
    if (itemSubMenu == 1) {
      display.fillRect(0, 40, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 42); display.println("AZUL");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 42); display.println("AZUL"); }
    if (itemSubMenu == 2) {
      display.fillRect(0, 54, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 56); display.println("VOLTA");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 56); display.println("VOLTA"); }

  // --- Diagnóstico do IR vindo da Cabeça ---
  } else if (subMenuCalibracao == SUBMENU_IR) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("=== TESTE IR ===");
    display.println();
    display.print("COM CABECA: ");
    display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
    display.print("ANGULO IR: ");
    if (irDetectado) { display.print(anguloIr, 1); display.println(" deg"); }
    else               { display.println("SEM BOLA"); }
    display.println();
    display.println("BTN1/2/3 VOLTAR");

  // --- Diagnóstico dos ultrassônicos ---
  } else if (subMenuCalibracao == SUBMENU_ULTRA) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("=== TESTE ULTRA ===");
    display.println();
    if (ultrasValidos && ((millis() - ultimoRxUltraMs) < TIMEOUT_ULTRA_MS)) {
      display.print("D: "); display.print(ultraDcm, 1); display.println(" cm");
      display.print("E: "); display.print(ultraEcm, 1); display.println(" cm");
      display.print("F: "); display.print(ultraFcm, 1); display.println(" cm");
      display.print("T: "); display.print(ultraTcm, 1); display.println(" cm");
    } else {
      display.println("SEM DADOS");
      display.println("ULTRA DA CABECA");
      display.println("AGUARDANDO...");
    }
    display.println("BTN1/2/3 VOLTAR");

  // --- Diagnóstico de linha (atacante: ângulo único; defensor: zonas A e B) ---
  } else if (subMenuCalibracao == SUBMENU_ESPNOW) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("=== TESTE LINHA ===");
    display.setCursor(0, 12);
    display.print("PAPEL: ");
    display.println(papelAtacante ? "ATC" : "DEF");
    if (papelAtacante) {
      display.setCursor(0, 24);
      display.print("ANG: ");
      if (linhaDetectada && anguloLinhaPe >= 0.0f) { display.print(anguloLinhaPe, 1); display.println(" deg"); }
      else                                           { display.println("SEM LEITURA"); }
      display.setCursor(0, 40);
      display.print("ESN: ");
      display.println(espnowConectadoRecente() ? "CONECTADO" : "DESCONECTADO");
    } else {
      display.setCursor(0, 22);
      display.print("A: ");
      if (linhaZonaAValida && anguloLinhaZonaA >= 0.0f) { display.print(anguloLinhaZonaA, 1); display.println(" deg"); }
      else                                                { display.println("SEM LEITURA"); }
      display.setCursor(0, 32);
      display.print("B: ");
      if (linhaZonaBValida && anguloLinhaZonaB >= 0.0f) { display.print(anguloLinhaZonaB, 1); display.println(" deg"); }
      else                                                { display.println("SEM LEITURA"); }
      display.setCursor(0, 44);
      display.print("ESN: ");
      display.println(espnowConectadoRecente() ? "CONECTADO" : "DESCONECTADO");
    }
    display.setCursor(0, 56);
    display.println("BTN1/2/3 VOLTAR");

  // --- Diagnóstico da câmera (bola + 2 gols: ângulo e distância) ---
  } else if (subMenuCalibracao == SUBMENU_CAMERA) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("TESTE CAM");
    bool dataTimeout = !cameraPacoteRecente();
    display.setTextSize(2);
    display.setCursor(0, 16);
    if (!dataTimeout) {
      display.print("B ");  display.print(cameraBallAngle);   display.print("/"); display.println(cameraBallDist);
      display.print("AZ "); display.print(cameraBlueAngle);   display.print("/"); display.println(cameraBlueDist);
      display.print("AM "); display.print(cameraYellowAngle); display.print("/"); display.println(cameraYellowDist);
    } else {
      display.println("SEM");
      display.println("SINAL");
    }

  // --- Calibração da bússola: mostra leitura atual e valor salvo ---
  } else {
    display.setCursor(0, 0);
    display.println("BUSSOLA AGORA");
    display.println();
    display.print("COM: ");
    display.println((comunicacaoCabecaOK && bussolaValida) ? "OK" : "SEM DADO");
    display.setTextSize(3);
    display.setCursor(8, 20);
    if (bussolaValida) {
      display.print(headingBussolaTeste);
      display.print((char)247);
    } else {
      display.setTextSize(2);
      display.setCursor(8, 24);
      display.print("---");
    }
    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print("SALVO:");
    display.print(headingBussolaSalvo);
    display.print((char)247);
    display.print("  BTN3 SALVA");
  }

  display.display();
}

// Desenha a tela de operação (jogo ativo) com telemetria de sensores
void desenharOperacao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== OPERACAO TESTE ===");
  display.println();
  int16_t anguloGolOperacao = -999;
  bool golVisivelOperacao = cameraTemGolSelecionadoValido(anguloGolOperacao);
  display.print("COM CABECA: "); display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
  display.print("IR ANG: ");
  if (irDetectado) { display.print(anguloIr, 1); display.println(" deg"); }
  else               { display.println("NAO DETECTADO"); }
  display.print("ERRO GOL: ");
  if (golVisivelOperacao) {
    float erroGolMostrado = calcularErroGolPorPapel((float)anguloGolOperacao);
    display.print(erroGolMostrado, 1); display.println(" deg");
  } else {
    display.println("SEM GOL");
  }
  display.print("LINHA ANG: ");
  if (linhaDetectada) {
    display.print(anguloLinhaPe, 1); display.println(" deg");
    display.print("FUGA CMD: "); display.print(anguloFugaLinhaCmd, 1); display.println(" deg");
  } else {
    display.println("SEM LINHA");
  }
  if (golVisivelOperacao) {
    display.print("ERRO: "); display.print(erroAlinhamentoGraus, 1); display.println(" deg");
    if (fugindoLinhaAgora) { display.println("MODO: FUGINDO LINHA"); }
    else                   { display.println(alinhandoAgora ? "MODO: ALINHANDO GOL" : "MODO: SEGUINDO BOLA"); }
    display.print("PIX: ");
    bool usarAzul = golReferenciaAzulEfetiva();
    display.println(usarAzul ? cameraBlueDist : cameraYellowDist);
  } else if (fugindoLinhaAgora) {
    display.println("MODO: FUGINDO LINHA");
  }
  display.display();
}

// Exibe tela com status de comunicação entre as placas
void desenharStatusPlacas() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== TESTE PLACAS ===");
  display.println();
  display.print("MUSC<->CAB: "); display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
  display.print("OLHO: ");       display.println(olhoOK ? "OK" : "FALHA");
  display.print("PE: ");         display.println(peOK   ? "OK" : "FALHA");
  display.display();
}

// Exibe tela de falha quando o handshake com a Cabeça cai
void mostrarTelaFalhaComunicacao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== TESTE COM/BTN ===");
  display.println();
  display.println("COM CABECA: FALHA");
  display.println("Verifique serial 17/18");
  display.println();
  display.print("Ultimo: "); display.println(mensagemBotao);
  display.display();
}

// Roteia para a tela correta conforme estado atual e comunicação
void desenharTelaAtual() {
  if (millis() < mostrarStatusAte) {
    desenharStatusPlacas();
    return;
  }
  // Sem handshake válido, tela de falha tem prioridade absoluta
  if (!comunicacaoCabecaOK) {
    mostrarTelaFalhaComunicacao();
    return;
  }
  if      (estadoAtual == MENU)       desenharMenu();
  else if (estadoAtual == CALIBRACAO) desenharSubmenuCalibracao();
  else if (estadoAtual == FUNCAO)     desenharSubmenuFuncao();
  else                                desenharOperacao();
}


// =============================================================================
// SECAO 17 — INTERFACE OLED: PROCESSAMENTO DE BOTOES
// =============================================================================

// Trata eventos de botão e navega entre menus, calibrações e operação
void processarEventoBotao(uint8_t botao) {

  // --- MENU PRINCIPAL: BTN1 sobe, BTN2 desce, BTN3 confirma ---
  if (estadoAtual == MENU) {
    if      (botao == 1) { itemSelecionado--; if (itemSelecionado < 0) itemSelecionado = 2; }
    else if (botao == 2) { itemSelecionado++; if (itemSelecionado > 2) itemSelecionado = 0; }
    else if (botao == 3) {
      if (itemSelecionado == 0) {
        estadoAtual = CALIBRACAO; subMenuCalibracao = SUBMENU_PRINCIPAL; itemSubMenu = 0;
      } else if (itemSelecionado == 1) {
        estadoAtual = FUNCAO; subMenuFuncao = SUBFUNCAO_PRINCIPAL; itemSubMenuFuncao = 0;
      } else {
        estadoAtual = INICIAR; atualizarSozinhoLocal(); enviarEstadoJogoParaCabeca(true);
      }
    }
    return;
  }

  // --- FUNCAO > PRINCIPAL ---
  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_PRINCIPAL) {
    if      (botao == 1) { itemSubMenuFuncao--; if (itemSubMenuFuncao < 0) itemSubMenuFuncao = 3; }
    else if (botao == 2) { itemSubMenuFuncao++; if (itemSubMenuFuncao > 3) itemSubMenuFuncao = 0; }
    else if (botao == 3) {
      if (itemSubMenuFuncao == 0) {
        subMenuFuncao = SUBFUNCAO_PAPEIS;
        if      (papelConfiguradoMenu == PAPEL_CONFIG_ATACANTE) itemSubMenuFuncao = 0;
        else if (papelConfiguradoMenu == PAPEL_CONFIG_DEFENSOR) itemSubMenuFuncao = 1;
        else                                                     itemSubMenuFuncao = 2;
      } else if (itemSubMenuFuncao == 1) {
        subMenuFuncao = SUBFUNCAO_SENSORES; solicitarSensoresBrutosPe(true);
      } else if (itemSubMenuFuncao == 2) {
        subMenuFuncao = SUBFUNCAO_KICKER; itemSubMenuFuncao = 0;
      } else {
        estadoAtual = MENU; itemSelecionado = 1;
      }
    }
    return;
  }

  // --- FUNCAO > PAPEIS ---
  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_PAPEIS) {
    if      (botao == 1) { itemSubMenuFuncao--; if (itemSubMenuFuncao < 0) itemSubMenuFuncao = 3; }
    else if (botao == 2) { itemSubMenuFuncao++; if (itemSubMenuFuncao > 3) itemSubMenuFuncao = 0; }
    else if (botao == 3) {
      if (itemSubMenuFuncao == 0) {
        papelConfiguradoMenu = PAPEL_CONFIG_ATACANTE; aplicarPapelConfiguradoLocal(); enviarPapelAtualParaCabeca();
        mensagemBotao = salvarPapelConfiguradoEEPROM() ? "PAPEL FIXO ATC" : "ERRO EEPROM";
      } else if (itemSubMenuFuncao == 1) {
        papelConfiguradoMenu = PAPEL_CONFIG_DEFENSOR; aplicarPapelConfiguradoLocal(); enviarPapelAtualParaCabeca();
        mensagemBotao = salvarPapelConfiguradoEEPROM() ? "PAPEL FIXO DEF" : "ERRO EEPROM";
      } else if (itemSubMenuFuncao == 2) {
        papelConfiguradoMenu = PAPEL_CONFIG_AUTO;
        mensagemBotao = salvarPapelConfiguradoEEPROM() ? "PAPEL AUTO" : "ERRO EEPROM";
      }
      subMenuFuncao = SUBFUNCAO_PRINCIPAL; itemSubMenuFuncao = 0;
    }
    return;
  }

  // --- FUNCAO > SENSORES ---
  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_SENSORES) {
    if (botao == 1 || botao == 2) { solicitarSensoresBrutosPe(true); }
    else if (botao == 3) {
      subMenuFuncao = SUBFUNCAO_LIMIAR_LINHA;
      limiarLinhaSincronizado = false;
      entradaTelaLimiarMs = millis();
      solicitarLimiarLinhaPe(true);
    }
    return;
  }

  // --- FUNCAO > LIMIAR DE LINHA ---
  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_LIMIAR_LINHA) {
    if      (botao == 1) { limiarLinhaEditado += LIMIAR_LINHA_PASSO; if (limiarLinhaEditado > LIMIAR_LINHA_MAX) limiarLinhaEditado = LIMIAR_LINHA_MAX; limiarLinhaSincronizado = true; }
    else if (botao == 2) { limiarLinhaEditado -= LIMIAR_LINHA_PASSO; if (limiarLinhaEditado < LIMIAR_LINHA_MIN) limiarLinhaEditado = LIMIAR_LINHA_MIN; limiarLinhaSincronizado = true; }
    else if (botao == 3) { enviarLimiarLinhaParaPe(); mensagemBotao = "LIMIAR SALVO"; subMenuFuncao = SUBFUNCAO_PRINCIPAL; itemSubMenuFuncao = 1; }
    return;
  }

  // --- FUNCAO > KICKER ---
  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_KICKER) {
    if (botao == 1 || botao == 2) { itemSubMenuFuncao = 1 - itemSubMenuFuncao; }
    else if (botao == 3) {
      if (itemSubMenuFuncao == 0) { solicitarChuteManualkicker(); mensagemBotao = "CHUTE MANUAL"; }
      else                         { subMenuFuncao = SUBFUNCAO_PRINCIPAL; itemSubMenuFuncao = 2; }
    }
    return;
  }

  // --- CALIBRACAO > PRINCIPAL ---
  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_PRINCIPAL) {
    if      (botao == 1) { itemSubMenu--; if (itemSubMenu < 0) itemSubMenu = 6; }
    else if (botao == 2) { itemSubMenu++; if (itemSubMenu > 6) itemSubMenu = 0; }
    else if (botao == 3) {
      if      (itemSubMenu == 0) { subMenuCalibracao = SUBMENU_GOL;     itemSubMenu = corGolAzul ? 1 : 0; }
      else if (itemSubMenu == 1) { subMenuCalibracao = SUBMENU_BUSSOLA; itemSubMenu = 0; }
      else if (itemSubMenu == 2) { subMenuCalibracao = SUBMENU_IR;      itemSubMenu = 0; }
      else if (itemSubMenu == 3) { subMenuCalibracao = SUBMENU_ULTRA;   itemSubMenu = 0; }
      else if (itemSubMenu == 4) { subMenuCalibracao = SUBMENU_CAMERA;  itemSubMenu = 0; }
      else if (itemSubMenu == 5) { subMenuCalibracao = SUBMENU_ESPNOW;  itemSubMenu = 0; }
      else                        { estadoAtual = MENU; itemSelecionado = 0; }
    }
    return;
  }

  // --- CALIBRACAO > GOL: seleciona a cor e marca envio pendente ---
  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_GOL) {
    if      (botao == 1) { itemSubMenu--; if (itemSubMenu < 0) itemSubMenu = 2; }
    else if (botao == 2) { itemSubMenu++; if (itemSubMenu > 2) itemSubMenu = 0; }
    else if (botao == 3) {
      if (itemSubMenu == 0 || itemSubMenu == 1) {
        corGolAzul = (itemSubMenu == 1);
        corGolPendenteEnvio = true;
        bool ok = salvarCorGolEEPROM();
        mensagemBotao = ok ? (corGolAzul ? "GOL AZUL SALVO" : "GOL AMARELO SALVO") : "ERRO EEPROM";
      }
      subMenuCalibracao = SUBMENU_PRINCIPAL; itemSubMenu = 0;
    }
    return;
  }

  // --- CALIBRACAO > BUSSOLA: BTN3 salva referência de heading ---
  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_BUSSOLA) {
    if (botao == 3) {
      headingBussolaSalvo = headingBussolaTeste;
      EEPROM.put(EEPROM_ADDR_BUSSOLA, headingBussolaSalvo);
      bool ok = EEPROM.commit();
      mensagemBotao = ok ? "BUSSOLA GRAVADA" : "ERRO EEPROM";
      subMenuCalibracao = SUBMENU_PRINCIPAL; itemSubMenu = 0;
    } else if (botao == 1 || botao == 2) {
      subMenuCalibracao = SUBMENU_PRINCIPAL; itemSubMenu = 1;
    }
    return;
  }

  // --- CALIBRACAO > IR, ULTRA, CAMERA, ESPNOW: qualquer botão volta ---
  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_IR) {
    if (botao == 1 || botao == 2 || botao == 3) { subMenuCalibracao = SUBMENU_PRINCIPAL; itemSubMenu = 2; }
    return;
  }
  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_ULTRA) {
    if (botao == 1 || botao == 2 || botao == 3) { subMenuCalibracao = SUBMENU_PRINCIPAL; itemSubMenu = 3; }
    return;
  }
  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_CAMERA) {
    if (botao == 1 || botao == 2 || botao == 3) { subMenuCalibracao = SUBMENU_PRINCIPAL; itemSubMenu = 4; }
    return;
  }
  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_ESPNOW) {
    if (botao == 1 || botao == 2 || botao == 3) { subMenuCalibracao = SUBMENU_PRINCIPAL; itemSubMenu = 5; }
    return;
  }

  // --- INICIAR: BTN3 volta ao menu e para o jogo ---
  if (estadoAtual == INICIAR && botao == 3) {
    estadoAtual = MENU;
    enviarEstadoJogoParaCabeca(true);
  }
}


// =============================================================================
// SECAO 18 — PROCESSAMENTO DE MENSAGENS DA CABECA (Serial1)
// =============================================================================

// Interpreta mensagens recebidas da Cabeça e atualiza estados locais
void processarMensagemCabeca(String msg) {
  // Normaliza o frame para simplificar comparações de protocolo
  msg.trim();
  msg.toUpperCase();

  // Handshake: confirma que a serial está viva
  if (msg == "OI") {
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Status agregado das placas auxiliares (Olho e Pé)
  if (msg.startsWith("STS:")) {
    int separador = msg.indexOf(',');
    if (separador > 4) {
      String olho = msg.substring(4, separador);
      String pe   = msg.substring(separador + 1);
      olho.trim(); pe.trim();
      olhoOK = (olho == "1"); peOK = (pe == "1");
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
      mostrarStatusAte = millis() + 3000;
    }
    return;
  }

  // Eventos de botão: BTN:1, BTN:2, BTN:3 ou BTN:23
  if (msg.startsWith("BTN:")) {
    String valor = msg.substring(4); valor.trim();
    if (valor == "1" || valor == "2" || valor == "3" || valor == "23") {
      mensagemBotao = "BOTAO " + valor + " APERTADO";
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
      processarEventoBotao((uint8_t)valor.toInt());
    }
    return;
  }

  // Ângulo IR: negativo = sem bola; 30° exato é descartado como ruído
  if (msg.startsWith("IR:")) {
    String valorIr = msg.substring(3); valorIr.trim();
    float novoAngulo = valorIr.toFloat();
    if (novoAngulo < 0.0f) {
      irDetectado = false; anguloIr = -1.0f;
    } else {
      bool eh30Graus = fabsf(novoAngulo - 30.0f) <= 1.0f;
      if (eh30Graus) {
        // Descarta imediatamente como sem sinal de bola
        irDetectado = false; anguloIr = -1.0f;
      } else {
        irDetectado = true; anguloIr = novoAngulo;
        ultimoAnguloIrValido = normalizarAngulo360(novoAngulo);
        ultimoIrValidoMs = millis();
      }
    }
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Confirmação de configuração de cor do gol
  if (msg == "CFG:GOL:OK") {
    corGolPendenteEnvio = false;
    mensagemBotao = corGolAzul ? "GOL AZUL OK" : "GOL AMARELO OK";
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Leitura da bússola: normaliza e converte para inteiro [0, 359]
  if (msg.startsWith("BUS:")) {
    String valorBus = msg.substring(4); valorBus.trim();
    if (!payloadNumericoValido(valorBus)) { bussolaValida = false; return; }
    float angBus = valorBus.toFloat();
    angBus = normalizarAngulo360(angBus);
    headingBussolaTeste = (int)(angBus + 0.5f);
    if (headingBussolaTeste >= 360) headingBussolaTeste = 0;
    bussolaValida = true; ultimoRxBussolaMs = millis();
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Telemetria de gol: erro angular, flag de detecção, pixels e estado da câmera
  if (msg.startsWith("GOL:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(','), p2 = payload.indexOf(',', p1+1), p3 = payload.indexOf(',', p2+1);
    if (p1 > 0 && p2 > p1) {
      String sErro = payload.substring(0, p1);
      String sDet  = payload.substring(p1+1, p2);
      String sPix  = payload.substring(p2+1, (p3 > p2) ? p3 : payload.length());
      sErro.trim(); sDet.trim(); sPix.trim();
      erroGolGraus = sErro.toFloat();
      golDetectado = (sDet == "1");
      int pix = sPix.toInt();
      if (pix < 0) pix = 0; if (pix > 65535) pix = 65535;
      golPixels = (uint16_t)pix;
      if (p3 > p2) { String sCam = payload.substring(p3+1); sCam.trim(); cameraOK = (sCam == "1"); }
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Linha atacante: ângulo único (negativo = sem leitura)
  if (msg.startsWith("LIN:")) {
    String sLinha = msg.substring(4); sLinha.trim();
    float novoAng = sLinha.toFloat();
    linhaDetectada = (novoAng >= 0.0f); anguloLinhaPe = novoAng;
    ultimoRxLinhaMs = millis();
    if (linhaDetectada) { ultimoAnguloLinhaValido = anguloLinhaPe; ultimoComandoLinhaMs = ultimoRxLinhaMs; }
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Linha defensor — Zona A (0°–180°)
  if (msg.startsWith("LINA:")) {
    String sLinhaA = msg.substring(5); sLinhaA.trim();
    float novoAngA = sLinhaA.toFloat();
    linhaZonaAValida = (novoAngA >= 0.0f); anguloLinhaZonaA = linhaZonaAValida ? novoAngA : -1.0f;
    ultimoRxLinhaMs = millis();
    if (linhaZonaAValida) { ultimoAnguloLinhaZonaAValido = anguloLinhaZonaA; ultimoRxLinhaZonaAMs = ultimoRxLinhaMs; }
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Linha defensor — Zona B (180°–360°)
  if (msg.startsWith("LINB:")) {
    String sLinhaB = msg.substring(5); sLinhaB.trim();
    float novoAngB = sLinhaB.toFloat();
    linhaZonaBValida = (novoAngB >= 0.0f); anguloLinhaZonaB = linhaZonaBValida ? novoAngB : -1.0f;
    ultimoRxLinhaMs = millis();
    if (linhaZonaBValida) { ultimoAnguloLinhaZonaBValido = anguloLinhaZonaB; ultimoRxLinhaZonaBMs = ultimoRxLinhaMs; }
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Ultrassônicos locais: ULT:D,E,F,T
  if (msg.startsWith("ULT:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(','), p2 = payload.indexOf(',', p1+1), p3 = payload.indexOf(',', p2+1);
    if (p1 > 0 && p2 > p1 && p3 > p2) {
      String sD = payload.substring(0, p1), sE = payload.substring(p1+1, p2);
      String sF = payload.substring(p2+1, p3), sT = payload.substring(p3+1);
      sD.trim(); sE.trim(); sF.trim(); sT.trim();
      ultraDcm = sD.toFloat(); ultraEcm = sE.toFloat();
      ultraFcm = sF.toFloat(); ultraTcm = sT.toFloat();
      ultrasValidos = (ultraDcm >= 0.0f && ultraEcm >= 0.0f && ultraFcm >= 0.0f && ultraTcm >= 0.0f);
      ultimoRxUltraMs = millis();
      atualizarPapelAutomaticoPorParceria();
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Ultrassônicos remotos (do outro robô via ESP-NOW): ULR:D,E,F,T
  if (msg.startsWith("ULR:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(','), p2 = payload.indexOf(',', p1+1), p3 = payload.indexOf(',', p2+1);
    if (p1 > 0 && p2 > p1 && p3 > p2) {
      String sD = payload.substring(0, p1), sE = payload.substring(p1+1, p2);
      String sF = payload.substring(p2+1, p3), sT = payload.substring(p3+1);
      sD.trim(); sE.trim(); sF.trim(); sT.trim();
      ultraRemotoDcm = sD.toFloat(); ultraRemotoEcm = sE.toFloat();
      ultraRemotoFcm = sF.toFloat(); ultraRemotoTcm = sT.toFloat();
      ultrasRemotosValidos = (ultraRemotoDcm >= 0.0f && ultraRemotoEcm >= 0.0f &&
                              ultraRemotoFcm >= 0.0f && ultraRemotoTcm >= 0.0f);
      ultimoRxUltraRemotoMs = millis();
      atualizarPapelAutomaticoPorParceria();
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Estado da chave do kicker: KIK:0 = acionada, KIK:1 = não acionada
  if (msg.startsWith("KIK:")) {
    String sKik = msg.substring(4); sKik.trim();
    if (sKik == "0" || sKik == "1") {
      kickerAtivado = (sKik == "0"); kickerRecebido = true;
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Papel definido pela Cabeça: ATC:1 = atacante, ATC:0 = defensor
  if (msg.startsWith("ATC:")) {
    String sAtc = msg.substring(4); sAtc.trim();
    if (sAtc == "0" || sAtc == "1") {
      bool novoValor = (sAtc == "1");
      if (papelConfiguradoMenu == PAPEL_CONFIG_AUTO) {
        if (novoValor != papelAtacante) {
          Serial0.print("ATCFB:"); Serial0.println(novoValor ? 1 : 0);
        }
        papelAtacante = novoValor; atualizarPapelAutomaticoPorParceria();
      } else {
        aplicarPapelConfiguradoLocal();
        if (novoValor != papelAtacante) { Serial0.print("ATCFB:"); Serial0.println(papelAtacante ? 1 : 0); }
      }
      papelAtacanteAnterior = papelAtacante;
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Dados de câmera: CAM:ballAngle,ballDist,blueAngle,blueDist,yellowAngle,yellowDist,cameraOK
  if (msg.startsWith("CAM:")) {
    String payload = msg.substring(4);
    int p1=payload.indexOf(','), p2=payload.indexOf(',',p1+1), p3=payload.indexOf(',',p2+1);
    int p4=payload.indexOf(',',p3+1), p5=payload.indexOf(',',p4+1), p6=payload.indexOf(',',p5+1);
    if (p1>0 && p2>p1 && p3>p2 && p4>p3 && p5>p4 && p6>p5) {
      String sBallA=payload.substring(0,p1),      sBallD=payload.substring(p1+1,p2);
      String sBlueA=payload.substring(p2+1,p3),   sBlueD=payload.substring(p3+1,p4);
      String sYellA=payload.substring(p4+1,p5),   sYellD=payload.substring(p5+1,p6);
      String sCamOK=payload.substring(p6+1);
      sBallA.trim(); sBallD.trim(); sBlueA.trim(); sBlueD.trim(); sYellA.trim(); sYellD.trim(); sCamOK.trim();
      cameraBallAngle = (int16_t)sBallA.toInt();
      cameraBallDist  = (uint16_t)sBallD.toInt();
      unsigned long agoraCameraMs = millis();
      bool cameraBolaBrutaValida = (cameraBallAngle != -999) && (cameraBallDist > 0) &&
                                   !((cameraBallAngle == 0) && (cameraBallDist == 0));
      if (cameraBolaBrutaValida) {
        cameraUltimaVezBolaDetetadaMs = agoraCameraMs;
        cameraSemBolaBrutaInicioMs = 0;
        adicionarLeituraCameraNoBuffer((float)cameraBallAngle);
      } else if (cameraSemBolaBrutaInicioMs == 0) {
        cameraSemBolaBrutaInicioMs = agoraCameraMs;
      }
      cameraBlueAngle   = (int16_t)sBlueA.toInt();
      cameraBlueDist    = (uint16_t)sBlueD.toInt();
      cameraYellowAngle = (int16_t)sYellA.toInt();
      cameraYellowDist  = (uint16_t)sYellD.toInt();
      cameraDadosValidos = (sCamOK == "1");
      ultimoRxCameraMs  = millis();
      atualizarValidadeCamera();
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Sensores brutos do Pé: SENS:S1,S9,S17,S25
  if (msg.startsWith("SENS:")) {
    String payload = msg.substring(5);
    int p1=payload.indexOf(','), p2=payload.indexOf(',',p1+1), p3=payload.indexOf(',',p2+1);
    if (p1>0 && p2>p1 && p3>p2) {
      String s1=payload.substring(0,p1), s9=payload.substring(p1+1,p2);
      String s17=payload.substring(p2+1,p3), s25=payload.substring(p3+1);
      s1.trim(); s9.trim(); s17.trim(); s25.trim();
      sensorBruto1 = s1.toInt(); sensorBruto9 = s9.toInt();
      sensorBruto17 = s17.toInt(); sensorBruto25 = s25.toInt();
      ultimoRxSensoresBrutosMs = millis();
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Limiar de linha recebido do Pé: LIM:valor
  if (msg.startsWith("LIM:")) {
    String valor = msg.substring(4); valor.trim();
    int recebido = valor.toInt();
    if (recebido >= LIMIAR_LINHA_MIN && recebido <= LIMIAR_LINHA_MAX) {
      limiarLinhaEditado = recebido; limiarLinhaSincronizado = true;
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Status do ESP-NOW entre as Cabeças: ESN:1 = conectado
  if (msg.startsWith("ESN:")) {
    String sEsn = msg.substring(4); sEsn.trim();
    espnowOK = (sEsn == "1"); ultimoRxEspnowMs = millis();
    atualizarSozinhoLocal(); atualizarPapelAutomaticoPorParceria();
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }
}

// Acumula bytes da Serial1 e despacha mensagens completas por linha
void lerSerialCabeca() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    // Protocolo orientado a linha: \r e \n encerram o frame atual
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        processarMensagemCabeca(bufferSerial);
        bufferSerial = "";
      }
      continue;
    }
    // Limita o buffer para evitar crescimento indefinido com ruído serial
    if (bufferSerial.length() < 32) {
      bufferSerial += c;
    }
  }
}


// =============================================================================
// SECAO 19 — ESTRATEGIA DO ATACANTE
// =============================================================================
//
// Responsabilidades:
//   • Alinhar o robô ao gol via câmera (PID de bússola)
//   • Seguir a bola por IR com rampa angular suave
//   • Usar câmera como fallback quando IR está ausente
//   • Fugir da linha branca com prioridade máxima
//   • Frear ao se aproximar de paredes (ultrassônicos)
//   • Chutar ao se alinhar (kicker automático via atualizarKicker)
//
// Parâmetros de ajuste — *** ALTERE APENAS AQUI para tunar o atacante ***
// =============================================================================

// --- Velocidades do atacante ---
const int VELOCIDADE_IR_FRONTAL_PWM       = 200;   // PWM na faixa frontal do IR (±32°)
const int VELOCIDADE_IR_FAIXA_REDUZIDA_PWM = 140;  // PWM em faixas laterais do IR

// --- Freio ultrassônico do atacante (laterais) ---
const float ATACANTE_ULTRA_FREIO_INICIO_CM    = 55.0f;   // Distância onde o freio começa
const float ATACANTE_ULTRA_FREIO_CRITICO_CM   = 35.0f;   // Distância de freio máximo
const int   ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN = 125;   // Velocidade mínima com freio
const int   ATACANTE_ULTRA_FREIO_PWM_POR_CM   = 3;       // Incremento de PWM por cm

// --- Buffer de perda de sinal IR ---
const unsigned long IR_BUFFER_PERDA_MS = 160; // Tempo que o último ângulo IR é mantido após perda

// --- Confirmação de linha + parede (evita falso positivo único) ---
const uint8_t ATACANTE_LINHA_PAREDE_CONFIRMACAO = 3;

// --- Tempo mínimo sem bola na câmera para iniciar busca ---
const unsigned long ATACANTE_ESPERA_SEM_BOLA_CAMERA_MS = 2000UL;

// --- PID de suavização angular do atacante (transição entre ângulos de movimento) ---
// *** AJUSTE AQUI para suavizar ou tornar mais responsiva a transição de direção ***
const float PID_MOVIMENTO_KP           = 2.0f;
const float PID_MOVIMENTO_KI           = 0.01f;
const float PID_MOVIMENTO_KD           = 0.8f;
const float PID_MOVIMENTO_INTEGRAL_MAX = 90.0f;
const float PID_MOVIMENTO_SAIDA_MAX    = 15.0f;
const float ALPHA_MOVIMENTO            = 0.15f;   // Suavização exponencial do ângulo atual
const float ALPHA_MOVIMENTO_ALVO       = 0.11f;   // Suavização exponencial do ângulo alvo
const float PASSO_MAX_MOVIMENTO_ALVO_GRAUS = 18.0f; // Passo máximo por ciclo no alvo filtrado

// Estado interno do PID de movimento do atacante
float        pidMovimentoIntegral              = 0.0f;
float        pidMovimento                      = 0.0f;
float        erroMovimento                     = 0.0f;
float        erroAnteriorMovimento             = 0.0f;
float        anguloMovimentoAtual              = -1.0f;
float        anguloMovimentoSuavizado          = -1.0f;
float        anguloMovimentoDesejadoFiltrado   = -1.0f;
unsigned long ultimoTempoPidMovimento          = 0;
unsigned long inicioCameraSemIrMs              = 0;

// Zera o controlador angular do atacante
void resetControleMovimentoAtacante() {
  pidMovimentoIntegral            = 0.0f;
  pidMovimento                    = 0.0f;
  erroMovimento                   = 0.0f;
  erroAnteriorMovimento           = 0.0f;
  anguloMovimentoAtual            = -1.0f;
  anguloMovimentoSuavizado        = -1.0f;
  anguloMovimentoDesejadoFiltrado = -1.0f;
  ultimoTempoPidMovimento         = 0;
}

// PID de transição angular do atacante (suaviza ângulo de movimento)
float calcularPidMovimento(float erro) {
  unsigned long agora = millis();
  float dt = 0.02f;
  if (ultimoTempoPidMovimento != 0) {
    dt = (agora - ultimoTempoPidMovimento) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f)   dt = 0.2f;
  }
  ultimoTempoPidMovimento = agora;

  pidMovimentoIntegral += erro * dt;
  if (pidMovimentoIntegral >  PID_MOVIMENTO_INTEGRAL_MAX) pidMovimentoIntegral =  PID_MOVIMENTO_INTEGRAL_MAX;
  if (pidMovimentoIntegral < -PID_MOVIMENTO_INTEGRAL_MAX) pidMovimentoIntegral = -PID_MOVIMENTO_INTEGRAL_MAX;

  float derivada = (erro - erroAnteriorMovimento) / dt;
  erroAnteriorMovimento = erro;

  pidMovimento = PID_MOVIMENTO_KP * erro + PID_MOVIMENTO_KI * pidMovimentoIntegral + PID_MOVIMENTO_KD * derivada;
  if (pidMovimento >  PID_MOVIMENTO_SAIDA_MAX) pidMovimento =  PID_MOVIMENTO_SAIDA_MAX;
  if (pidMovimento < -PID_MOVIMENTO_SAIDA_MAX) pidMovimento = -PID_MOVIMENTO_SAIDA_MAX;

  // Evita microcorreções perto do alvo
  if (fabsf(erro) < 2.0f) pidMovimento = 0.0f;

  return -pidMovimento;
}

// Aplica suavização exponencial circular ao ângulo de movimento do atacante
float suavizarAnguloMovimentoAtacante(float anguloMovimentoDesejado) {
  // Referencial deslocado 180°: o que era 0 passa a ser 180 e vice-versa
  float alvoBruto = normalizarAngulo360(anguloMovimentoDesejado + 180.0f);

  if ((anguloMovimentoAtual < 0.0f) || (anguloMovimentoSuavizado < 0.0f)) {
    anguloMovimentoAtual            = alvoBruto;
    anguloMovimentoSuavizado        = alvoBruto;
    anguloMovimentoDesejadoFiltrado = alvoBruto;
    erroMovimento = erroAnteriorMovimento = pidMovimentoIntegral = pidMovimento = 0.0f;
    ultimoTempoPidMovimento = 0;
    return anguloMovimentoSuavizado;
  }

  // Estabiliza o alvo em modo circular para evitar saltos (ex.: 90° para 270°)
  float deltaAlvo = normalizarErro180(alvoBruto - anguloMovimentoDesejadoFiltrado);
  deltaAlvo = constrain(deltaAlvo, -PASSO_MAX_MOVIMENTO_ALVO_GRAUS, PASSO_MAX_MOVIMENTO_ALVO_GRAUS);
  anguloMovimentoDesejadoFiltrado = normalizarAngulo360(anguloMovimentoDesejadoFiltrado + deltaAlvo);
  anguloMovimentoDesejadoFiltrado = normalizarAngulo360(
      anguloMovimentoDesejadoFiltrado +
      ALPHA_MOVIMENTO_ALVO * normalizarErro180(alvoBruto - anguloMovimentoDesejadoFiltrado));

  // Erro: última direção realmente comandada como realimentação
  erroMovimento = normalizarErro180(anguloMovimentoDesejadoFiltrado - anguloMovimentoAtual);
  pidMovimento  = calcularPidMovimento(erroMovimento);

  anguloMovimentoAtual = normalizarAngulo360(anguloMovimentoAtual + pidMovimento);
  anguloMovimentoSuavizado = anguloMovimentoSuavizado +
                             ALPHA_MOVIMENTO * normalizarErro180(anguloMovimentoAtual - anguloMovimentoSuavizado);
  anguloMovimentoSuavizado = normalizarAngulo360(anguloMovimentoSuavizado);
  return anguloMovimentoSuavizado;
}

// Faz rampa angular curta (100–300 ms) entre faixas do IR
float obterAnguloIrSuavizado(float anguloAlvoGraus) {
  float alvo  = normalizarAngulo360(anguloAlvoGraus);
  unsigned long agora = millis();

  if (!anguloIrSuaveInicializado) {
    anguloIrSuaveAtual = anguloIrSuaveInicio = anguloIrSuaveAlvo = alvo;
    inicioTransicaoIrMs = agora; duracaoTransicaoIrMs = TRANSICAO_ANGULO_IR_MIN_MS;
    anguloIrSuaveInicializado = true;
    return quantizarAnguloPasso(anguloIrSuaveAtual, PASSO_ANGULO_IR_GRAUS);
  }

  float erroNovoAlvo = fabsf(normalizarErro180(alvo - anguloIrSuaveAlvo));
  if (erroNovoAlvo >= 1.0f) {
    anguloIrSuaveInicio = anguloIrSuaveAtual;
    anguloIrSuaveAlvo   = alvo;
    inicioTransicaoIrMs = agora;
    float delta             = fabsf(normalizarErro180(anguloIrSuaveAlvo - anguloIrSuaveInicio));
    unsigned long duracaoCalculada = (unsigned long)(delta * TRANSICAO_ANGULO_IR_MS_POR_GRAU);
    if (duracaoCalculada < TRANSICAO_ANGULO_IR_MIN_MS) duracaoCalculada = TRANSICAO_ANGULO_IR_MIN_MS;
    if (duracaoCalculada > TRANSICAO_ANGULO_IR_MAX_MS) duracaoCalculada = TRANSICAO_ANGULO_IR_MAX_MS;
    duracaoTransicaoIrMs = duracaoCalculada;
  }

  unsigned long decorridoMs = agora - inicioTransicaoIrMs;
  if (decorridoMs >= duracaoTransicaoIrMs) {
    anguloIrSuaveAtual = anguloIrSuaveAlvo;
  } else {
    float progresso = (float)decorridoMs / (float)duracaoTransicaoIrMs;
    float delta     = normalizarErro180(anguloIrSuaveAlvo - anguloIrSuaveInicio);
    anguloIrSuaveAtual = normalizarAngulo360(anguloIrSuaveInicio + delta * progresso);
  }

  return quantizarAnguloPasso(anguloIrSuaveAtual, PASSO_ANGULO_IR_GRAUS);
}

// Detecta faixa frontal do IR em torno de 0° (±32°), tratando wrap 360°->0°
bool irNaFaixaFrontal(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);
  return (ang > 328.0f || ang < 32.0f);
}

// Retorna true e ângulo do IR (com buffer de retenção após perda de sinal)
bool obterAnguloIrComBuffer(float &anguloIrSaida) {
  if (irDetectado && anguloIr >= 0.0f) {
    anguloIrSaida = normalizarAngulo360(anguloIr); return true;
  }
  if ((ultimoAnguloIrValido >= 0.0f) && (ultimoIrValidoMs > 0) &&
      ((millis() - ultimoIrValidoMs) <= IR_BUFFER_PERDA_MS)) {
    anguloIrSaida = normalizarAngulo360(ultimoAnguloIrValido); return true;
  }
  return false;
}

// Retorna true se algum ultrassônico lateral do atacante está em nível crítico
bool ultraLateralCriticoAtacante() {
  bool ultraDireitoCritico  = (ultraDcm >= 0.0f) && (ultraDcm <= ATACANTE_ULTRA_FREIO_CRITICO_CM);
  bool ultraEsquerdoCritico = (ultraEcm >= 0.0f) && (ultraEcm <= ATACANTE_ULTRA_FREIO_CRITICO_CM);
  return ultraDireitoCritico || ultraEsquerdoCritico;
}

// Limita velocidade por freio ultrassônico frontal (frente do robô)
int aplicarFreioUltrassonicoAtacanteFrente(int velocidadeDesejada) {
  int velocidadeBase = constrain(velocidadeDesejada, 0, 255);
  bool ultrasRecentes = (ultimoRxUltraMs > 0) && ((millis() - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);
  if (!ultrasRecentes) return velocidadeBase;

  float menorUltraCm = -1.0f;
  // Usa apenas ultraF para o freio frontal; ultraT é ignorado se estiver próximo
  float leituras[] = { ultraFcm };
  for (float leitura : leituras) {
    if (leitura < 0.0f) continue;
    if (ultraTcm > 150) {
      if ((menorUltraCm < 0.0f) || (leitura < menorUltraCm)) menorUltraCm = leitura;
    }
    if ((menorUltraCm < 0.0f) || (menorUltraCm > ATACANTE_ULTRA_FREIO_INICIO_CM)) return velocidadeBase;
    int velocidadeLimite = ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;
    if (menorUltraCm > ATACANTE_ULTRA_FREIO_CRITICO_CM)
      velocidadeLimite += (int)((menorUltraCm - ATACANTE_ULTRA_FREIO_CRITICO_CM) * ATACANTE_ULTRA_FREIO_PWM_POR_CM);
    if (velocidadeLimite > velocidade_maxima) velocidadeLimite = velocidade_maxima;
    return min(velocidadeBase, velocidadeLimite);
  }
  return velocidadeBase;
}

// Limita velocidade por freio ultrassônico lateral do atacante (D e E)
int aplicarFreioUltrassonicoAtacante(int velocidadeDesejada) {
  int velocidadeBase = constrain(velocidadeDesejada, 0, 255);
  bool ultrasRecentes = (ultimoRxUltraMs > 0) && ((millis() - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);
  if (!ultrasRecentes) return velocidadeBase;

  float menorUltraCm = -1.0f;
  float leituras[] = { ultraDcm, ultraEcm };
  for (float leitura : leituras) {
    if (leitura < 0.0f) continue;
    if ((menorUltraCm < 0.0f) || (leitura < menorUltraCm)) menorUltraCm = leitura;
  }
  if ((menorUltraCm < 0.0f) || (menorUltraCm > ATACANTE_ULTRA_FREIO_INICIO_CM)) return velocidadeBase;

  int velocidadeLimite = ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;
  if (menorUltraCm > ATACANTE_ULTRA_FREIO_CRITICO_CM)
    velocidadeLimite += (int)((menorUltraCm - ATACANTE_ULTRA_FREIO_CRITICO_CM) * ATACANTE_ULTRA_FREIO_PWM_POR_CM);
  if (velocidadeLimite > velocidade_maxima) velocidadeLimite = velocidade_maxima;
  return min(velocidadeBase, velocidadeLimite);
}

// Ajusta o ângulo da bola para o ângulo de comando de movimento (mapeamento por faixas)
// *** ALTERE AQUI para modificar o comportamento de contorno da bola pelo atacante ***
float mapearAnguloBolaParaMovimento(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);

  if(ang >= 32.0f && ang <= 60.0f)   return 100.0f; // Faixa diagonal direita-frente
  if(ang >= 300.0f && ang <= 328.0f) return 260.0f; // Faixa diagonal esquerda-frente

  if(ang > 60.0f  && ang < 90.0f)   return 135.0f;
  if(ang >= 90.0f && ang < 135.0f)  return 180.0f;
  if(ang >= 270.0f && ang < 300.0f) return 225.0f;
  if(ang >= 225.0f && ang < 270.0f) return 180.0f;
  if(ang >= 180.0f && ang < 225.0f) return 135.0f;
  if(ang >= 135.0f && ang < 180.0f) return 225.0f;

  return ang;
}

// Reduz velocidade em faixas próximas do frontal para melhorar controle lateral
int calcularVelocidadeIrPorAngulo(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);
  if ((ang >= 33.0f && ang <= 60.0f) || (ang >= 300.0f && ang <= 328.0f)) return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;
  if (ang >= 140.0f && ang < 220.0f) return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;
  return velocidade_maxima;
}

// Calcula ângulo de busca quando a câmera perdeu a bola (usa ultrassônicos laterais)
float calcularAnguloBuscaSemBolaCameraAtacante() {
  bool ultrasRecentes = ultrasValidos && (ultimoRxUltraMs > 0) && ((millis() - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);
  if (ultrasRecentes) {
    bool esquerdaPerto = (ultraEcm >= 0.0f) && (ultraEcm < 60.0f);
    bool direitaPerto  = (ultraDcm >= 0.0f) && (ultraDcm < 60.0f);
    bool esquerdaLivre = ultraEcm > 50.0f;
    bool direitaLivre  = ultraDcm > 50.0f;
    if (esquerdaPerto && direitaLivre)  return  90.0f;
    if (direitaPerto  && esquerdaLivre) return 270.0f;
  }
  return 0.0f;
}

// *** FUNCAO PRINCIPAL DO ATACANTE ***
// Alinha ao gol, segue bola por IR/câmera, foge da linha e chuta quando alinhado
void atacante() {
  int16_t anguloGolCamera = -999;
  bool golVisivelCamera = cameraTemGolSelecionadoValido(anguloGolCamera);
  erroAlinhamentoGraus = golVisivelCamera ? calcularErroGolPorPapel((float)anguloGolCamera) : 0.0f;
  fugindoLinhaAgora    = false;
  anguloFugaLinhaCmd   = 0.0f;

  static uint8_t confirmacoesLinhaParede  = 0;
  static bool    forcarIrDiretoAposLinha  = false;

  int  cmdPidAssinado         = 0;
  bool cameraBolaBrutaVisivel = cameraTemBolaValida();
  float anguloCameraBolaFiltrado  = -1.0f;
  bool cameraBolaFiltradaVisivel  = obterAnguloCameraBolaFiltrado(anguloCameraBolaFiltrado);
  float anguloIrBufferizado        = -1.0f;
  bool irDisponivel               = obterAnguloIrComBuffer(anguloIrBufferizado);

  // Decide se precisa girar para alinhar ao gol
  bool precisaAlinhar = golVisivelCamera && (fabsf(erroAlinhamentoGraus) > TOLERANCIA_ALINHAMENTO_GRAUS);
  bool erroGrande     = golVisivelCamera && (fabsf(erroAlinhamentoGraus) > 40.0f);
  if (precisaAlinhar) {
    alinhandoAgora = true;
    int cmdPid    = calcularSaidaPidBussola(erroAlinhamentoGraus);
    cmdPidAssinado = -SINAL_GIRO_PID * cmdPid;
  } else {
    alinhandoAgora = false; resetPidBussola();
  }

  // Câmera assume o controle se o IR falhar por tempo suficiente
  if (!irDisponivel && cameraBolaFiltradaVisivel) {
    if (inicioCameraSemIrMs == 0) inicioCameraSemIrMs = millis();
  } else {
    inicioCameraSemIrMs = 0;
  }
  bool ignorarLinhaPorCamera = (inicioCameraSemIrMs != 0) &&
                               ((millis() - inicioCameraSemIrMs) >= TEMPO_CAMERA_SEM_IR_PARA_IGNORAR_LINHA_MS);

  // Mantém janela curta de fuga para evitar perda por oscilação de frame
  bool linhaRecenteForcada = (ultimoComandoLinhaMs > 0) &&
                             ((millis() - ultimoComandoLinhaMs) <= RETENCAO_FUGA_LINHA_MS);
  float anguloLinhaParaFuga = (linhaDetectada && anguloLinhaPe >= 0.0f) ? anguloLinhaPe : ultimoAnguloLinhaValido;

  bool linhaValida = ((linhaDetectada && (anguloLinhaPe >= 0.0f) && !ignorarLinhaPorCamera) ||
                      (linhaRecenteForcada && (anguloLinhaParaFuga >= 0.0f)));
/*
  // Verifica linha + parede crítica para forçar IR direto
  bool linhaComParedeCritica = linhaValida && ultraLateralCriticoAtacante();
  if (linhaComParedeCritica) {
    if (confirmacoesLinhaParede < ATACANTE_LINHA_PAREDE_CONFIRMACAO) confirmacoesLinhaParede++;
    if (confirmacoesLinhaParede >= ATACANTE_LINHA_PAREDE_CONFIRMACAO) forcarIrDiretoAposLinha = true;
  } else if (!linhaValida) {
    confirmacoesLinhaParede = 0; forcarIrDiretoAposLinha = false;
  }

  bool irDiretoAtivo = forcarIrDiretoAposLinha && irDisponivel;
*/
  // PRIORIDADE 1: linha + parede = IR direto (ignora linha)
  if (linhaValida) {
    if (irDiretoAtivo) {
      if (irNaFaixaFrontal(anguloIrBufferizado)) {
        if (ultraTcm > 150) moverFrenteComGiro(aplicarFreioUltrassonicoAtacanteFrente(VELOCIDADE_IR_FRONTAL_PWM), cmdPidAssinado);
        else                 moverFrenteComGiro(aplicarFreioUltrassonicoAtacante(VELOCIDADE_IR_FRONTAL_PWM), cmdPidAssinado);
      } else {
        float anguloIrAlvo           = mapearAnguloBolaParaMovimento(anguloIrBufferizado);
        float anguloIrComRampa       = obterAnguloIrSuavizado(anguloIrAlvo);
        int   velocidadeIr           = aplicarFreioUltrassonicoAtacante(calcularVelocidadeIrPorAngulo(anguloIrBufferizado));
        float anguloMovimentoComControle = suavizarAnguloMovimentoAtacante(anguloIrComRampa);
        seguirDirecaoPorAngulo(anguloMovimentoComControle, velocidadeIr);
      }
      return;
    }

    // PRIORIDADE 2: fuga de linha (sem parede crítica)
    fugindoLinhaAgora  = true;
    anguloFugaLinhaCmd = normalizarAngulo360(anguloLinhaParaFuga);
    float anguloFugaSuavizado = suavizarAnguloMovimentoAtacante(anguloFugaLinhaCmd);
    seguirDirecaoPorAngulo(anguloFugaSuavizado, aplicarFreioUltrassonicoAtacante(VELOCIDADE_FUGA_LINHA));

  // PRIORIDADE 3: erro grande de alinhamento = giro puro
  } else if (erroGrande) {
    girarNoEixo(cmdPidAssinado);

  // PRIORIDADE 4: IR disponível = segue bola pelo IR
  } else if (irDisponivel) {
    // Verifica linha antes de qualquer movimento de IR
    if (linhaDetectada && anguloLinhaPe >= 0.0f) {
      fugindoLinhaAgora  = true;
      anguloFugaLinhaCmd = normalizarAngulo360(anguloLinhaPe);
      float anguloFugaSuavizado = suavizarAnguloMovimentoAtacante(anguloFugaLinhaCmd);
      seguirDirecaoPorAngulo(anguloFugaSuavizado, aplicarFreioUltrassonicoAtacante(VELOCIDADE_FUGA_LINHA));
      return;
    }
    if (irNaFaixaFrontal(anguloIrBufferizado)) {
      if (ultraTcm > 150) moverFrenteComGiro(aplicarFreioUltrassonicoAtacanteFrente(VELOCIDADE_IR_FRONTAL_PWM), cmdPidAssinado);
      else                 moverFrenteComGiro(aplicarFreioUltrassonicoAtacante(VELOCIDADE_IR_FRONTAL_PWM), cmdPidAssinado);
    } else {
      float anguloIrAlvo           = normalizarAngulo360(mapearAnguloBolaParaMovimento(anguloIrBufferizado) + (erroAlinhamentoGraus * 2));
      float anguloIrComRampa       = obterAnguloIrSuavizado(anguloIrAlvo);
      int   velocidadeIr           = aplicarFreioUltrassonicoAtacante(calcularVelocidadeIrPorAngulo(anguloIrBufferizado));
      float anguloMovimentoComControle = suavizarAnguloMovimentoAtacante(anguloIrComRampa);
      seguirDirecaoPorAngulo(anguloMovimentoComControle, velocidadeIr);
    }

  // PRIORIDADE 5: câmera como fallback de bola
  } else if (cameraBolaFiltradaVisivel) {
    bool semBolaTempoSuficiente = !cameraBolaBrutaVisivel &&
                                  (cameraSemBolaBrutaInicioMs > 0) &&
                                  ((millis() - cameraSemBolaBrutaInicioMs) >= ATACANTE_ESPERA_SEM_BOLA_CAMERA_MS);
    float anguloCameraVetorial   = semBolaTempoSuficiente ? calcularAnguloBuscaSemBolaCameraAtacante() : anguloCameraBolaFiltrado;
    float anguloCameraComRampa   = obterAnguloIrSuavizado(anguloCameraVetorial);
    float anguloMovimentoComControle = suavizarAnguloMovimentoAtacante(anguloCameraComRampa);
    seguirDirecaoComGiro(anguloMovimentoComControle, aplicarFreioUltrassonicoAtacante(velocidade_maxima), cmdPidAssinado);

  // PRIORIDADE 6: apenas alinhamento (sem bola)
  } else if (precisaAlinhar) {
    girarNoEixo(cmdPidAssinado);

  // Sem informações: para
  } else {
    pararMotores(); delay(10);
  }
}


// =============================================================================
// SECAO 20 — ESTRATEGIA DO DEFENSOR (GOLEIRO)
// =============================================================================
//
// Responsabilidades:
//   • Manter posição na frente do gol usando linha como referência (zonas A e B)
//   • Acompanhar a bola lateralmente via IR / câmera
//   • Conter a bola com ultrassônicos laterais e de profundidade
//   • Retornar ao gol pela bússola quando sem linha
//   • Avançar sobre a bola quando ela fica frontal por tempo suficiente
//
// Parâmetros de ajuste — *** ALTERE APENAS AQUI para tunar o defensor ***
// =============================================================================

// --- Referências de zona (ângulos esperados das linhas A e B no referencial do defensor) ---
constexpr float DEFENSOR_REFERENCIA_ZONA_A = 90.0f;
constexpr float DEFENSOR_REFERENCIA_ZONA_B = 270.0f;

// --- Tolerâncias angulares do defensor ---
constexpr float DEFENSOR_TOLERANCIA_GIRO_GRAUS     = 5.0f;   // Deadzone de giro
constexpr float DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS = 3.0f;  // Alinhamento antes do avanço frontal

// --- Pesos de atração lateral por bola ---
constexpr float DEFENSOR_PESO_MIN_BOLA = 75.0f;
constexpr float DEFENSOR_PESO_MAX_BOLA = 200.0f;

// --- Limites dos ultrassônicos do defensor ---
constexpr float DEFENSOR_ULTRA_LATERAL_ATIVO_CM       = 65.0f;   // Distância onde começa a repulsão lateral
constexpr float DEFENSOR_ULTRA_LATERAL_CRITICO_CM     = 45.0f;   // Distância de repulsão máxima
constexpr float DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM = 60.0f;   // Confirma a parede oposta livre
constexpr float DEFENSOR_ULTRA_FRENTE_LIMITE_CM       = 40.0f;   // Limite para repulsão de profundidade (frente)
constexpr float DEFENSOR_ULTRA_TRAS_LIMITE_CM         = 35.0f;   // Limite para repulsão de profundidade (trás)
constexpr float DEFENSOR_ULTRA_FRONTAL_CONFIRMADOR_CM = 100.0f;  // Confirmador de campo frontal livre
constexpr float DEFENSOR_ULTRA_PROFUNDIDADE_MAX_CM    = 60.0f;
constexpr float DEFENSOR_ULTRA_PROFUNDIDADE_MIN_CM    = 10.0f;
constexpr float DEFENSOR_PESO_MAX_ULTRA               = 100.0f;
constexpr float DEFENSOR_PESO_MAX_ULTRA_PROFUNDIDADE  = 38.0f;

// --- Deadzones do defensor ---
constexpr float DEFENSOR_DEADZONE_VETOR = 6.0f;
constexpr float DEFENSOR_DEADZONE_GIRO  = 5.0f;

// --- Velocidades do defensor ---
constexpr float DEFENSOR_VELOCIDADE_MIN_PWM        = 145.0f;
constexpr float DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM = 165.0f;

// --- Suavização do defensor (fator exponencial) ---
constexpr float DEFENSOR_SUAVIZACAO_VETOR = 0.45f;
constexpr float DEFENSOR_SUAVIZACAO_GIRO  = 0.35f;

// --- Parâmetros do avanço frontal temporizado do defensor ---
const float         DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS      = 45.0f;
const int           DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM = 255;
const unsigned long DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS      = 3000;
const unsigned long DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS       = 2500;

// --- Controle proporcional lateral do defensor por IR ---
int calcularVelocidadeLateralDefensorPorIr(float anguloBolaGraus) {
  const float ANG_DIREITA_MIN  = 20.0f;
  const float ANG_DIREITA_MAX  = 160.0f;  /// 
  const float ANG_ESQUERDA_MIN = 200.0f;  /// 
  const float ANG_ESQUERDA_MAX = 340.0f;
  const int   VEL_MIN = 130;
  const int   VEL_MAX = 255;

  float ang = normalizarAngulo360(anguloBolaGraus);
  if ((ang >= 0.0f && ang <= ANG_DIREITA_MIN) || (ang >= ANG_ESQUERDA_MAX && ang <= 360.0f)) return 0;
  if (ang > ANG_DIREITA_MIN && ang <= ANG_DIREITA_MAX) {
    float ganho = (float)(VEL_MAX - VEL_MIN) / (ANG_DIREITA_MAX - ANG_DIREITA_MIN);
    return constrain((int)(VEL_MIN + ganho * (ang - ANG_DIREITA_MIN)), VEL_MIN, VEL_MAX);
  }
  if (ang >= ANG_ESQUERDA_MIN && ang < ANG_ESQUERDA_MAX) {
    float ganho = (float)(VEL_MAX - VEL_MIN) / (ANG_ESQUERDA_MAX - ANG_ESQUERDA_MIN);
    return constrain((int)(VEL_MIN + ganho * (ANG_ESQUERDA_MAX - ang)), VEL_MIN, VEL_MAX);
  }
  return 0;
}

// --- PID do giro do goleiro usando zonas A e B da linha ---
// *** AJUSTE AQUI para tunar o giro do defensor ***
const float PID_LINHA_GOL_KP           = 0.9f;
const float PID_LINHA_GOL_KI           = 0.01f;
const float PID_LINHA_GOL_KD           = 0.55f;
const float PID_LINHA_GOL_INTEGRAL_MAX = 90.0f;
const int   PID_LINHA_GOL_SAIDA_MIN    = 60;
const int   PID_LINHA_GOL_SAIDA_MAX    = 220;

// Estado interno do PID de linha do goleiro
float        pidLinhaGolIntegral      = 0.0f;
float        pidLinhaGolErroAnterior  = 0.0f;
unsigned long pidLinhaGolUltimoMs     = 0;

// Zera o PID do goleiro por linha
void resetPidLinhaGoleiro() {
  pidLinhaGolIntegral     = 0.0f;
  pidLinhaGolErroAnterior = 0.0f;
  pidLinhaGolUltimoMs     = 0;
}

// Calcula saída do PID do goleiro para o giro por linha
int calcularSaidaPidLinhaGoleiro(float erroGraus) {
  unsigned long agora = millis();
  float dt = 0.02f;
  if (pidLinhaGolUltimoMs != 0) {
    dt = (agora - pidLinhaGolUltimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f)   dt = 0.2f;
  }
  pidLinhaGolUltimoMs = agora;

  pidLinhaGolIntegral += erroGraus * dt;
  if (pidLinhaGolIntegral >  PID_LINHA_GOL_INTEGRAL_MAX) pidLinhaGolIntegral =  PID_LINHA_GOL_INTEGRAL_MAX;
  if (pidLinhaGolIntegral < -PID_LINHA_GOL_INTEGRAL_MAX) pidLinhaGolIntegral = -PID_LINHA_GOL_INTEGRAL_MAX;

  float derivada = (erroGraus - pidLinhaGolErroAnterior) / dt;
  pidLinhaGolErroAnterior = erroGraus;

  float u    = PID_LINHA_GOL_KP * erroGraus + PID_LINHA_GOL_KI * pidLinhaGolIntegral + PID_LINHA_GOL_KD * derivada;
  int   saida = (int)fabsf(u);
  if (saida < PID_LINHA_GOL_SAIDA_MIN)           saida = PID_LINHA_GOL_SAIDA_MIN;
  if (saida > PID_LINHA_GOL_SAIDA_MAX)           saida = PID_LINHA_GOL_SAIDA_MAX;
  if (saida > VELOCIDADE_GIRO_ALINHAMENTO)       saida = VELOCIDADE_GIRO_ALINHAMENTO;
  return (u >= 0.0f) ? saida : -saida;
}

// --- Funções auxiliares do defensor ---

// Aplica suavização exponencial a um valor do defensor
float suavizarDefensor(float atual, float alvo, float fator) {
  float fatorClamped = constrain(fator, 0.0f, 1.0f);
  return atual + ((alvo - atual) * fatorClamped);
}

// Aplica deadzone a um valor do defensor
float aplicarDeadzoneDefensor(float valor, float deadzone) {
  return (fabsf(valor) < deadzone) ? 0.0f : valor;
}

// Calcula a magnitude do vetor de movimento do defensor
float calcularMagnitudeVetorDefensor(float vetorX, float vetorY) {
  return sqrtf((vetorX * vetorX) + (vetorY * vetorY));
}

// Calcula o ângulo do vetor de movimento do defensor (0 = frente, 90 = direita)
float calcularAnguloVetorDefensor(float vetorX, float vetorY) {
  return normalizarAngulo360(atan2f(vetorX, vetorY) * 180.0f / PI);
}

// Retorna o sinal do erro angular do defensor com deadzone
int sinalErroDefensor(float erro, float toleranciaZero) {
  if (erro >  toleranciaZero) return  1;
  if (erro < -toleranciaZero) return -1;
  return 0;
}

// Calcula comando de giro do defensor com base no desequilíbrio entre zonas A e B
int calcularGiroDefensor(float erroA, float erroB, float toleranciaIgual, int giroMaximo) {
  float deltaMag = fabsf(erroA) - fabsf(erroB);
  if (fabsf(deltaMag) <= toleranciaIgual) return 0;
  float ganho  = 2.0f;
  int   cmdGiro = (int)(fabsf(deltaMag) * ganho);
  if (cmdGiro < 35)          cmdGiro = 35;
  if (cmdGiro > giroMaximo)  cmdGiro = giroMaximo;
  return (deltaMag >= 0.0f) ? cmdGiro : -cmdGiro;
}

// Calcula velocidade proporcional de translação (frente/trás) pelo erro de linha
int calcularVelocidadeLinhaDefensor(float erroA, float erroB, bool temZonaA, bool temZonaB,
                                    float toleranciaZero, float erroMaxRef,
                                    int velocidadeMinima, int velocidadeMaxima) {
  float intensidade = 0.0f;
  if (temZonaA) { float magA = fabsf(erroA) - toleranciaZero; if (magA > intensidade) intensidade = magA; }
  if (temZonaB) { float magB = fabsf(erroB) - toleranciaZero; if (magB > intensidade) intensidade = magB; }
  if (intensidade <= 0.0f) return 0;
  if (intensidade > erroMaxRef) intensidade = erroMaxRef;
  float t = intensidade / erroMaxRef;
  return constrain((int)(velocidadeMinima + t * (float)(velocidadeMaxima - velocidadeMinima)), velocidadeMinima, velocidadeMaxima);
}

// Calcula vetor de atração média entre duas leituras de linha
float calcularVetorAtracaoLinha(float anguloA, float anguloB) {
  float aRad = anguloA * PI / 180.0f;
  float bRad = anguloB * PI / 180.0f;
  float mx   = cosf(aRad) + cosf(bRad);
  float my   = sinf(aRad) + sinf(bRad);
  return normalizarAngulo360(atan2f(my, mx) * 180.0f / PI);
}

// Compõe ângulo de retorno pela bússola com vetor de repulsão dos ultrassônicos laterais
float comporAnguloRetornoBussolaComUltraLaterais(float anguloRetornoBase, bool ultrasRecentes) {
  float anguloBaseRad = anguloRetornoBase * PI / 180.0f;
  float vetorX = sinf(anguloBaseRad) * DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM;
  float vetorY = cosf(anguloBaseRad) * DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM;

  if (ultrasRecentes) {
    if ((ultraDcm >= 0.0f) && (ultraDcm < DEFENSOR_ULTRA_LATERAL_ATIVO_CM) && (ultraEcm > DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM))
      vetorX -= mapearFaixaClamped(ultraDcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA);
    if ((ultraEcm >= 0.0f) && (ultraEcm < DEFENSOR_ULTRA_LATERAL_ATIVO_CM) && (ultraDcm > DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM))
      vetorX += mapearFaixaClamped(ultraEcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA);
  }
  return calcularAnguloVetorDefensor(vetorX, vetorY);
}

// Executa avanço frontal temporizado quando a bola fica na frente por tempo suficiente
bool executarAvancoFrontalTemporizadoDefensor(unsigned long agora,
                                              float &vetorXSuave,
                                              float &vetorYSuave,
                                              float &cmdGiroSuave) {
  static unsigned long inicioDeteccaoIrFrontalMs  = 0;
  static unsigned long inicioAvancoIrFrontalMs    = 0;
  static bool avancoIrFrontalAtivo                = false;
  static bool alinhamentoIrFrontalAtivo           = false;
  static bool aguardarSaidaJanelaIrFrontal        = false;
  static float ultimoAnguloAvancoIrFrontal        = 0.0f;

  bool irFrontalAtivo = irDetectado &&
                        (fabsf(normalizarErro180(anguloIr)) <= DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS);

  // Executa avanço enquanto dentro do tempo
  if (avancoIrFrontalAtivo) {
    if ((agora - inicioAvancoIrFrontalMs) < DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS) {
      if (irDetectado) ultimoAnguloAvancoIrFrontal = normalizarAngulo360(anguloIr);
      alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
      resetPidBussola(); resetPidLinhaGoleiro();
      vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
      seguirDirecaoPorAngulo(ultimoAnguloAvancoIrFrontal, DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM);
      return true;
    }
    avancoIrFrontalAtivo = false; inicioAvancoIrFrontalMs = inicioDeteccaoIrFrontalMs = 0;
  }

  // Alinha ao ângulo da bola antes de avançar
  if (alinhamentoIrFrontalAtivo) {
    if (!irDetectado) {
      alinhamentoIrFrontalAtivo = false; inicioDeteccaoIrFrontalMs = 0;
      aguardarSaidaJanelaIrFrontal = false; resetPidBussola(); return false;
    }
    float erroAlinhamentoBola    = normalizarErro180(anguloIr);
    ultimoAnguloAvancoIrFrontal  = normalizarAngulo360(anguloIr);
    erroAlinhamentoGraus         = erroAlinhamentoBola;
    if (fabsf(erroAlinhamentoBola) > DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS) {
      alinhandoAgora = true; resetPidLinhaGoleiro();
      vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
      int cmdPidBola  = calcularSaidaPidBussola(erroAlinhamentoBola);
      int cmdGiroBola = -SINAL_GIRO_PID * cmdPidBola;
      girarNoEixo(-cmdGiroBola); return true;
    }
    alinhamentoIrFrontalAtivo = false; avancoIrFrontalAtivo = true;
    inicioAvancoIrFrontalMs   = agora;
    alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
    resetPidBussola(); resetPidLinhaGoleiro();
    vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
    seguirDirecaoPorAngulo(ultimoAnguloAvancoIrFrontal, DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM);
    return true;
  }

  // Aguarda bola sair da janela frontal antes de nova detecção
  if (!irFrontalAtivo) { inicioDeteccaoIrFrontalMs = 0; aguardarSaidaJanelaIrFrontal = false; return false; }
  if (aguardarSaidaJanelaIrFrontal) return false;

  // Cronometra tempo com bola frontal
  if (inicioDeteccaoIrFrontalMs == 0) { inicioDeteccaoIrFrontalMs = agora; return false; }
  if ((agora - inicioDeteccaoIrFrontalMs) < DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS) return false;

  // Gatilho atingido: inicia alinhamento antes do avanço
  alinhamentoIrFrontalAtivo    = true;
  aguardarSaidaJanelaIrFrontal = true;
  ultimoAnguloAvancoIrFrontal  = normalizarAngulo360(anguloIr);
  alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
  resetPidBussola(); resetPidLinhaGoleiro();
  vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
  return false;
}

// Placeholder para nova lógica de ultrassônico específica do defensor
bool ultrassonico_defensor() {
  return false;
}

// *** FUNCAO PRINCIPAL DO DEFENSOR ***
// Mantém posição no gol usando linha, bola e ultrassônicos como entradas do vetor de movimento
void defensor() {
  int16_t anguloGolSelecionadoMenu = -999;
  uint16_t distanciaGolSelecionadoMenu = 0;
  bool golSelecionadoMenuVisivel = cameraLerGolSelecionadoMenu(anguloGolSelecionadoMenu, distanciaGolSelecionadoMenu);
  bool retornoDefensorPorBussolaValido = bussolaTemReferenciaValida();
  float anguloRetornoDefensor = retornoDefensorPorBussolaValido ? calcularAnguloRetornoGolPorBussola() : 0.0f;
  static unsigned long ultimoPrintGolSelecionadoMs = 0;

  if ((millis() - ultimoPrintGolSelecionadoMs) >= 200) {
    Serial.print("DEF GOL MENU: ");
    Serial.print(corGolAzul ? "AZUL" : "AMARELO");
    Serial.print(" ANG=");
    Serial.print(anguloGolSelecionadoMenu);
    Serial.print(" DIST=");
    Serial.print(distanciaGolSelecionadoMenu);
    Serial.print(" VIS=");
    Serial.println(golSelecionadoMenuVisivel ? 1 : 0);
    ultimoPrintGolSelecionadoMs = millis();
  }



  static float vetorXSuave = 0.0f;
  static float vetorYSuave = 0.0f;
  static float cmdGiroSuave = 0.0f;
  static bool retornoAoGolPorBussolaAtivo = false;
  static bool alinhamentoAntesRetornoBussolaPendente = false;
  static bool alinhamentoUnicoBussolaPendente = false;
  static unsigned long inicioAlinhamentoAntesRetornoBussolaMs = 0;
  static unsigned long inicioAlinhamentoUnicoBussolaMs = 0;

  bool temZonaAAtual = linhaZonaAValida && (anguloLinhaZonaA >= 0.0f);
  bool temZonaBAtual = linhaZonaBValida && (anguloLinhaZonaB >= 0.0f);
  float anguloZonaAUsado = temZonaAAtual ? anguloLinhaZonaA : -1.0f;
  float anguloZonaBUsado = temZonaBAtual ? anguloLinhaZonaB : -1.0f;

  unsigned long agora = millis();
  bool temZonaARetida = (!temZonaAAtual) && (ultimoAnguloLinhaZonaAValido >= 0.0f) &&
                        ((agora - ultimoRxLinhaZonaAMs) <= RETENCAO_ZONA_LINHA_DEFENSOR_MS);
  bool temZonaBRetida = (!temZonaBAtual) && (ultimoAnguloLinhaZonaBValido >= 0.0f) &&
                        ((agora - ultimoRxLinhaZonaBMs) <= RETENCAO_ZONA_LINHA_DEFENSOR_MS);

  if (temZonaARetida) {
    anguloZonaAUsado = ultimoAnguloLinhaZonaAValido;
  }
  if (temZonaBRetida) {
    anguloZonaBUsado = ultimoAnguloLinhaZonaBValido;
  }

  bool temZonaA = temZonaAAtual || temZonaARetida;
  bool temZonaB = temZonaBAtual || temZonaBRetida;
  bool linhaDefensorDisponivel = temZonaA || temZonaB;

  alinhandoAgora = false;
  fugindoLinhaAgora = false;

  float erroAngularLinha = 0.0f;
  int quantidadeErros = 0;

  if (temZonaA) {
    erroAngularLinha += normalizarErro180(anguloZonaAUsado - DEFENSOR_REFERENCIA_ZONA_A);
    quantidadeErros++;
  }

  if (temZonaB) {
    erroAngularLinha += normalizarErro180(anguloZonaBUsado - DEFENSOR_REFERENCIA_ZONA_B);
    quantidadeErros++;
  }

  if (quantidadeErros > 0) {
    erroAngularLinha /= (float)quantidadeErros;
  } else {
    resetPidLinhaGoleiro();
  }

  erroAngularLinha = aplicarDeadzoneDefensor(erroAngularLinha, 0.5f);
  erroAlinhamentoGraus = erroAngularLinha;

  int cmdPid = calcularSaidaPidLinhaGoleiro(erroAngularLinha);
  if (fabsf(erroAngularLinha) < DEFENSOR_TOLERANCIA_GIRO_GRAUS) {
    cmdPid = 0;
  }
  cmdPid = constrain(cmdPid, -180, 180);

  float cmdGiroAlvo = (float)(SINAL_GIRO_PID * cmdPid);
  if (fabsf(cmdGiroAlvo) < DEFENSOR_DEADZONE_GIRO) {
    cmdGiroAlvo = 0.0f;
  }
  cmdGiroSuave = suavizarDefensor(cmdGiroSuave, cmdGiroAlvo, DEFENSOR_SUAVIZACAO_GIRO);
  int cmdGiro = (int)roundf(cmdGiroSuave);
  alinhandoAgora = (fabsf(erroAngularLinha) >= DEFENSOR_TOLERANCIA_GIRO_GRAUS);

  float vetorX = 0.0f;
  float vetorY = 0.0f;
  const bool centroLinhaValido = temZonaA && temZonaB;
  const float pesoAtracaoLinha = 35.0f;
  const float fatorBolaComLinha = centroLinhaValido ? 0.9f : 1.78f;
  const float fatorUltra = centroLinhaValido ? 0.75f : 1.0f;
  const float bolaDireitaMin = 15.0f;
  const float bolaDireitaMax = 115.0f;
  const float bolaEsquerdaMin = 245.0f;
  const float bolaEsquerdaMax = 345.0f;
  const unsigned long TEMPO_MAX_ALINHAMENTO_UNICO_BUSSOLA_MS = 2000;
  bool ultrasRecentes = ultrasValidos && (ultimoRxUltraMs > 0) && ((agora - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);

  if (executarAvancoFrontalTemporizadoDefensor(agora, vetorXSuave, vetorYSuave, cmdGiroSuave)) {
    return;
  }

  if (!linhaDefensorDisponivel && retornoDefensorPorBussolaValido) {
    if (!retornoAoGolPorBussolaAtivo) {
      retornoAoGolPorBussolaAtivo = true;
      alinhamentoAntesRetornoBussolaPendente = true;
      alinhamentoUnicoBussolaPendente = true;
      inicioAlinhamentoAntesRetornoBussolaMs = 0;
      inicioAlinhamentoUnicoBussolaMs = 0;
      erroAlinhamentoGraus = 0.0f;
      alinhandoAgora = false;
      resetPidLinhaGoleiro();
      resetPidBussola();
      vetorXSuave = 0.0f;
      vetorYSuave = 0.0f;
      cmdGiroSuave = 0.0f;
    }

    if (alinhamentoAntesRetornoBussolaPendente) {
      if (inicioAlinhamentoAntesRetornoBussolaMs == 0) {
        inicioAlinhamentoAntesRetornoBussolaMs = agora;
      }

      float erroBussolaRetorno = calcularErroReferenciaBussola();
      erroAlinhamentoGraus = erroBussolaRetorno;
      bool tempoAlinhamentoAtivo = (agora - inicioAlinhamentoAntesRetornoBussolaMs) < TEMPO_MAX_ALINHAMENTO_UNICO_BUSSOLA_MS;

      if (tempoAlinhamentoAtivo && (fabsf(erroBussolaRetorno) > TOLERANCIA_ALINHAMENTO_GRAUS)) {
        alinhandoAgora = true;
        resetPidLinhaGoleiro();
        vetorXSuave = 0.0f;
        vetorYSuave = 0.0f;
        cmdGiroSuave = 0.0f;

        int cmdPidBussola = calcularSaidaPidBussola(erroBussolaRetorno);
        int cmdGiroBussola = SINAL_GIRO_PID * cmdPidBussola;
        girarNoEixo(cmdGiroBussola);
        return;
      }

      alinhamentoAntesRetornoBussolaPendente = false;
      inicioAlinhamentoAntesRetornoBussolaMs = 0;
      resetPidBussola();
    }

    erroAlinhamentoGraus = 0.0f;
    alinhandoAgora = false;
    float anguloRetornoComUltra = comporAnguloRetornoBussolaComUltraLaterais(anguloRetornoDefensor, ultrasRecentes);
    seguirDirecaoPorAngulo(anguloRetornoComUltra,
                           (int)DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM);
    return;
  }

  if (linhaDefensorDisponivel && alinhamentoUnicoBussolaPendente) {
    if (retornoDefensorPorBussolaValido) {
      if (inicioAlinhamentoUnicoBussolaMs == 0) {
        inicioAlinhamentoUnicoBussolaMs = agora;
      }

      float erroBussolaRetorno = calcularErroReferenciaBussola();
      erroAlinhamentoGraus = erroBussolaRetorno;
      bool tempoAlinhamentoAtivo = (agora - inicioAlinhamentoUnicoBussolaMs) < TEMPO_MAX_ALINHAMENTO_UNICO_BUSSOLA_MS;

      if (tempoAlinhamentoAtivo && (fabsf(erroBussolaRetorno) > TOLERANCIA_ALINHAMENTO_GRAUS)) {
        alinhandoAgora = true;
        resetPidLinhaGoleiro();
        vetorXSuave = 0.0f;
        vetorYSuave = 0.0f;
        cmdGiroSuave = 0.0f;

        int cmdPidBussola = calcularSaidaPidBussola(erroBussolaRetorno);
        int cmdGiroBussola = SINAL_GIRO_PID * cmdPidBussola;
        girarNoEixo(cmdGiroBussola);
        return;
      }
    }

    alinhamentoUnicoBussolaPendente = false;
    retornoAoGolPorBussolaAtivo = false;
    alinhamentoAntesRetornoBussolaPendente = false;
    inicioAlinhamentoAntesRetornoBussolaMs = 0;
    inicioAlinhamentoUnicoBussolaMs = 0;
    resetPidBussola();
  }

  if (!retornoAoGolPorBussolaAtivo) {
    alinhamentoAntesRetornoBussolaPendente = false;
    alinhamentoUnicoBussolaPendente = false;
    inicioAlinhamentoAntesRetornoBussolaMs = 0;
    inicioAlinhamentoUnicoBussolaMs = 0;
  }

  if (centroLinhaValido) {
    float anguloCentroLinha = calcularVetorAtracaoLinha(anguloZonaAUsado, anguloZonaBUsado);
    float anguloCentroLinhaRad = anguloCentroLinha * PI / 180.0f;

    // O centro entre A e B passa a ser a prioridade maxima de translacao do defensor.
    vetorX += sinf(anguloCentroLinhaRad) * pesoAtracaoLinha;
    vetorY += cosf(anguloCentroLinhaRad) * pesoAtracaoLinha;
  }

  // Bola: mantem a mesma logica de peso, mas usa a camera quando o IR nao estiver vendo.
  float anguloBola = -1.0f;
  bool bolaDisponivel = false;


  float anguloIrBufferizado = -1.0f;
  if (obterAnguloIrComBuffer(anguloIrBufferizado)) {
    anguloBola = anguloIrBufferizado;
    bolaDisponivel = true;
  } else if (cameraTemBolaValida()) {
    anguloBola = normalizarAngulo360((float)cameraBallAngle);
    bolaDisponivel = true;
  }

  if (bolaDisponivel) {
    if (anguloBola >= bolaDireitaMin && anguloBola <= bolaDireitaMax) {
      vetorX += mapearFaixaClamped(anguloBola, bolaDireitaMin, bolaDireitaMax, DEFENSOR_PESO_MIN_BOLA, DEFENSOR_PESO_MAX_BOLA) * fatorBolaComLinha;
    }
    if (anguloBola >= bolaEsquerdaMin && anguloBola <= bolaEsquerdaMax) {
      vetorX -= mapearFaixaClamped(anguloBola, bolaEsquerdaMax, bolaEsquerdaMin, DEFENSOR_PESO_MIN_BOLA, DEFENSOR_PESO_MAX_BOLA) * fatorBolaComLinha;
    }
  }else{
        if ((ultraDcm >= 0.0f) && (ultraDcm < 80) && (ultraEcm > 40)) {
      vetorX -= mapearFaixaClamped(ultraDcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA) * fatorUltra;
    }else{
              if ((ultraEcm >= 0.0f) && (ultraEcm < 80) && (ultraDcm > 40)) {
vetorX += mapearFaixaClamped(ultraEcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA) * fatorUltra;
    }
    }
  }

  // Ultrassons: vetores de contencao, nunca mais como prioridade bloqueante.
  if (ultrasRecentes) {
    if ((ultraDcm >= 0.0f) && (ultraDcm < DEFENSOR_ULTRA_LATERAL_ATIVO_CM) && (ultraEcm > DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM)) {
      vetorX -= mapearFaixaClamped(ultraDcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA) * fatorUltra;
    }
    if ((ultraEcm >= 0.0f) && (ultraEcm < DEFENSOR_ULTRA_LATERAL_ATIVO_CM) && (ultraDcm > DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM)) {
      vetorX += mapearFaixaClamped(ultraEcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA) * fatorUltra;
    }
    if ((ultraTcm > DEFENSOR_ULTRA_FRENTE_LIMITE_CM) && (ultraFcm > DEFENSOR_ULTRA_FRONTAL_CONFIRMADOR_CM)) {
      vetorY -= mapearFaixaClamped(ultraTcm, DEFENSOR_ULTRA_FRENTE_LIMITE_CM, DEFENSOR_ULTRA_PROFUNDIDADE_MAX_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA_PROFUNDIDADE) * fatorUltra;
    }
    if ((ultraTcm >= 0.0f) && (ultraTcm < DEFENSOR_ULTRA_TRAS_LIMITE_CM) && (ultraFcm > DEFENSOR_ULTRA_FRONTAL_CONFIRMADOR_CM)) {
      vetorY += mapearFaixaClamped(ultraTcm, DEFENSOR_ULTRA_TRAS_LIMITE_CM, DEFENSOR_ULTRA_PROFUNDIDADE_MIN_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA_PROFUNDIDADE) * fatorUltra;
    }
  }

  vetorX = aplicarDeadzoneDefensor(vetorX, DEFENSOR_DEADZONE_VETOR);
  vetorY = aplicarDeadzoneDefensor(vetorY, DEFENSOR_DEADZONE_VETOR);

  // Suavizacao exponencial para reduzir jitter entre frames.
  vetorXSuave = suavizarDefensor(vetorXSuave, vetorX, DEFENSOR_SUAVIZACAO_VETOR);
  vetorYSuave = suavizarDefensor(vetorYSuave, vetorY, DEFENSOR_SUAVIZACAO_VETOR);
  vetorXSuave = aplicarDeadzoneDefensor(vetorXSuave, DEFENSOR_DEADZONE_VETOR * 0.5f);
  vetorYSuave = aplicarDeadzoneDefensor(vetorYSuave, DEFENSOR_DEADZONE_VETOR * 0.5f);

  // Resultado final: soma de linha, bola e ultras em um unico comando de translacao.
  float magnitudeVetor = calcularMagnitudeVetorDefensor(vetorXSuave, vetorYSuave);
  int velocidadeFinal = (int)roundf(constrain(magnitudeVetor, 0.0f, (float)velocidade_maxima));
  if (velocidadeFinal > 0 && velocidadeFinal < DEFENSOR_VELOCIDADE_MIN_PWM) {
    velocidadeFinal = (int)DEFENSOR_VELOCIDADE_MIN_PWM;
  }

  float anguloFinal = calcularAnguloVetorDefensor(vetorXSuave, vetorYSuave);
  seguirDirecaoComGiro(anguloFinal, velocidadeFinal, cmdGiro);
}

// =============================================================================
// SECAO 21 — SETUP: INICIALIZACAO DO HARDWARE E HANDSHAKE INICIAL
// =============================================================================

void setup() {
  // Inicializa seriais: debug (USB) e comunicação com a Cabeça
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_CABECA, TX_CABECA);

  // Recupera da EEPROM a referência de bússola, papel e cor de gol
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDR_BUSSOLA, headingBussolaSalvo);
  if (headingBussolaSalvo < 0 || headingBussolaSalvo >= 360) headingBussolaSalvo = 0;
  carregarPapelConfiguradoEEPROM();
  carregarCorGolEEPROM();

  // Configura todas as saídas da ponte H e o pino do kicker
  pinMode(IN1_1_A, OUTPUT); pinMode(IN2_1_A, OUTPUT);
  pinMode(IN1_2_A, OUTPUT); pinMode(IN2_2_A, OUTPUT);
  pinMode(IN1_1_B, OUTPUT); pinMode(IN2_1_B, OUTPUT);
  pinMode(IN1_2_B, OUTPUT); pinMode(IN2_2_B, OUTPUT);
  pinMode(KICKER_PIN, OUTPUT);
  digitalWrite(KICKER_PIN, LOW);

  // Associa cada pino PWM ao seu canal no ESP32
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_1_A, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_2_A, PWM_CH2);
  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_1_B, PWM_CH3);
  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_2_B, PWM_CH4);
  pararMotores();

  // Inicializa I2C e display OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1) { delay(100); }
  }

  // Exibe tela de inicialização
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 25);
  display.println("TESTANDO...");
  display.display();

  // Tenta handshake inicial com a Cabeça por TIMEOUT_COM_MS
  unsigned long inicio = millis();
  while ((millis() - inicio) < TIMEOUT_COM_MS) {
    if (millis() - ultimoEnvioOi >= 300) {
      Serial1.println("oi"); ultimoEnvioOi = millis();
    }
    lerSerialCabeca();
    if (comunicacaoCabecaOK) break;
    delay(10);
  }

  enviarEstadoJogoParaCabeca(true);
  desenharTelaAtual();
}


// =============================================================================
// SECAO 22 — LOOP PRINCIPAL
// =============================================================================

void loop() {
  // Heartbeat: mantém a serial com a Cabeça viva
  if (millis() - ultimoEnvioOi >= INTERVALO_OI_MS) {
    Serial1.println("oi"); ultimoEnvioOi = millis();
  }

  // Processa todas as entradas e envia configurações pendentes
  lerSerialCabeca();
  atualizarValidadeBussola();
  atualizarValidadeLinha();
  atualizarValidadeCamera();
  atualizarSozinhoLocal();
  atualizarPapelAutomaticoPorParceria();
  enviarEstadoJogoParaCabeca();
  enviarCorGolParaCabeca();
  solicitarSensoresBrutosPe();
  if (!limiarLinhaSincronizado) solicitarLimiarLinhaPe();
  atualizarKicker();

  // Timeout de comunicação: derruba estado se Cabeça ficar silenciosa
  if ((millis() - ultimoRxCabeca) > TIMEOUT_COM_MS) {
    comunicacaoCabecaOK = false; bussolaValida = false;
  }

  // Lógica de jogo: executa estratégia conforme papel atual
  if (estadoAtual == INICIAR && comunicacaoCabecaOK) {
    if (papelAtacante != papelAtacanteAnterior) {
      // Reinicia transição angular ao mudar de papel (evita carregar estado antigo)
      resetControleMovimentoAtacante();
      papelAtacanteAnterior = papelAtacante;
    }
    if (papelAtacante) atacante();
    else               defensor();
  } else {
    // Fora do modo de jogo: zera controles e para os motores
    alinhandoAgora = false; fugindoLinhaAgora = false;
    resetControleMovimentoAtacante();
    resetPidBussola();
    pararMotores();
  }

  // Redesenha o OLED com taxa limitada para evitar flicker
  static unsigned long ultimaTela = 0;
  if ((millis() - ultimaTela) > 120) {
    desenharTelaAtual(); ultimaTela = millis();
  }

 // delay(5);
}