// =============================================================================
// MUSCULO.CPP — Placa de Atuadores e Estratégias do Nexus
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
#include "motores_movimentacao.hpp"
#include "display/ihm_display.hpp"
#include "atacante.hpp"
#include "defensor.hpp"


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

// --- Pinos do barramento I2C ---
#define SDA_PIN 8
#define SCL_PIN 9

// --- Pino e temporização do solenoide (kicker) ---
constexpr uint8_t  KICKER_PIN = 21;
constexpr unsigned long KICK_PULSE_MS = 100;   // Duração do pulso de chute (ms)
constexpr unsigned long KICK_INTERVAL_MS = 1000;  // Intervalo mínimo entre chutes (ms)

// =============================================================================
// SECAO 2 — VARIAVEIS DE COMUNICACAO E ESTADO GERAL
// =============================================================================

// --- Estado da comunicação com a Cabeça ---
bool   comunicacaoCabecaOK  = false;
String bufferSerial         = "";
String mensagemBotao        = "NENHUM";

//////////////////////////////////////////////////////////////////////////////////////////////
// --- Módulo RoboCup Junior informado pela Cabeça (MOD:0/1) ---
// Módulo RoboCup Junior recebido da Cabeça: MOD:0 ou MOD:1
bool USANDO_MODULO = true;  // Mude manualmente para true se quiser exigir o módulo
bool MODULO_ATIVO = false;
unsigned long ultimoRxModuloMs = 0;
///////////////////////////////////////////////////////////////////////////////////////////
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



extern const int VELOCIDADE_FUGA_LINHA = 255;
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
unsigned long ultimoRxIrValidoMs = 0;

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
const unsigned long INTERVALO_REQ_LIMIAR_LINHA_MS = 200;

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
const unsigned long TIMEOUT_LINHA_MS      = 100;

// --- Cor do gol de referência e envio pendente à Cabeça ---
bool corGolAzul             = false;
bool corGolPendenteEnvio    = true;
unsigned long ultimoEnvioCorGolMs         = 0;
const unsigned long INTERVALO_ENVIO_COR_GOL_MS = 200;
unsigned long ultimoEnvioRefBussolaMs = 0;
const unsigned long INTERVALO_ENVIO_REF_BUSSOLA_MS = 200;

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

// --- Zona da bola vista pelo parceiro quando ele esta no papel de defensor ---
char zonaDefensorRecebida = 'C';
unsigned long ultimoRxZonaDefensorMs = 0;
const unsigned long TIMEOUT_ZONA_DEFENSOR_MS = 3000;

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
int papelConfiguradoMenu = PAPEL_CONFIG_AUTO;

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
int estadoAtual   = MENU;
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
int subMenuCalibracao = SUBMENU_PRINCIPAL;
int itemSubMenu = 0;

// Submenus de função
enum SubMenuFuncao {
  SUBFUNCAO_PRINCIPAL,
  SUBFUNCAO_PAPEIS,
  SUBFUNCAO_POSICIONAMENTO,
  SUBFUNCAO_SENSORES,
  SUBFUNCAO_LIMIAR_LINHA,
  SUBFUNCAO_KICKER
};
int subMenuFuncao     = SUBFUNCAO_PRINCIPAL;
int           itemSubMenuFuncao = 0;

bool bussolaTemReferenciaValida();
float calcularErroReferenciaBussola();

// Posicionamento por coordenadas enviado via HTTPS (repasse da Cabeca)
bool posicionamentoAlvoAtivo = false;
float posicionamentoAlvoXcm = 91.0f;
float posicionamentoAlvoYcm = 121.5f;


// =============================================================================
// SECAO 6 — CONTROLE DE MOVIMENTO: PARAMETROS GERAIS
// =============================================================================

int velocidade_maxima = 255;

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


const float CAMPO_LARGURA_CM = 182.0f;
const float CAMPO_ALTURA_CM = 243.0f;
const float ROBO_DIAMETRO_CAMPO_CM = 21.0f;
const float ROBO_RAIO_CAMPO_CM = ROBO_DIAMETRO_CAMPO_CM * 0.5f;
const float POS_COMP_TOL_CM = 22.0f;
const float POS_TAU_FAST = 0.26f;
const float POS_TAU_SLOW = 0.48f;
const float POS_TOLERANCIA_CM = 10.0f;
const float POS_TOLERANCIA_STOP_BRUTA_CM = 13.0f;
const float POS_JUMP_MAX_CM = 35.0f;
const int POS_VELOCIDADE_PWM = 150;
const int POS_VELOCIDADE_PWM_MIN = 100;
const float POS_DIST_RAMP_CM = 80.0f;
const unsigned long POS_STOP_CONFIRM_MS = 180;
const float POS_TOLERANCIA_GIRO_GRAUS = 6.0f;
const float POS_GIRO_SO_EIXO_GRAUS = 35.0f;

// --- Tolerâncias de alinhamento ---
// *** AJUSTE AQUI para afinar a janela de alinhamento com o gol ***
const float TOLERANCIA_ALINHAMENTO_GRAUS   = 8.0f;
const float JANELA_FUZZY_ALINHAMENTO_GRAUS = 30.0f;
const int   VELOCIDADE_GIRO_ALINHAMENTO    = VELOCIDADE_GIRO;
const int   SINAL_GIRO_PID                 = SINAL_GIRO;

// --- Tempo de retenção de dados de linha e zona (evita perda por frame único) ---
const unsigned long RETENCAO_FUGA_LINHA_MS          = 500;
const unsigned long RETENCAO_ZONA_LINHA_DEFENSOR_MS  = 300;
const unsigned long TEMPO_CAMERA_SEM_IR_PARA_IGNORAR_LINHA_MS = 1000;
const unsigned long RETENCAO_IR_VALIDO_MS = 500;


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

bool posicionamentoModuloArmado() {
  return (estadoAtual == FUNCAO) && (subMenuFuncao == SUBFUNCAO_POSICIONAMENTO);
}

void cancelarPosicionamentoAlvo(bool manterMensagem = false) {
  posicionamentoAlvoAtivo = false;
  resetPidBussola();
  pararMotores();
  if (!manterMensagem) {
    mensagemBotao = "POS CANCELADO";
    mostrarStatusAte = millis() + 1800;
  }
}

bool dentroToleranciaPos(float erroX, float erroY, float tolerancia) {
  return (fabsf(erroX) <= tolerancia) && (fabsf(erroY) <= tolerancia);
}

bool executarPosicionamentoAlvo() {
  static unsigned long dentroTolDesdeMs = 0;

  if (!posicionamentoModuloArmado()) {
    dentroTolDesdeMs = 0;
    if (posicionamentoAlvoAtivo) {
      cancelarPosicionamentoAlvo(true);
      mensagemBotao = "POS SAIU MODULO";
      mostrarStatusAte = millis() + 1800;
    }
    return false;
  }

  if (!posicionamentoAlvoAtivo) {
    dentroTolDesdeMs = 0;
    resetPidBussola();
    pararMotores();
    return true;
  }

  if (!ultrasValidos) {
    pararMotores();
    mensagemBotao = "POS AGUARDA SENS";
    mostrarStatusAte = millis() + 1200;
    return true;
  }

  atualizarLeiturasPosicionamento(ultraEcm, ultraDcm, ultraFcm, ultraTcm, ultrasValidos);
  if (!atualizarPosicaoAtual()) {
    pararMotores();
    mensagemBotao = "POS SEM POSICAO";
    mostrarStatusAte = millis() + 1200;
    return true;
  }

  float atualX = obterPosicaoX();
  float atualY = obterPosicaoY();

  const float erroX = posicionamentoAlvoXcm - atualX;
  const float erroY = posicionamentoAlvoYcm - atualY;
  const bool ultraEValido = isfinite(ultraEcm) && ultraEcm > 1.0f && ultraEcm < 350.0f;
  const bool ultraFValido = isfinite(ultraFcm) && ultraFcm > 1.0f && ultraFcm < 350.0f;
  const float erroXBruto = posicionamentoAlvoXcm - constrain((ultraEValido ? (ultraEcm + ROBO_RAIO_CAMPO_CM) : atualX), ROBO_RAIO_CAMPO_CM, CAMPO_LARGURA_CM - ROBO_RAIO_CAMPO_CM);
  const float erroYBruto = posicionamentoAlvoYcm - constrain((ultraFValido ? (ultraFcm + ROBO_RAIO_CAMPO_CM) : atualY), ROBO_RAIO_CAMPO_CM, CAMPO_ALTURA_CM - ROBO_RAIO_CAMPO_CM);
  bool emTolerancia = dentroToleranciaPos(erroX, erroY, POS_TOLERANCIA_CM) ||
                      dentroToleranciaPos(erroXBruto, erroYBruto, POS_TOLERANCIA_STOP_BRUTA_CM);

  if (emTolerancia) {
    if (dentroTolDesdeMs == 0) {
      dentroTolDesdeMs = millis();
    }
    if ((millis() - dentroTolDesdeMs) >= POS_STOP_CONFIRM_MS) {
      cancelarPosicionamentoAlvo(true);
      mensagemBotao = "POSICAO ATINGIDA";
      mostrarStatusAte = millis() + 2200;
      dentroTolDesdeMs = 0;
      return true;
    }
  } else {
    dentroTolDesdeMs = 0;
  }

  bool usarGiroPos = false;
  if (bussolaTemReferenciaValida()) {
    float erroBussolaPos = calcularErroReferenciaBussola();
    if (fabsf(erroBussolaPos) > 5.0f) {
      usarGiroPos = true;
    }
  }

  bool movimentoOk = false;
  if (usarGiroPos) {
    movimentoOk = moverParaComGiro(posicionamentoAlvoXcm, posicionamentoAlvoYcm);
  } else {
    movimentoOk = moverParaSemGiro(posicionamentoAlvoXcm, posicionamentoAlvoYcm);
  }

  if (!movimentoOk) {
    pararMotores();
    mensagemBotao = "POS SEM POSICAO";
    mostrarStatusAte = millis() + 1200;
    return true;
  }

  return true;
}


// =============================================================================
// SECAO 9 — KICKER (SOLENOIDE)
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
// SECAO 10 — FUNCOES DE GOL, BUSSOLA E ANGULO DE CAMPO (compartilhadas)
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
  static bool headingFiltradoInicializado = false;
  static float headingBussolaFiltrado = 0.0f;
  const float ALPHA_FILTRO_COMPLEMENTAR_BUSSOLA = 0.8f;

  if (!bussolaValida) {
    headingFiltradoInicializado = false;
    return 0.0f;
  }

  float headingAtual = normalizarAngulo360((float)headingBussolaTeste);

  if (!headingFiltradoInicializado) {
    headingBussolaFiltrado = headingAtual;
    headingFiltradoInicializado = true;
  } else {
    float erroCircular = normalizarErro180(headingAtual - headingBussolaFiltrado);
    headingBussolaFiltrado = normalizarAngulo360(
        headingBussolaFiltrado + (ALPHA_FILTRO_COMPLEMENTAR_BUSSOLA * erroCircular));
  }

  return normalizarErro180((float)headingBussolaSalvo - headingBussolaFiltrado);
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
// SECAO 11 — FUNCOES DE CAMERA (compartilhadas)
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
// SECAO 12 — FUNCOES DE LINHA (compartilhadas)
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
// SECAO 13 — FUNCOES ESP-NOW E PAPEL AUTOMATICO
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
// SECAO 14 — FUNCOES DE ENVIO PARA A CABECA
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

// Envia a referencia de heading salva na EEPROM para a Cabeca.
void enviarReferenciaBussolaParaCabeca(bool forcar = false) {
  if (!forcar && (millis() - ultimoEnvioRefBussolaMs) < INTERVALO_ENVIO_REF_BUSSOLA_MS) {
    return;
  }

  int ref = headingBussolaSalvo;
  if (ref < 0) ref = 0;
  if (ref >= 360) ref %= 360;

  Serial1.print("BUSREF:");
  Serial1.println(ref);
  ultimoEnvioRefBussolaMs = millis();
}

// Envia para a Cabeca uma zona de teste (A/B/C) para forcar no ESP-NOW,
// ou "AUTO" para voltar ao calculo automatico pela camera. Uso: tela de
// teste do defensor, para comprovar o reposicionamento do atacante.
void enviarZonaTesteParaCabeca(char zona) {
  Serial1.print("TESTZONA:");
  if (zona == 'A' || zona == 'B' || zona == 'C') {
    Serial1.println(zona);
  } else {
    Serial1.println("AUTO");
  }
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


// IHM movida para src/NEXUS/musculo/display/ihm_display.cpp


// =============================================================================
// SECAO 15 — PROCESSAMENTO DE MENSAGENS DA CABECA (Serial1)
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

// Estado do módulo RoboCup Junior: MOD:1 = ativo, MOD:0 = inativo
// Estado do módulo RoboCup Junior: MOD:1 = ativo, MOD:0 = inativo
if (msg.startsWith("MOD:")) {
  String valorModulo = msg.substring(4);
  valorModulo.trim();

  if (valorModulo == "0" || valorModulo == "1") {
    MODULO_ATIVO = (valorModulo == "1");
    ultimoRxModuloMs = millis();
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = ultimoRxModuloMs;
  }
  return;
}

  // Estado do módulo RoboCup Junior: MOD:1 = ativo, MOD:0 = inativo
  if (msg.startsWith("MOD:")) {
    String valorModulo = msg.substring(4); valorModulo.trim();
    if (valorModulo == "0" || valorModulo == "1") {
      MODULO_ATIVO = (valorModulo == "1");
      ultimoRxModuloMs = millis();
      comunicacaoCabecaOK = true; ultimoRxCabeca = ultimoRxModuloMs;
    }
    return;
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

  // Ângulo IR: negativo = sem bola;
  if (msg.startsWith("IR:")) {
    String valorIr = msg.substring(3); valorIr.trim();
    float novoAngulo = valorIr.toFloat();
    if (novoAngulo < 0.0f) {
      irDetectado = false; anguloIr = -1.0f;
    } else {
        irDetectado = true; anguloIr = novoAngulo;
        ultimoAnguloIrValido = normalizarAngulo360(novoAngulo);
        ultimoRxIrValidoMs = millis();
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

  if (msg.startsWith("POS:")) {
    String payload = msg.substring(4);
    payload.trim();
    int separador = payload.indexOf('/');
    if (separador > 0) {
      String sx = payload.substring(0, separador);
      String sy = payload.substring(separador + 1);
      sx.trim(); sy.trim();
      if (payloadNumericoValido(sx) && payloadNumericoValido(sy)) {
        float alvoX = sx.toFloat();
        float alvoY = sy.toFloat();
        float minX = ROBO_RAIO_CAMPO_CM;
        float maxX = CAMPO_LARGURA_CM - ROBO_RAIO_CAMPO_CM;
        float minY = ROBO_RAIO_CAMPO_CM;
        float maxY = CAMPO_ALTURA_CM - ROBO_RAIO_CAMPO_CM;
        if (!posicionamentoModuloArmado()) {
          mensagemBotao = "POS BLOQ DISPLAY";
          mostrarStatusAte = millis() + 1800;
          cancelarPosicionamentoAlvo(true);
          comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
          return;
        }
        if (alvoX >= minX && alvoX <= maxX && alvoY >= minY && alvoY <= maxY) {
          posicionamentoAlvoXcm = alvoX;
          posicionamentoAlvoYcm = alvoY;
          posicionamentoAlvoAtivo = true;
          mensagemBotao = "POS ALVO RECEBIDO";
          mostrarStatusAte = millis() + 1800;
        } else {
          mensagemBotao = "POS FORA CAMPO";
          mostrarStatusAte = millis() + 1800;
        }
        comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
      }
    }
    return;
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

  // Zona da bola vista pelo parceiro (papel defensor): ZONA:A / ZONA:B / ZONA:C
  if (msg.startsWith("ZONA:")) {
    String sZona = msg.substring(5); sZona.trim();
    if (sZona == "A" || sZona == "B" || sZona == "C") {
      zonaDefensorRecebida = sZona[0]; ultimoRxZonaDefensorMs = millis();
    }
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
// SECAO 16 — SETUP: INICIALIZACAO DO HARDWARE E HANDSHAKE INICIAL
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

  // Configura o pino do kicker
  pinMode(KICKER_PIN, OUTPUT);
  digitalWrite(KICKER_PIN, LOW);

  // Inicializa toda a camada de motores e movimentacao pela biblioteca dedicada
  MotoresMovimentacaoConfig motoresCfg;
  motoresCfg.velocidadeMaxima = velocidade_maxima;
  motoresCfg.passoRampaPwm = PASSO_RAMPA_PWM;
  motoresCfg.ganhoGiroMisto = GANHO_GIRO_MISTO;
  inicializarMotoresMovimentacao(motoresCfg);

  MotoresPosicionamentoConfig posCfg;
  posCfg.campoLarguraCm = CAMPO_LARGURA_CM;
  posCfg.campoAlturaCm = CAMPO_ALTURA_CM;
  posCfg.roboRaioCm = ROBO_RAIO_CAMPO_CM;
  posCfg.compToleranciaCm = POS_COMP_TOL_CM;
  posCfg.filtroTauRapido = POS_TAU_FAST;
  posCfg.filtroTauLento = POS_TAU_SLOW;
  posCfg.saltoMaximoCm = POS_JUMP_MAX_CM;
  posCfg.velocidadePwmMax = POS_VELOCIDADE_PWM;
  posCfg.velocidadePwmMin = POS_VELOCIDADE_PWM_MIN;
  posCfg.distanciaRampaCm = POS_DIST_RAMP_CM;
  posCfg.toleranciaGiroGraus = POS_TOLERANCIA_GIRO_GRAUS;
  posCfg.giroSomenteEixoGraus = POS_GIRO_SO_EIXO_GRAUS;
  posCfg.velocidadeGiroEixoPwm = VELOCIDADE_GIRO_ALINHAMENTO;
  posCfg.sinalGiro = SINAL_GIRO_PID;
  posCfg.erroMinimoAtivarGiroGraus = 5.0f;
  posCfg.hookTemReferenciaOrientacao = bussolaTemReferenciaValida;
  posCfg.hookObterErroOrientacaoGraus = calcularErroReferenciaBussola;
  posCfg.hookCalcularComandoGiro = calcularSaidaPidBussola;
  inicializarMotoresPosicionamento(posCfg);

  // Inicializa I2C e display OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!iniciarDisplayIHM()) {
    while (1) { delay(100); }
  }
  mostrarBootEtapaIHM("OLED", 20);
  mostrarBootEtapaIHM("I2C", 35);
  mostrarBootEtapaIHM("UART", 55);
  mostrarBootEtapaIHM("EEPROM", 75);

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

  mostrarBootEtapaIHM("CABECA", 100);

  enviarEstadoJogoParaCabeca(true);
  enviarReferenciaBussolaParaCabeca(true);
  desenharTelaAtual();
}


// =============================================================================
// SECAO 17 — LOOP PRINCIPAL
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
  enviarReferenciaBussolaParaCabeca();
  solicitarSensoresBrutosPe();
  if (!limiarLinhaSincronizado) solicitarLimiarLinhaPe();
  atualizarKicker();

  // Timeout de comunicação: derruba estado se Cabeça ficar silenciosa
  if ((millis() - ultimoRxCabeca) > TIMEOUT_COM_MS) {
    comunicacaoCabecaOK = false; bussolaValida = false;
  }

  if (executarPosicionamentoAlvo()) {
    static unsigned long ultimaTelaPos = 0;
    if ((millis() - ultimaTelaPos) > 120) {
      desenharTelaAtual(); ultimaTelaPos = millis();
    }
    return;
  }

bool podeExecutarJogo =
    estadoAtual == INICIAR &&
    comunicacaoCabecaOK &&
    (!USANDO_MODULO || MODULO_ATIVO);

if (podeExecutarJogo) {



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

  