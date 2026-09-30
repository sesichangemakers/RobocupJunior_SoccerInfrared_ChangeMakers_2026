#include "ihm_display.hpp"

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

namespace {
constexpr int MENU = 0;
constexpr int CALIBRACAO = 1;
constexpr int FUNCAO = 2;
constexpr int INICIAR = 3;

constexpr int SUBMENU_PRINCIPAL = 0;
constexpr int SUBMENU_GOL = 1;
constexpr int SUBMENU_BUSSOLA = 2;
constexpr int SUBMENU_IR = 3;
constexpr int SUBMENU_ULTRA = 4;
constexpr int SUBMENU_CAMERA = 5;
constexpr int SUBMENU_ESPNOW = 6;
constexpr int SUBMENU_BUSSOLA_AJUSTE = 7;
constexpr int SUBMENU_GOL_CALIBRACAO = 8;

constexpr int SUBFUNCAO_PRINCIPAL = 0;
constexpr int SUBFUNCAO_PAPEIS = 1;
constexpr int SUBFUNCAO_POSICIONAMENTO = 2;
constexpr int SUBFUNCAO_SENSORES = 3;
constexpr int SUBFUNCAO_LIMIAR_LINHA = 4;
constexpr int SUBFUNCAO_KICKER = 5;
constexpr int SUBFUNCAO_ZONA_TESTE = 6;

constexpr int PAPEL_CONFIG_ATACANTE = 0;
constexpr int PAPEL_CONFIG_DEFENSOR = 1;
constexpr int PAPEL_CONFIG_AUTO = 2;

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr unsigned long TIMEOUT_ULTRA_MS_LOCAL = 1000;
constexpr unsigned long TIMEOUT_SENSORES_BRUTOS_MS_LOCAL = 1200;
constexpr int LIMIAR_LINHA_PASSO_LOCAL = 100;
constexpr int LIMIAR_LINHA_MIN_LOCAL = 100;
constexpr int LIMIAR_LINHA_MAX_LOCAL = 4000;
constexpr int EEPROM_ADDR_BUSSOLA_LOCAL = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
}

// Variaveis globais do musculo.cpp usadas pela IHM.
extern bool papelAtacante;
extern bool comunicacaoCabecaOK;
extern String mensagemBotao;
extern unsigned long mostrarStatusAte;
extern bool olhoOK;
extern bool peOK;

extern int estadoAtual;
extern int itemSelecionado;
extern int subMenuCalibracao;
extern int itemSubMenu;
extern int subMenuFuncao;
extern int itemSubMenuFuncao;
extern bool posicionamentoAlvoAtivo;
extern float posicionamentoAlvoXcm;
extern float posicionamentoAlvoYcm;

extern int papelConfiguradoMenu;
extern bool corGolAzul;
extern bool corGolPendenteEnvio;

extern bool bussolaValida;
extern int headingBussolaTeste;
extern int headingBussolaSalvo;

extern bool irDetectado;
extern float anguloIr;

extern bool ultrasValidos;
extern unsigned long ultimoRxUltraMs;
extern float ultraDcm;
extern float ultraEcm;
extern float ultraFcm;
extern float ultraTcm;

extern bool linhaDetectada;
extern float anguloLinhaPe;
extern bool linhaZonaAValida;
extern bool linhaZonaBValida;
extern float anguloLinhaZonaA;
extern float anguloLinhaZonaB;

extern bool cameraDadosValidos;
extern int16_t cameraBallAngle;
extern uint16_t cameraBallDist;
extern int16_t cameraBlueAngle;
extern uint16_t cameraBlueDist;
extern int16_t cameraYellowAngle;
extern uint16_t cameraYellowDist;

extern int sensorBruto1;
extern int sensorBruto9;
extern int sensorBruto17;
extern int sensorBruto25;
extern unsigned long ultimoRxSensoresBrutosMs;

extern int limiarLinhaEditado;
extern bool limiarLinhaSincronizado;
extern unsigned long entradaTelaLimiarMs;

extern float anguloFugaLinhaCmd;
extern float erroAlinhamentoGraus;
extern bool fugindoLinhaAgora;
extern bool alinhandoAgora;

// Zona da bola recebida do parceiro (papel defensor) via ESP-NOW.
extern char zonaDefensorRecebida;
extern unsigned long ultimoRxZonaDefensorMs;
extern const unsigned long TIMEOUT_ZONA_DEFENSOR_MS;

// Funcoes globais do musculo.cpp chamadas pela IHM.
extern bool espnowConectadoRecente();
extern bool cameraTemGolSelecionadoValido(int16_t &anguloGol);
extern float calcularErroGolPorPapel(float anguloGol);
extern bool golReferenciaAzulEfetiva();
extern bool cameraPacoteRecente();

extern void atualizarSozinhoLocal();
extern void enviarEstadoJogoParaCabeca(bool forcar);
extern void enviarPapelAtualParaCabeca();
extern bool salvarPapelConfiguradoEEPROM();
extern bool salvarCorGolEEPROM();
extern void solicitarSensoresBrutosPe(bool forcar);
extern void solicitarLimiarLinhaPe(bool forcar);
extern void enviarLimiarLinhaParaPe();
extern void solicitarChuteManualkicker();
extern void enviarReferenciaBussolaParaCabeca(bool forcar);
extern void aplicarPapelConfiguradoLocal();
extern void enviarZonaTesteParaCabeca(char zona);

bool iniciarDisplayIHM() {
  return display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
}

void mostrarBootEtapaIHM(const char* etapa, uint8_t percentual) {
  if (percentual > 100) percentual = 100;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(34, 8);
  display.println("NEXUS OS");
  display.setCursor(14, 22);
  display.println("Inicializando...");

  display.drawRect(10, 38, 108, 12, SSD1306_WHITE);
  int preenchimento = (int)((104 * percentual) / 100);
  if (preenchimento > 0) {
    display.fillRect(12, 40, preenchimento, 8, SSD1306_WHITE);
  }

  display.setCursor(10, 54);
  display.print(percentual);
  display.print("% ");
  display.print(etapa);

  display.display();
}

void desenharMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("MENU PRINCIPAL");

  if (itemSelecionado == 0) {
    display.fillRect(0, 16, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 18);
    display.println("INICIAR");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(4, 18);
    display.println("INICIAR");
  }

  if (itemSelecionado == 1) {
    display.fillRect(0, 30, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 32);
    display.println("DADOS");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(4, 32);
    display.println("DADOS");
  }

  if (itemSelecionado == 2) {
    display.fillRect(0, 44, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 46);
    display.println("FUNCOES");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(4, 46);
    display.println("FUNCOES");
  }

  display.setCursor(0, 56);
  display.print("P:");
  display.print(papelAtacante ? "ATC" : "DEF");
  display.print(" ESN:");
  display.print(espnowConectadoRecente() ? "ON" : "OFF");
  display.display();
}

void desenharSubmenuFuncao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("FUNCOES");
  display.println();

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
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32);
      display.println("POSICIONA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 32);
      display.println("POSICIONA");
    }

    if (itemSubMenuFuncao == 2) {
      display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 40);
      display.println("KICKER");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 40);
      display.println("KICKER");
    }

    if (itemSubMenuFuncao == 3) {
      display.fillRect(0, 48, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 48);
      display.println(papelAtacante ? "ZONA (ver)" : "ZONA (envia)");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 48);
      display.println(papelAtacante ? "ZONA (ver)" : "ZONA (envia)");
    }

    if (itemSubMenuFuncao == 4) {
      display.fillRect(0, 56, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 56);
      display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 56);
      display.println("VOLTAR");
    }

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
      display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 40);
      display.println("VOLTAR");
    }

    display.setCursor(0, 56);
    display.println("Salva imediato");

  } else if (subMenuFuncao == SUBFUNCAO_SENSORES) {
    bool sensoresRecentes = (ultimoRxSensoresBrutosMs > 0) &&
                            ((millis() - ultimoRxSensoresBrutosMs) <= TIMEOUT_SENSORES_BRUTOS_MS_LOCAL);
    display.setCursor(0, 8);
    display.println("AMOSTRAGEM LINHA");
    display.setCursor(0, 20);
    display.print("S1 :"); display.print(sensoresRecentes ? sensorBruto1  : -1);
    display.setCursor(66, 20);
    display.print("S9 :"); display.print(sensoresRecentes ? sensorBruto9  : -1);
    display.setCursor(0, 34);
    display.print("S17:"); display.print(sensoresRecentes ? sensorBruto17 : -1);
    display.setCursor(66, 34);
    display.print("S25:"); display.print(sensoresRecentes ? sensorBruto25 : -1);
    display.setCursor(0, 56);
    display.println("OK: limiar");

  } else if (subMenuFuncao == SUBFUNCAO_LIMIAR_LINHA) {
    display.setCursor(0, 0);
    display.println("CALIBRACAO LINHA");
    display.setCursor(61, 16);
    display.println("^");
    display.setTextSize(2);
    display.setCursor(24, 26);
    display.println(limiarLinhaEditado);
    display.setTextSize(1);
    display.setCursor(61, 48);
    display.println("v");
    display.setCursor(0, 56);
    display.println("OK salva");

  } else if (subMenuFuncao == SUBFUNCAO_POSICIONAMENTO) {
    display.setCursor(0, 10);
    display.println("AGUARDANDO HTTPS");
    display.setCursor(0, 24);
    if (posicionamentoAlvoAtivo) {
      display.print("ALVO X:");
      display.print((int)posicionamentoAlvoXcm);
      display.setCursor(0, 34);
      display.print("ALVO Y:");
      display.print((int)posicionamentoAlvoYcm);
      display.setCursor(0, 56);
      display.println("OK: voltar/parar");
    } else {
      display.println("SEM ALVO");
      display.setCursor(0, 34);
      display.println("ESPERE COMANDO");
      display.setCursor(0, 56);
      display.println("OK: voltar");
    }

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
    display.println("OK confirma");

  } else if (subMenuFuncao == SUBFUNCAO_ZONA_TESTE) {
    if (papelAtacante) {
      // ATACANTE: tela somente leitura, mostra a zona recebida do parceiro.
      display.println("ZONA (recebida)");
      display.println();
      display.setTextSize(2);
      display.setCursor(24, 20);
      bool zonaRecente = (ultimoRxZonaDefensorMs > 0) &&
                         ((millis() - ultimoRxZonaDefensorMs) <= TIMEOUT_ZONA_DEFENSOR_MS);
      display.println(zonaRecente ? String(zonaDefensorRecebida) : String("-"));
      display.setTextSize(1);
      display.setCursor(0, 44);
      display.print("Status: ");
      display.println(zonaRecente ? "OK" : "SEM DADOS");
      display.setCursor(0, 56);
      display.println("Qualquer BTN volta");
    } else {
      // DEFENSOR: escolhe A/B/C/AUTO e envia para a Cabeca via ESP-NOW.
      display.println("ENVIAR ZONA TESTE");
      display.println();

      const char* opcoes[5] = {"AUTO", "A", "B", "C", "VOLTAR"};
      for (int i = 0; i < 5; i++) {
        int y = 16 + (i * 8);
        if (itemSubMenuFuncao == i) {
          display.fillRect(0, y, 128, 8, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
          display.setCursor(4, y);
          display.println(opcoes[i]);
          display.setTextColor(SSD1306_WHITE);
        } else {
          display.setCursor(4, y);
          display.println(opcoes[i]);
        }
      }
    }
  }

  display.display();
}

void desenharSubmenuCalibracao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("DADOS");
  display.println();

  if (subMenuCalibracao == SUBMENU_PRINCIPAL) {
    if (itemSubMenu == 0) {
      display.fillRect(0, 16, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 16); display.println("AMOSTRAGENS");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 16); display.println("AMOSTRAGENS"); }

    if (itemSubMenu == 1) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 24); display.println("CALIBRACOES");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 24); display.println("CALIBRACOES"); }

    if (itemSubMenu == 2) {
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32); display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 32); display.println("VOLTAR"); }

  } else if (subMenuCalibracao == SUBMENU_GOL) {
    // Amostragens
    if (itemSubMenu == 0) {
      display.fillRect(0, 16, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 16); display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 16); display.println("VOLTAR"); }

    if (itemSubMenu == 1) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 24); display.println("LINHA");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 24); display.println("LINHA"); }

    if (itemSubMenu == 2) {
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32); display.println("INFRAVERMELHO");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 32); display.println("INFRAVERMELHO"); }

    if (itemSubMenu == 3) {
      display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 40); display.println("ULTRASSONICOS");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 40); display.println("ULTRASSONICOS"); }

    if (itemSubMenu == 4) {
      display.fillRect(0, 48, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 48); display.println("CAMERA");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 48); display.println("CAMERA"); }

  } else if (subMenuCalibracao == SUBMENU_BUSSOLA) {
    // Calibracoes
    if (itemSubMenu == 0) {
      display.fillRect(0, 16, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 16); display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 16); display.println("VOLTAR"); }

    if (itemSubMenu == 1) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 24); display.println("LINHA");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 24); display.println("LINHA"); }

    if (itemSubMenu == 2) {
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32); display.println("BUSSOLA");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 32); display.println("BUSSOLA"); }

    if (itemSubMenu == 3) {
      display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 40); display.println("GOL");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 40); display.println("GOL"); }

  } else if (subMenuCalibracao == SUBMENU_BUSSOLA_AJUSTE) {
    display.println("BUSSOLA");
    display.println();
    display.print("Atual:");
    if (bussolaValida) display.println(headingBussolaTeste);
    else display.println("sem");
    display.print("Ref  :");
    display.println(headingBussolaSalvo);
    display.println("OK salva");
    display.println("BTN1/2 VOLTA");

  } else if (subMenuCalibracao == SUBMENU_GOL_CALIBRACAO) {
    display.println("DEFINIR GOL");
    display.println();

    if (itemSubMenu == 0) {
      display.fillRect(0, 16, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 16); display.println("AMARELO");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 16); display.println("AMARELO"); }

    if (itemSubMenu == 1) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 24); display.println("AZUL");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 24); display.println("AZUL"); }

    if (itemSubMenu == 2) {
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32); display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else { display.setCursor(4, 32); display.println("VOLTAR"); }

    display.setCursor(0, 56);
    display.println(corGolAzul ? "Atual: AZUL" : "Atual: AMARELO");

  } else if (subMenuCalibracao == SUBMENU_IR) {
    display.println("LINHA");
    display.println();
    display.print("Angulo:"); display.println(linhaDetectada ? String(anguloLinhaPe, 1) : String("sem"));
    int ativos = 0;
    if (sensorBruto1 > limiarLinhaEditado) ativos++;
    if (sensorBruto9 > limiarLinhaEditado) ativos++;
    if (sensorBruto17 > limiarLinhaEditado) ativos++;
    if (sensorBruto25 > limiarLinhaEditado) ativos++;
    display.print("Ativos:"); display.println(ativos);
    display.println("BTN1/2/3 VOLTA");

  } else if (subMenuCalibracao == SUBMENU_ULTRA) {
    display.println("INFRAVERMELHO");
    display.println();
    display.print("Ang:");
    if (irDetectado) { display.println(String(anguloIr, 1)); } else { display.println("sem"); }
    display.println("Int: (camera)");
    display.print("Dom:");
    display.println(sensorBruto1 > sensorBruto9 ? "S1" : "S9");
    display.println("BTN1/2/3 VOLTA");

  } else if (subMenuCalibracao == SUBMENU_CAMERA) {
    display.println("ULTRASSONICOS");
    display.println();
    display.print("Frente : "); display.print(ultraFcm, 1); display.println("cm");
    display.print("Direita: "); display.print(ultraDcm, 1); display.println("cm");
    display.print("Tras   : "); display.print(ultraTcm, 1); display.println("cm");
    display.print("Esq    : "); display.print(ultraEcm, 1); display.println("cm");

  } else if (subMenuCalibracao == SUBMENU_ESPNOW) {
    display.println("CAMERA");
    display.println();
    bool usarAzul = golReferenciaAzulEfetiva();
    display.print("Gol: "); display.println(usarAzul ? "Azul" : "Amarelo");
    int16_t angGol = usarAzul ? cameraBlueAngle : cameraYellowAngle;
    display.print("Ang: "); display.println(angGol);
    display.print("Erro:");
    if (cameraDadosValidos) display.println((int)calcularErroGolPorPapel((float)angGol));
    else display.println("sem");
    display.println("BTN1/2/3 VOLTA");
  }

  display.display();
}

void desenharOperacao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(papelAtacante ? "ATACANTE" : "DEFENSOR");

  int16_t anguloGolOperacao = -999;
  bool golVisivelOperacao = cameraTemGolSelecionadoValido(anguloGolOperacao);

  display.setCursor(0, 12);
  display.println("Bola");
  display.setCursor(42, 12);
  if (irDetectado) { display.print(anguloIr, 1); display.println(" deg"); }
  else { display.println("sem"); }

  display.setCursor(0, 24);
  display.println("Gol");
  display.setCursor(42, 24);
  if (golVisivelOperacao) {
    float erroGolMostrado = calcularErroGolPorPapel((float)anguloGolOperacao);
    display.print(erroGolMostrado, 1); display.println(" deg");
  } else {
    display.println("sem");
  }

  display.setCursor(0, 36);
  display.println("Bussola");
  display.setCursor(42, 36);
  if (bussolaValida) { display.print(headingBussolaTeste); display.println(" deg"); }
  else { display.println("sem"); }

  display.setCursor(0, 48);
  display.println("Erro");
  display.setCursor(42, 48);
  if (golVisivelOperacao) {
    display.print(erroAlinhamentoGraus, 1); display.println(" deg");
  } else {
    display.println("---");
  }

  display.display();
}

void desenharStatusPlacas() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("DIAGNOSTICO");
  display.println();
  display.print("Superior.... "); display.println(olhoOK ? "OK" : "FALHA");
  display.print("Inferior.... "); display.println(peOK ? "OK" : "FALHA");
  display.print("OpenMV...... "); display.println(cameraDadosValidos ? "OK" : "FALHA");
  display.print("Bussola..... "); display.println(bussolaValida ? "OK" : "FALHA");
  display.display();
}

void mostrarTelaFalhaComunicacao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("COMUNICACAO");
  display.println();
  display.println("CAB: FALHA");
  display.println("Verifique serial");
  display.print("Ultimo: ");
  display.println(mensagemBotao);
  display.display();
}

void desenharTelaAtual() {
  if (millis() < mostrarStatusAte) {
    desenharStatusPlacas();
    return;
  }

  if (!comunicacaoCabecaOK) {
    mostrarTelaFalhaComunicacao();
    return;
  }

  if (estadoAtual == MENU) {
    desenharMenu();
  } else if (estadoAtual == CALIBRACAO) {
    desenharSubmenuCalibracao();
  } else if (estadoAtual == FUNCAO) {
    desenharSubmenuFuncao();
  } else {
    desenharOperacao();
  }
}

void processarEventoBotao(uint8_t botao) {
  if (estadoAtual == MENU) {
    if (botao == 1) {
      itemSelecionado--;
      if (itemSelecionado < 0) itemSelecionado = 2;
    } else if (botao == 2) {
      itemSelecionado++;
      if (itemSelecionado > 2) itemSelecionado = 0;
    } else if (botao == 3) {
      if (itemSelecionado == 0) {
        estadoAtual = INICIAR;
        atualizarSozinhoLocal();
        enviarEstadoJogoParaCabeca(true);
      } else if (itemSelecionado == 1) {
        estadoAtual = CALIBRACAO;
        subMenuCalibracao = SUBMENU_PRINCIPAL;
        itemSubMenu = 0;
      } else {
        estadoAtual = FUNCAO;
        subMenuFuncao = SUBFUNCAO_PRINCIPAL;
        itemSubMenuFuncao = 0;
      }
    }
    return;
  }

  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_PRINCIPAL) {
    if (botao == 1) {
      itemSubMenuFuncao--;
      if (itemSubMenuFuncao < 0) itemSubMenuFuncao = 4;
    } else if (botao == 2) {
      itemSubMenuFuncao++;
      if (itemSubMenuFuncao > 4) itemSubMenuFuncao = 0;
    } else if (botao == 3) {
      if (itemSubMenuFuncao == 0) {
        subMenuFuncao = SUBFUNCAO_PAPEIS;
        if (papelConfiguradoMenu == PAPEL_CONFIG_ATACANTE) itemSubMenuFuncao = 0;
        else if (papelConfiguradoMenu == PAPEL_CONFIG_DEFENSOR) itemSubMenuFuncao = 1;
        else itemSubMenuFuncao = 2;
      } else if (itemSubMenuFuncao == 1) {
        subMenuFuncao = SUBFUNCAO_POSICIONAMENTO;
        itemSubMenuFuncao = 0;
      } else if (itemSubMenuFuncao == 2) {
        subMenuFuncao = SUBFUNCAO_KICKER;
        itemSubMenuFuncao = 0;
      } else if (itemSubMenuFuncao == 3) {
        subMenuFuncao = SUBFUNCAO_ZONA_TESTE;
        itemSubMenuFuncao = 0;
      } else {
        estadoAtual = MENU;
        itemSelecionado = 2;
      }
    }
    return;
  }

  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_PAPEIS) {
    if (botao == 1) {
      itemSubMenuFuncao--;
      if (itemSubMenuFuncao < 0) itemSubMenuFuncao = 3;
    } else if (botao == 2) {
      itemSubMenuFuncao++;
      if (itemSubMenuFuncao > 3) itemSubMenuFuncao = 0;
    } else if (botao == 3) {
      if (itemSubMenuFuncao == 0) {
        papelConfiguradoMenu = PAPEL_CONFIG_ATACANTE;
        aplicarPapelConfiguradoLocal();
        enviarPapelAtualParaCabeca();
        mensagemBotao = salvarPapelConfiguradoEEPROM() ? "PAPEL FIXO ATC" : "ERRO EEPROM";
      } else if (itemSubMenuFuncao == 1) {
        papelConfiguradoMenu = PAPEL_CONFIG_DEFENSOR;
        aplicarPapelConfiguradoLocal();
        enviarPapelAtualParaCabeca();
        mensagemBotao = salvarPapelConfiguradoEEPROM() ? "PAPEL FIXO DEF" : "ERRO EEPROM";
      } else if (itemSubMenuFuncao == 2) {
        papelConfiguradoMenu = PAPEL_CONFIG_AUTO;
        mensagemBotao = salvarPapelConfiguradoEEPROM() ? "PAPEL AUTO" : "ERRO EEPROM";
      }
      subMenuFuncao = SUBFUNCAO_PRINCIPAL;
      itemSubMenuFuncao = 0;
    }
    return;
  }

  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_SENSORES) {
    if (botao == 1 || botao == 2) {
      solicitarSensoresBrutosPe(true);
    } else if (botao == 3) {
      subMenuFuncao = SUBFUNCAO_LIMIAR_LINHA;
      limiarLinhaSincronizado = false;
      entradaTelaLimiarMs = millis();
      solicitarLimiarLinhaPe(true);
    }
    return;
  }

  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_LIMIAR_LINHA) {
    if (botao == 1) {
      limiarLinhaEditado += LIMIAR_LINHA_PASSO_LOCAL;
      if (limiarLinhaEditado > LIMIAR_LINHA_MAX_LOCAL) limiarLinhaEditado = LIMIAR_LINHA_MAX_LOCAL;
      limiarLinhaSincronizado = true;
    } else if (botao == 2) {
      limiarLinhaEditado -= LIMIAR_LINHA_PASSO_LOCAL;
      if (limiarLinhaEditado < LIMIAR_LINHA_MIN_LOCAL) limiarLinhaEditado = LIMIAR_LINHA_MIN_LOCAL;
      limiarLinhaSincronizado = true;
    } else if (botao == 3) {
      enviarLimiarLinhaParaPe();
      mensagemBotao = "LIMIAR SALVO";
      subMenuFuncao = SUBFUNCAO_PRINCIPAL;
      itemSubMenuFuncao = 1;
    }
    return;
  }

  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_POSICIONAMENTO) {
    if (botao == 3) {
      subMenuFuncao = SUBFUNCAO_PRINCIPAL;
      itemSubMenuFuncao = 1;
    }
    return;
  }

  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_KICKER) {
    if (botao == 1 || botao == 2) {
      itemSubMenuFuncao = 1 - itemSubMenuFuncao;
    } else if (botao == 3) {
      if (itemSubMenuFuncao == 0) {
        solicitarChuteManualkicker();
        mensagemBotao = "CHUTE MANUAL";
      } else {
        subMenuFuncao = SUBFUNCAO_PRINCIPAL;
        itemSubMenuFuncao = 2;
      }
    }
    return;
  }

  if (estadoAtual == FUNCAO && subMenuFuncao == SUBFUNCAO_ZONA_TESTE) {
    if (papelAtacante) {
      // ATACANTE: tela so de leitura, qualquer botao volta.
      if (botao == 1 || botao == 2 || botao == 3) {
        subMenuFuncao = SUBFUNCAO_PRINCIPAL;
        itemSubMenuFuncao = 3;
      }
      return;
    }

    // DEFENSOR: navega entre AUTO/A/B/C/VOLTAR e confirma com BTN3.
    if (botao == 1) {
      itemSubMenuFuncao--;
      if (itemSubMenuFuncao < 0) itemSubMenuFuncao = 4;
    } else if (botao == 2) {
      itemSubMenuFuncao++;
      if (itemSubMenuFuncao > 4) itemSubMenuFuncao = 0;
    } else if (botao == 3) {
      if (itemSubMenuFuncao == 4) {
        subMenuFuncao = SUBFUNCAO_PRINCIPAL;
        itemSubMenuFuncao = 3;
      } else {
        const char opcoes[4] = {'0', 'A', 'B', 'C'};  // '0' = AUTO
        char zona = opcoes[itemSubMenuFuncao];
        enviarZonaTesteParaCabeca(zona);
        mensagemBotao = (zona == '0') ? "ZONA: AUTO ENVIADA" : ("ZONA " + String(zona) + " ENVIADA");
      }
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_PRINCIPAL) {
    if (botao == 1) {
      itemSubMenu--;
      if (itemSubMenu < 0) itemSubMenu = 2;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 2) itemSubMenu = 0;
    } else if (botao == 3) {
      if (itemSubMenu == 0) {
        subMenuCalibracao = SUBMENU_GOL;
        itemSubMenu = 0;
      } else if (itemSubMenu == 1) {
        subMenuCalibracao = SUBMENU_BUSSOLA;
        itemSubMenu = 0;
      } else {
        estadoAtual = MENU;
        itemSelecionado = 1;
      }
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_GOL) {
    if (botao == 1) {
      itemSubMenu--;
      if (itemSubMenu < 0) itemSubMenu = 4;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 4) itemSubMenu = 0;
    } else if (botao == 3) {
      if (itemSubMenu == 0) {
        subMenuCalibracao = SUBMENU_PRINCIPAL;
        itemSubMenu = 0;
      } else if (itemSubMenu == 1) {
        subMenuCalibracao = SUBMENU_IR;
      } else if (itemSubMenu == 2) {
        subMenuCalibracao = SUBMENU_ULTRA;
      } else if (itemSubMenu == 3) {
        subMenuCalibracao = SUBMENU_CAMERA;
      } else if (itemSubMenu == 4) {
        subMenuCalibracao = SUBMENU_ESPNOW;
      }
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_BUSSOLA) {
    if (botao == 1) {
      itemSubMenu--;
      if (itemSubMenu < 0) itemSubMenu = 3;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 3) itemSubMenu = 0;
    } else if (botao == 3) {
      if (itemSubMenu == 0) {
        subMenuCalibracao = SUBMENU_PRINCIPAL;
        itemSubMenu = 1;
      } else if (itemSubMenu == 1) {
        subMenuFuncao = SUBFUNCAO_LIMIAR_LINHA;
        estadoAtual = FUNCAO;
        limiarLinhaSincronizado = false;
        entradaTelaLimiarMs = millis();
        solicitarLimiarLinhaPe(true);
      } else if (itemSubMenu == 2) {
        subMenuCalibracao = SUBMENU_BUSSOLA_AJUSTE;
      } else if (itemSubMenu == 3) {
        subMenuCalibracao = SUBMENU_GOL_CALIBRACAO;
        itemSubMenu = corGolAzul ? 1 : 0;
      }
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_BUSSOLA_AJUSTE) {
    if (botao == 3) {
      headingBussolaSalvo = headingBussolaTeste;
      EEPROM.put(EEPROM_ADDR_BUSSOLA_LOCAL, headingBussolaSalvo);
      bool ok = EEPROM.commit();
      if (ok) {
        enviarReferenciaBussolaParaCabeca(true);
        mensagemBotao = "BUSSOLA GRAVADA";
      } else {
        mensagemBotao = "ERRO EEPROM";
      }
      subMenuCalibracao = SUBMENU_BUSSOLA;
      itemSubMenu = 2;
    } else if (botao == 1 || botao == 2) {
      subMenuCalibracao = SUBMENU_BUSSOLA;
      itemSubMenu = 2;
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_GOL_CALIBRACAO) {
    if (botao == 1) {
      itemSubMenu--;
      if (itemSubMenu < 0) itemSubMenu = 2;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 2) itemSubMenu = 0;
    } else if (botao == 3) {
      if (itemSubMenu == 0 || itemSubMenu == 1) {
        corGolAzul = (itemSubMenu == 1);
        corGolPendenteEnvio = true;
        bool ok = salvarCorGolEEPROM();
        mensagemBotao = ok ? (corGolAzul ? "GOL AZUL SALVO" : "GOL AMARELO SALVO") : "ERRO EEPROM";
      }
      subMenuCalibracao = SUBMENU_BUSSOLA;
      itemSubMenu = 3;
    }
    return;
  }

  if (estadoAtual == CALIBRACAO &&
      (subMenuCalibracao == SUBMENU_IR || subMenuCalibracao == SUBMENU_ULTRA ||
       subMenuCalibracao == SUBMENU_CAMERA || subMenuCalibracao == SUBMENU_ESPNOW)) {
    if (botao == 1 || botao == 2 || botao == 3) {
      subMenuCalibracao = SUBMENU_GOL;
      itemSubMenu = 0;
    }
    return;
  }

  if (estadoAtual == INICIAR && botao == 3) {
    estadoAtual = MENU;
    enviarEstadoJogoParaCabeca(true);
  }
}
