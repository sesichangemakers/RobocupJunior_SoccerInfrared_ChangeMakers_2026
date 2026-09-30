#pragma once

#include <Arduino.h>

bool iniciarDisplayIHM();
void mostrarBootEtapaIHM(const char* etapa, uint8_t percentual);

void desenharTelaAtual();
void processarEventoBotao(uint8_t botao);
