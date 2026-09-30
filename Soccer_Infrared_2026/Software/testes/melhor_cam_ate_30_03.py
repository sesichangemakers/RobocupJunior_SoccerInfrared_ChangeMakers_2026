# Script de teste da camera (versao de referencia).
# Funcao: detectar gol por blob de cor, calcular erro angular e enviar
# pacote para a placa Olho por UART no protocolo definido.
# Entrada: frames da camera e comando de cor vindo do robo.
# Saida: pacote com gol_detectado, erro e quantidade de pixels.
import sensor
import time
import math
import struct
from machine import UART

# ===== COMUNICAÇÃO SERIAL COM OLHO =====
# UART1: TX=P3, RX=P4 (pinos padrão RT1062)
uart_olho = UART(1, 19200, timeout_char=200)

# ===== PROTOCOLO =====
BYTE_INICIA = 0xAA
BYTE_PARA = 0x55
ID_PLACA_CAMERA = 0x03  # Identificador único da câmera

# ===== CONFIGURAÇÃO =====
R = 105
cx = 155
cy = 133
R2 = R * R
# ========================

# ===== SELETOR DE COR (será recebido da OLHO) =====
AZUL = False  # Padrão: amarelo. Será atualizado via serial

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

# Ponto fixo
goal_x = 155
goal_y = 115

# Thresholds (LAB)
blue_threshold   = (40, 70, -15, 15, -40, -5)
yellow_threshold = (70, 100, -10, 30, 20, 50)

# ===== FILTRO DE "GOL" =====
MIN_GOAL_PIXELS = 400
MIN_GOAL_AREA   = 100

# ===== LIMITADOR DE FPS =====
TARGET_FPS = 20
FRAME_MS = int(1000 / TARGET_FPS)

# ===== VARIÁVEIS DE RESULTADO =====
gol_detectado = False
erro_gol = 0
pixels_detectados = 0


def enviar_dados_gol():
    """Envia dados do gol detectado para a placa Olho"""
    # Estrutura: [gol_detectado (bool=1B)] [erro_gol (int16=2B)] [pixels (uint16=2B)]
    # > = big-endian, b = int8, h = int16, H = uint16
    erro_raw = int(erro_gol * 10)
    if erro_raw > 32767:
        erro_raw = 32767
    elif erro_raw < -32768:
        erro_raw = -32768

    if pixels_detectados > 65535:
        pixels_raw = 65535
    elif pixels_detectados < 0:
        pixels_raw = 0
    else:
        pixels_raw = pixels_detectados

    msg = struct.pack(">bhH", 1 if gol_detectado else 0, erro_raw, pixels_raw)
    
    # Framing: BYTE_INICIA + ID + dados + BYTE_PARA
    buffer = bytearray(1 + 1 + 5 + 1)
    buffer[0] = BYTE_INICIA
    buffer[1] = ID_PLACA_CAMERA
    buffer[2:7] = msg
    buffer[7] = BYTE_PARA
    
    uart_olho.write(buffer)


def receber_selecao_cor():
    """Recebe bool AZUL da placa Olho"""
    global AZUL

    # Descarta bytes ate encontrar BYTE_INICIA
    while uart_olho.any():
        b = uart_olho.read(1)
        if b is None:
            break
        if b[0] == BYTE_INICIA:
            # Pacote: [ID(1)] [bool_azul(1)] [BYTE_PARA(1)] = 3 bytes restantes
            pacote = uart_olho.read(3)
            if pacote is None or len(pacote) < 3:
                break
            id_placa = pacote[0]
            bool_azul = pacote[1]
            byte_para = pacote[2]
            if id_placa == 0x01 and byte_para == BYTE_PARA:
                AZUL = bool(bool_azul)
                print("COR ATUALIZADA: %s" % ("AZUL" if AZUL else "AMARELO"))
            break


while True:
    clock.tick()
    t0 = time.ticks_ms()

    # Verifica se recebeu nova seleção de cor
    receber_selecao_cor()

    img = sensor.snapshot()

    # Máscara quadrada externa
    left = cx - R
    right = cx + R
    top = cy - R
    bottom = cy + R

    img.draw_rectangle(0, 0, img.width(), top, color=(0, 0, 0), fill=True)
    img.draw_rectangle(0, bottom, img.width(), img.height() - bottom, color=(0, 0, 0), fill=True)
    img.draw_rectangle(0, top, left, bottom - top, color=(0, 0, 0), fill=True)
    img.draw_rectangle(right, top, img.width() - right, bottom - top, color=(0, 0, 0), fill=True)

    # Seleciona threshold ativo baseado na cor
    active_threshold = blue_threshold if AZUL else yellow_threshold

    # Detecta blobs da cor ativa
    blobs = img.find_blobs([active_threshold], pixels_threshold=50, area_threshold=50)

    # Escolhe maior blob dentro do círculo
    best_blob = None
    best_pixels = 0

    for blob in blobs:
        dx = blob.cx() - cx
        dy = blob.cy() - cy

        if (dx * dx + dy * dy) <= R2:
            p = blob.pixels()
            if p > best_pixels:
                best_pixels = p
                best_blob = blob

    # Processa detecção
    if (best_blob is not None) and (best_pixels >= MIN_GOAL_PIXELS) and (best_blob.area() >= MIN_GOAL_AREA):
        gol_detectado = True
        pixels_detectados = best_pixels

        img.draw_rectangle(best_blob.rect(), color=(0, 255, 0), thickness=2)
        img.draw_cross(best_blob.cx(), best_blob.cy(), color=(255, 0, 0))
        img.draw_line(cx, cy, best_blob.cx(), best_blob.cy(), color=(255, 255, 0), thickness=2)

        # Ângulo do ponto fixo em relação ao centro
        dx_ref = goal_x - cx
        dy_ref = goal_y - cy
        ang_ref = math.degrees(math.atan2(dy_ref, dx_ref))

        # Ângulo do blob em relação ao centro
        dx_blob = best_blob.cx() - cx
        dy_blob = best_blob.cy() - cy
        ang_blob = math.degrees(math.atan2(dy_blob, dx_blob))

        erro_gol = ang_blob - ang_ref

        # Normaliza para -180 a 180
        if erro_gol > 180:
            erro_gol -= 360
        elif erro_gol < -180:
            erro_gol += 360

        cor_txt = "AZUL" if AZUL else "AMARELO"
        img.draw_string(5, 5, "COR: %s" % cor_txt, color=(255, 255, 255))
        img.draw_string(5, 20, "Erro: %.1f" % erro_gol, color=(255, 255, 0))
        img.draw_string(5, 35, "Pixels: %d" % best_pixels, color=(255, 255, 0))

    else:
        gol_detectado = False
        pixels_detectados = 0
        erro_gol = 0

        cor_txt = "AZUL" if AZUL else "AMARELO"
        img.draw_string(5, 5, "COR: %s" % cor_txt, color=(255, 255, 255))
        img.draw_string(5, 20, "SEM GOL", color=(255, 0, 0))
        img.draw_string(5, 35, "MaxPix: %d" % best_pixels, color=(255, 0, 0))

    # Desenha referências
    img.draw_circle(cx, cy, R, color=(0, 255, 0), thickness=2)
    img.draw_circle(cx, cy, 5, color=(0, 255, 0), thickness=2)
    img.draw_circle(goal_x, goal_y, 3, color=(0, 255, 0), thickness=2)

    # ===== ENVIA DADOS PARA OLHO =====
    enviar_dados_gol()

    print("FPS: %d | GOL: %s | ERRO: %.1f | PIXELS: %d" %
          (clock.fps(), ("SIM" if gol_detectado else "NÃO"), erro_gol, pixels_detectados))

    # ===== Limita FPS =====
    dt = time.ticks_diff(time.ticks_ms(), t0)
    if dt < FRAME_MS:
        time.sleep_ms(FRAME_MS - dt)
