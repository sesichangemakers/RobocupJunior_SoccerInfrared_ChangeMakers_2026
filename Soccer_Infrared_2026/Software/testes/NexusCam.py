import sensor
import time
import math
import struct
from machine import UART
from machine import LED

# ===== COMUNICAÇÃO SERIAL COM OLHO =====
uart_olho = UART(1, 115200, timeout_char=200)

# ===== PROTOCOLO SERIAL =====
BYTE_INICIA = 0xAA
BYTE_PARA = 0x55
ID_PLACA_CAMERA = 0x03

# =========================================================
# CONFIGURAÇÕES GERAIS
# =========================================================

R = 105
cx = 172
cy = 110
R2 = R * R

DEBUG = True
center = [170, 120]

# =========================================================
# CRIAÇÃO DOS LEDS
# =========================================================

red_led = LED("LED_RED")
green_led = LED("LED_GREEN")
blue_led = LED("LED_BLUE")

# =========================================================
# FILTRO ANTI-RUIDO DA BOLA
# =========================================================
BALL_FILTER_WINDOW = 5
BALL_FILTER_OUTLIER_DEG = 70
BALL_FILTER_HOLD_FRAMES = 3

ball_angle_hist = []
ball_dist_hist = []
ball_filtered_valid = False
ball_filtered_angle = 0
ball_filtered_dist = 0
ball_lost_frames = 0

# =========================================================
# FILTRO DE PRESENÇA DA BOLA
# =========================================================
BALL_PRESENCE_WINDOW = 10
BALL_PRESENCE_MIN_VALID = 8
ball_presence_hist = []

# =========================================================
# ZONA DA BOLA
# =========================================================
BALL_ZONE_INNER = 25
BALL_ZONE_OUTER = 90

BALL_ZONE_INNER2 = BALL_ZONE_INNER * BALL_ZONE_INNER
BALL_ZONE_OUTER2 = BALL_ZONE_OUTER * BALL_ZONE_OUTER

# =========================================================
# ZONA DOS GOLS
# =========================================================
GOAL_ZONE_OUTER = 120
GOAL_ZONE_OUTER2 = GOAL_ZONE_OUTER * GOAL_ZONE_OUTER

# =========================================================
# ZONA DE EXCLUSÃO DA BOLA AO REDOR DO GOL AMARELO
# =========================================================
YELLOW_EXCLUSION_MARGIN_X = 5
YELLOW_EXCLUSION_MARGIN_Y = 5

# =========================================================
# FUNÇÕES AUXILIARES
# =========================================================

def calc_goal_angle_and_distance(x, y, center_x, center_y):
    """
    GOLS:
    Mantém a lógica antiga:
    - ângulo em faixa negativa/positiva
    """
    dx = x - center_x
    dy = y - center_y

    angle = -((((math.atan2(dx, dy) * 180) / math.pi) + 360) % 360 - 180)
    distance = math.sqrt(dx**2 + dy**2)

    return int(angle), int(distance)


def calc_ball_angle_and_distance(x, y, center_x, center_y):
    """
    BOLA:
    - 0 a 359 graus
    - corrigida para compensar a imagem espelhada
    """
    dx = x - center_x
    dy = y - center_y

    angle = (360 - ((math.degrees(math.atan2(dx, dy)) + 360) % 360) + 180) % 360

    # corrige o espelhamento lateral da imagem
    angle = (360 - angle) % 360

    distance = math.sqrt(dx**2 + dy**2)

    return int(angle), int(distance)


def is_in_ball_zone(x, y):
    dx = x - center[0]
    dy = y - center[1]
    d2 = dx * dx + dy * dy
    return (d2 >= BALL_ZONE_INNER2) and (d2 <= BALL_ZONE_OUTER2)


def is_in_goal_zone(x, y):
    dx = x - center[0]
    dy = y - center[1]
    d2 = dx * dx + dy * dy
    return (d2 <= GOAL_ZONE_OUTER2)


def is_inside_yellow_exclusion_zone(x, y, yellow_blob, margin_x=25, margin_y=25):
    if yellow_blob is None:
        return False

    zx = yellow_blob.x() - margin_x
    zy = yellow_blob.y() - margin_y
    zw = yellow_blob.w() + (2 * margin_x)
    zh = yellow_blob.h() + (2 * margin_y)

    return (x >= zx) and (x <= zx + zw) and (y >= zy) and (y <= zy + zh)


def find_best_ball_blob_in_zone(img, threshold, pixels_threshold, area_threshold,
                                merge, margin, yellow_blob=None,
                                min_pixels=None, max_pixels=None):
    best_blob = None
    best_perimeter = 0

    for blob in img.find_blobs([threshold],
                               pixels_threshold=pixels_threshold,
                               area_threshold=area_threshold,
                               merge=merge,
                               margin=margin):

        px = blob.pixels()

        if min_pixels is not None and px < min_pixels:
            continue
        if max_pixels is not None and px > max_pixels:
            continue

        if not is_in_ball_zone(blob.cx(), blob.cy()):
            continue

        if is_inside_yellow_exclusion_zone(blob.cx(), blob.cy(), yellow_blob,
                                           YELLOW_EXCLUSION_MARGIN_X,
                                           YELLOW_EXCLUSION_MARGIN_Y):
            continue

        if blob.perimeter() > best_perimeter:
            best_perimeter = blob.perimeter()
            best_blob = blob

    return best_blob


def find_best_goal_blob_in_zone(img, threshold, pixels_threshold, area_threshold,
                                merge, margin):
    best_blob = None
    best_pixels = 0

    for blob in img.find_blobs([threshold],
                               pixels_threshold=pixels_threshold,
                               area_threshold=area_threshold,
                               merge=merge,
                               margin=margin):

        px = blob.pixels()

        if not is_in_goal_zone(blob.cx(), blob.cy()):
            continue

        if px > best_pixels:
            best_pixels = px
            best_blob = blob

    return best_blob


def draw_blob_info(img, blob, color_rgb, label, angle, distance, extra_text=""):
    img.draw_rectangle(blob.rect(), color=color_rgb, thickness=2)
    img.draw_cross(blob.cx(), blob.cy(), color=color_rgb, size=10, thickness=2)

    text1 = "{} A:{} D:{}".format(label, angle, distance)
    img.draw_string(blob.x(), max(blob.y() - 20, 0), text1, color=color_rgb, scale=1)

    if extra_text:
        img.draw_string(blob.x(), max(blob.y() - 8, 0), extra_text, color=color_rgb, scale=1)


def angle_diff_deg(a, b):
    """Menor diferenca angular assinada entre dois angulos (graus)."""
    return ((a - b + 540) % 360) - 180


def circular_mean_deg(values):
    """Media circular para angulos em graus (0..359)."""
    sx = 0.0
    sy = 0.0
    for v in values:
        r = math.radians(v)
        sx += math.cos(r)
        sy += math.sin(r)
    if sx == 0 and sy == 0:
        return 0
    return int((math.degrees(math.atan2(sy, sx)) + 360) % 360)


def median_int(values):
    if not values:
        return 0
    s = sorted(values)
    n = len(s)
    m = n // 2
    if n % 2 == 1:
        return int(s[m])
    return int((s[m - 1] + s[m]) / 2)


def update_ball_presence_filter(found):
    global ball_presence_hist

    ball_presence_hist.append(1 if found else 0)

    if len(ball_presence_hist) > BALL_PRESENCE_WINDOW:
        ball_presence_hist.pop(0)

    valid_count = sum(ball_presence_hist)

    if len(ball_presence_hist) < BALL_PRESENCE_WINDOW:
        return 0, valid_count

    if valid_count >= BALL_PRESENCE_MIN_VALID:
        return 1, valid_count

    return 0, valid_count


def update_ball_filter(found, raw_angle, raw_dist):
    """Filtra ruido da bola com janela temporal + rejeicao de outlier + hold curto."""
    global ball_angle_hist, ball_dist_hist
    global ball_filtered_valid, ball_filtered_angle, ball_filtered_dist, ball_lost_frames

    if found:
        ball_lost_frames = 0

        accept_sample = True
        if ball_filtered_valid and len(ball_angle_hist) >= 3:
            if abs(angle_diff_deg(raw_angle, ball_filtered_angle)) > BALL_FILTER_OUTLIER_DEG:
                # Salto brusco isolado e tratado como ruido.
                accept_sample = False

        if accept_sample:
            ball_angle_hist.append(raw_angle)
            ball_dist_hist.append(raw_dist)

            if len(ball_angle_hist) > BALL_FILTER_WINDOW:
                ball_angle_hist.pop(0)
            if len(ball_dist_hist) > BALL_FILTER_WINDOW:
                ball_dist_hist.pop(0)

            ball_filtered_angle = circular_mean_deg(ball_angle_hist)
            ball_filtered_dist = median_int(ball_dist_hist)
            ball_filtered_valid = True

    else:
        ball_lost_frames += 1
        if ball_lost_frames > BALL_FILTER_HOLD_FRAMES:
            ball_angle_hist = []
            ball_dist_hist = []
            ball_filtered_valid = False
            ball_filtered_angle = 0
            ball_filtered_dist = 0

    if ball_filtered_valid:
        return 1, ball_filtered_angle, ball_filtered_dist
    return 0, 0, 0


def enviar_dados_visao(ball_angle, ball_dist, blue_angle, blue_dist, yellow_angle, yellow_dist):
    """
    Envia dados de BOLA + 2 GOLS via serial para placa OLHO
    Estrutura: [BALL_A(h)][BALL_D(H)][BLUE_A(h)][BLUE_D(H)][YELLOW_A(h)][YELLOW_D(H)]
    h = int16 (ângulo), H = uint16 (distância)
    Total: 2+2+2+2+2+2 = 12 bytes de dados
    """
    # Normaliza ângulos para range -180 a 180 (compatível com int16)
    ball_a_raw = int(ball_angle) if ball_angle is not None else -999
    ball_d_raw = int(ball_dist) if ball_dist is not None else 0
    blue_a_raw = int(blue_angle) if blue_angle is not None else -999
    blue_d_raw = int(blue_dist) if blue_dist is not None else 0
    yellow_a_raw = int(yellow_angle) if yellow_angle is not None else -999
    yellow_d_raw = int(yellow_dist) if yellow_dist is not None else 0

    # Limita valores para range válido de int16/uint16
    ball_a_raw = max(-32768, min(32767, ball_a_raw))
    ball_d_raw = max(0, min(65535, ball_d_raw))
    blue_a_raw = max(-32768, min(32767, blue_a_raw))
    blue_d_raw = max(0, min(65535, blue_d_raw))
    yellow_a_raw = max(-32768, min(32767, yellow_a_raw))
    yellow_d_raw = max(0, min(65535, yellow_d_raw))

    # Empacota dados: > = big-endian, h = int16, H = uint16
    msg = struct.pack(">hHhHhH",
                      ball_a_raw, ball_d_raw,
                      blue_a_raw, blue_d_raw,
                      yellow_a_raw, yellow_d_raw)

    # Framing: BYTE_INICIA + ID + dados(12) + BYTE_PARA
    buffer = bytearray(1 + 1 + 12 + 1)
    buffer[0] = BYTE_INICIA
    buffer[1] = ID_PLACA_CAMERA
    buffer[2:14] = msg
    buffer[14] = BYTE_PARA

    uart_olho.write(buffer)


# =========================================================
# THRESHOLDS
# =========================================================

thresholdb = [9, 25, -50, 10, -20, -8]   # azul
thresholdy = [65, 45, 3, 20, 45, 5]      # amarelo
thresholdo = [35, 50, -7, 20, 9, 18]     # laranja

# =========================================================
# CÂMERA
# =========================================================

red_led.on()
blue_led.off()
green_led.off()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)

sensor.set_auto_whitebal(False, rgb_gain_db=(62, 60, 64))
sensor.set_auto_exposure(False, exposure_us=25000)
sensor.set_auto_gain(False, gain_db=20)

print("RGB gain:", sensor.get_rgb_gain_db())
print("Exposure:", sensor.get_exposure_us())
print("Gain:", sensor.get_gain_db())

sensor.skip_frames(time=1000)

clock = time.clock()
print("Iniciando detecção...")

# =========================================================
# LOOP PRINCIPAL
# =========================================================

while True:

    red_led.off()
    blue_led.off()
    green_led.on()

    clock.tick()
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

    # DEBUG DAS ZONAS
    if DEBUG:
        img.draw_cross(center[0], center[1], color=(255, 255, 255), size=12)
        img.draw_circle(center[0], center[1], BALL_ZONE_INNER, color=(80, 80, 80))
        img.draw_circle(center[0], center[1], BALL_ZONE_OUTER, color=(255, 140, 0))
        img.draw_circle(center[0], center[1], GOAL_ZONE_OUTER, color=(0, 255, 0))

    # GOL AZUL
    blue_blob = find_best_goal_blob_in_zone(
        img,
        threshold=thresholdb,
        pixels_threshold=150,
        area_threshold=150,
        merge=True,
        margin=10
    )

    blue_found = 0
    blue_angle = 0
    blue_dist = 0

    if blue_blob is not None:
        blue_found = 1
        blue_angle, blue_dist = calc_goal_angle_and_distance(
            blue_blob.cx(), blue_blob.cy(), center[0], center[1]
        )

        if DEBUG:
            extra = "ZG:1 PX:{} AR:{}".format(blue_blob.pixels(), blue_blob.area())
            draw_blob_info(img, blue_blob, (0, 0, 255), "BLUE", blue_angle, blue_dist, extra)

    # GOL AMARELO
    yellow_blob = find_best_goal_blob_in_zone(
        img,
        threshold=thresholdy,
        pixels_threshold=50,
        area_threshold=1,
        merge=True,
        margin=10
    )

    yellow_found = 0
    yellow_angle = 0
    yellow_dist = 0

    if yellow_blob is not None:
        yellow_found = 1
        yellow_angle, yellow_dist = calc_goal_angle_and_distance(
            yellow_blob.cx(), yellow_blob.cy(), center[0], center[1]
        )

        if DEBUG:
            extra = "ZG:1 PX:{} AR:{}".format(yellow_blob.pixels(), yellow_blob.area())
            draw_blob_info(img, yellow_blob, (255, 255, 0), "YELL", yellow_angle, yellow_dist, extra)

            zx = yellow_blob.x() - YELLOW_EXCLUSION_MARGIN_X
            zy = yellow_blob.y() - YELLOW_EXCLUSION_MARGIN_Y
            zw = yellow_blob.w() + (2 * YELLOW_EXCLUSION_MARGIN_X)
            zh = yellow_blob.h() + (2 * YELLOW_EXCLUSION_MARGIN_Y)
            img.draw_rectangle(zx, zy, zw, zh, color=(255, 0, 255), thickness=2)

    # BOLA
    orange_blob = find_best_ball_blob_in_zone(
        img,
        threshold=thresholdo,
        pixels_threshold=10,
        area_threshold=10,
        merge=True,
        margin=3,
        yellow_blob=yellow_blob,
        min_pixels=5,
        max_pixels=100
    )

    raw_orange_found = 0
    raw_orange_angle = 0
    raw_orange_dist = 0

    if orange_blob is not None:
        raw_orange_found = 1
        raw_orange_angle, raw_orange_dist = calc_ball_angle_and_distance(
            orange_blob.cx(), orange_blob.cy(), center[0], center[1]
        )

    presence_ok, valid_count = update_ball_presence_filter(raw_orange_found)

    if not presence_ok:
        raw_orange_found = 0

    orange_found, orange_angle, orange_dist = update_ball_filter(
        raw_orange_found,
        raw_orange_angle,
        raw_orange_dist
    )

    if DEBUG and orange_blob is not None:
        extra = "FLT PX:{} AR:{} PR:{} PV:{}/{}".format(
            orange_blob.pixels(),
            orange_blob.area(),
            int(orange_blob.perimeter()),
            valid_count,
            BALL_PRESENCE_WINDOW
        )
        draw_blob_info(img, orange_blob, (255, 140, 0), "BALL", orange_angle, orange_dist, extra)

    # ===== ENVIA DADOS VIA SERIAL =====
    enviar_dados_visao(orange_angle, orange_dist, blue_angle, blue_dist, yellow_angle, yellow_dist)

    # DEBUG NO TERMINAL
    print("FPS: {:.2f} | BALL:{} A:{} D:{} | BALLV:{}/{} | BLUE:{} A:{} D:{} | YELL:{} A:{} D:{}".format(
        clock.fps(),
        orange_found, orange_angle, orange_dist,
        valid_count, BALL_PRESENCE_WINDOW,
        blue_found, blue_angle, blue_dist,
        yellow_found, yellow_angle, yellow_dist
    ))
