"""
Controleur verin electrique - ESP32-S3
Protection par detection de surintensite via R_IS/L_IS du BTS7960
(pas de fin de course physique disponible)
"""

import network
import socket
from machine import PWM, Pin, ADC
import time

SSID = "pilot"
PASSWORD = "gui12345"

# ---------------------------------------------------------------------------
# PINOUT
# ---------------------------------------------------------------------------
adc_r_is = ADC(Pin(1))
adc_l_is = ADC(Pin(2))
adc_r_is.atten(ADC.ATTN_11DB)   # plage 0-3.3V approx
adc_l_is.atten(ADC.ATTN_11DB)

RPWM = PWM(Pin(4), freq=1000, duty=0)
LPWM = PWM(Pin(5), freq=1000, duty=0)
R_EN = Pin(6, Pin.OUT)
L_EN = Pin(7, Pin.OUT)

BTN_ONOFF   = Pin(8,  Pin.IN, Pin.PULL_UP)
BTN_PLUS1   = Pin(9,  Pin.IN, Pin.PULL_UP)
BTN_MINUS1  = Pin(10, Pin.IN, Pin.PULL_UP)
BTN_PLUS10  = Pin(11, Pin.IN, Pin.PULL_UP)
BTN_MINUS10 = Pin(12, Pin.IN, Pin.PULL_UP)

# ---------------------------------------------------------------------------
# PARAMETRES - A CALIBRER avec ton module reel (voir etape de verification)
# ---------------------------------------------------------------------------
CENTER = 511
DEADBAND = 15

SPEED_FAST = 1023
SPEED_SLOW = 350

NETWORK_TIMEOUT_MS = 1500
DEBOUNCE_MS = 150
BACKEND_HOST = None  # ex: "192.168.1.10" ; laisser None pour desactiver la notif au backend
BACKEND_PORT = 23322

# Seuil de courant (en unite brute ADC 0-4095) au-dela duquel on considere
# que le verin est bloque/en butee. A CALIBRER: mesure la valeur ADC normale
# en fonctionnement (marche a vide) puis la valeur en blocage, mets le seuil
# entre les deux, plus proche de la valeur de blocage.
CURRENT_THRESHOLD = 2500          # PLACEHOLDER - a ajuster apres mesure reelle
CURRENT_TRIP_DURATION_MS = 150     # duree au-dessus du seuil avant de couper
                                     # (evite de couper sur un pic transitoire au demarrage)
CURRENT_SAMPLE_MS = 20

# ---------------------------------------------------------------------------
# ETAT GLOBAL
# ---------------------------------------------------------------------------
motor_enabled = False
automation_mode = False        # True quand l'autopilot / le pilot est actif
fault_latched = False          # True apres un declenchement surintensite,
                                 # necessite un cycle OFF/ON pour reinitialiser
current_over_since_ms = None

last_network_speed = 0
last_network_cmd_ms = 0
last_button_ms = 0
manual_override_active = False
current_direction = 0          # +1, -1 ou 0, pour savoir quel IS surveiller


def notify_backend_state(enabled):
    """Notifies the backend that the pilot automation is ON or OFF.
    The backend protocol expects lines like: ap.enabled=true\n
    """
    if BACKEND_HOST is None:
        return
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1)
        s.connect((BACKEND_HOST, BACKEND_PORT))
        payload = f"ap.enabled={'true' if enabled else 'false'}\n".encode()
        s.sendall(payload)
        s.close()
    except Exception:
        # Ne bloque pas le controleur si le backend n'est pas present.
        pass


def hard_stop():
    RPWM.duty(0)
    LPWM.duty(0)
    R_EN.value(0)
    L_EN.value(0)


def check_overcurrent():
    """
    Lit R_IS/L_IS selon la direction active et declenche un fault
    si le courant depasse le seuil pendant plus de CURRENT_TRIP_DURATION_MS.
    """
    global fault_latched, current_over_since_ms

    if current_direction == 0:
        current_over_since_ms = None
        return

    reading = adc_r_is.read() if current_direction > 0 else adc_l_is.read()

    now = time.ticks_ms()
    if reading >= CURRENT_THRESHOLD:
        if current_over_since_ms is None:
            current_over_since_ms = now
        elif time.ticks_diff(now, current_over_since_ms) >= CURRENT_TRIP_DURATION_MS:
            fault_latched = True
            hard_stop()
            print("FAULT: surintensite detectee, valeur ADC =", reading)
    else:
        current_over_since_ms = None


def set_motor(speed):
    global current_direction

    if fault_latched:
        hard_stop()
        current_direction = 0
        return

    if not motor_enabled and not manual_override_active:
        hard_stop()
        current_direction = 0
        return

    # Une commande distante / autopilot doit rester bloquee tant que le mode
    # automation n'est pas relance explicitement par ON.
    if not automation_mode and not manual_override_active:
        hard_stop()
        current_direction = 0
        return

    if speed > 0:
        current_direction = 1
        RPWM.duty(speed)
        LPWM.duty(0)
        R_EN.value(1)
        L_EN.value(1)
    elif speed < 0:
        current_direction = -1
        RPWM.duty(0)
        LPWM.duty(-speed)
        R_EN.value(1)
        L_EN.value(1)
    else:
        current_direction = 0
        hard_stop()


def set_motor_from_raw(raw_value):
    global last_network_speed, last_network_cmd_ms, manual_override_active

    if not automation_mode:
        last_network_speed = 0
        manual_override_active = False
        hard_stop()
        return

    centered = raw_value - CENTER
    if abs(centered) <= DEADBAND:
        speed = 0
    elif centered > 0:
        speed = centered - DEADBAND
    else:
        speed = centered + DEADBAND

    speed = max(-1023, min(1023, speed))
    last_network_speed = speed
    last_network_cmd_ms = time.ticks_ms()

    if not manual_override_active:
        set_motor(speed)


def check_network_watchdog():
    if manual_override_active:
        return
    if last_network_speed != 0:
        elapsed = time.ticks_diff(time.ticks_ms(), last_network_cmd_ms)
        if elapsed > NETWORK_TIMEOUT_MS:
            hard_stop()


def check_buttons():
    global motor_enabled, manual_override_active, last_button_ms, fault_latched, automation_mode

    now = time.ticks_ms()

    if BTN_ONOFF.value() == 0 and time.ticks_diff(now, last_button_ms) > DEBOUNCE_MS:
        automation_mode = not automation_mode
        motor_enabled = automation_mode
        manual_override_active = False
        last_network_speed = 0
        current_direction = 0
        last_button_ms = now
        if automation_mode:
            fault_latched = False   # reinitialise le fault a chaque activation
            print("Mode automation: ON")
            notify_backend_state(True)
        else:
            hard_stop()
            print("Mode automation: OFF")
            notify_backend_state(False)
        time.sleep_ms(DEBOUNCE_MS)

    # Si le mode automation est desactive, on autorise le mouvement manuel direct
    # depuis le bouton physique. Sinon, les boutons ne doivent pas contourner le pilot.
    if fault_latched:
        manual_override_active = False
        return

    if automation_mode and not motor_enabled:
        manual_override_active = False
        return

    if BTN_PLUS10.value() == 0:
        if not automation_mode:
            manual_override_active = True
            set_motor(SPEED_FAST)
        else:
            manual_override_active = False
            hard_stop()
    elif BTN_MINUS10.value() == 0:
        if not automation_mode:
            manual_override_active = True
            set_motor(-SPEED_FAST)
        else:
            manual_override_active = False
            hard_stop()
    elif BTN_PLUS1.value() == 0:
        if not automation_mode:
            manual_override_active = True
            set_motor(SPEED_SLOW)
        else:
            manual_override_active = False
            hard_stop()
    elif BTN_MINUS1.value() == 0:
        if not automation_mode:
            manual_override_active = True
            set_motor(-SPEED_SLOW)
        else:
            manual_override_active = False
            hard_stop()
    else:
        if manual_override_active:
            manual_override_active = False
            hard_stop()


# ---------------------------------------------------------------------------
# WIFI + SERVEUR
# ---------------------------------------------------------------------------
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)
print("Connecting to WiFi...")
count = 0
while not wlan.isconnected():
    print(f"Attempt {count}...")
    time.sleep(1)
    count += 1
    if count > 20:
        print("Failed!")
        break

hard_stop()

if wlan.isconnected():
    ip = wlan.ifconfig()[0]
    print(f"Connected! IP: {ip}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', 80))
    sock.listen(1)
    sock.settimeout(0.05)
    print("Server listening...")

    last_current_check_ms = time.ticks_ms()

    while True:
        check_buttons()
        check_network_watchdog()

        now = time.ticks_ms()
        if time.ticks_diff(now, last_current_check_ms) >= CURRENT_SAMPLE_MS:
            check_overcurrent()
            last_current_check_ms = now

        try:
            conn, addr = sock.accept()
        except OSError:
            continue

        try:
            request = conn.recv(1024).decode()
            response_body = "ESP32 Motor Server\n"
            raw_value = None

            if "/motor/" in request:
                try:
                    raw_value = int(request.split("/motor/")[1].split(" ")[0])
                    set_motor_from_raw(raw_value)
                    status = "FAULT" if fault_latched else ("ON" if motor_enabled else "OFF")
                    response_body = f"Motor raw: {raw_value} status: {status}\n"
                except Exception as e:
                    response_body = f"Error: {e}\n"

            response = f"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: {len(response_body)}\r\n\r\n{response_body}"
            print(addr, raw_value, "FAULT" if fault_latched else "")
            conn.sendall(response.encode())
        finally:
            conn.close()
else:
    print("Mode manuel seul (pas de WiFi)")
    last_current_check_ms = time.ticks_ms()
    while True:
        check_buttons()
        now = time.ticks_ms()
        if time.ticks_diff(now, last_current_check_ms) >= CURRENT_SAMPLE_MS:
            check_overcurrent()
            last_current_check_ms = now
        time.sleep_ms(10)
