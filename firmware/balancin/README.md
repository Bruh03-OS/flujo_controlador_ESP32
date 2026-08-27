# Firmware — Balancín / Ball and Beam (ESP32-S3)

Firmware del prototipo **balancín** para la plataforma modular de control.
La variable controlada es la **posición de la bola** sobre la viga (medida por
el sensor Sharp); el control ajusta el **ángulo de la viga** mediante el servo
para llevar esa posición al setpoint.

## Hardware

| Componente | Detalle |
|---|---|
| Placa | ESP32-S3-N16R8 |
| Actuador | Servomotor MG996R (inclina la viga) |
| Sensor | Sharp IR analógico (GP2Y0A21 aprox.) — posición de la bola |
| Alimentación | Batería Li-Ion 2S 7.4V → buck |

## Pines (VERIFICAR contra tu diagrama)

```
Sensor Sharp (salida analógica):  GPIO 4   (debe ser un pin con ADC)
Servo MG996R (señal PWM):         GPIO 18
```

## Calibración importante

- **Sensor Sharp:** es no lineal. El firmware usa la curva aproximada del
  GP2Y0A21 (`dist ≈ 27.86·V^-1.15`); **recalíbrala con tu sensor** midiendo la
  bola a distancias conocidas. Ajusta también `BEAM_CENTER_CM` (distancia al
  centro de la viga, para que setpoint 0 = centro).
- **Alimentación del Sharp:** si lo alimentas a 5V su salida puede superar los
  3.3V del ADC del ESP32; usa un divisor de tensión o aliméntalo a 3.3V.
- **Servo:** `SERVO_CENTER` es el ángulo con la viga nivelada y `SERVO_SPAN` la
  inclinación máxima a cada lado. Si la bola "se escapa" en vez de centrarse,
  invierte `CONTROL_SIGN` (1 → -1). El MG996R conviene alimentarlo con fuente
  aparte (no desde el 3.3V del ESP32); comparte GND.

## Protocolo serial (115200 baud)

Idéntico al backend y al carrito:

```
PC -> ESP32 : START,<controlador>,<p1>,<p2>,<p3>,<p4>\n
PC -> ESP32 : STOP\n
ESP32 -> PC : DATA,<t>,<ref>,<vc>,<error>,<u>\n
ESP32 -> PC : DONE\n
```

| Controlador | p1 | p2 | p3 | p4 |
|---|---|---|---|---|
| `pid` | setpoint | kp | ki | kd |
| `difusa` | setpoint | ke | kde | ku |
| `lqr` | setpoint | q_pos | q_vel | r |
| `smc` | setpoint | lambda (λ) | eta (η) | phi (φ) |

`u` es la señal de control que el firmware convierte en ángulo de la viga.
LQR y SMC usan el modelo de la planta del balancín (`{K:1.0, wn:2.2, z:0.18}`),
definido en `balancin.ino` como `BALANCIN`.

## Cargar al ESP32

1. Arduino IDE con el core **esp32 by Espressif (3.x)**.
2. Placa: *ESP32S3 Dev Module*.
3. Abrir `balancin.ino` (deja `control_laws.h` en la misma carpeta).
4. Seleccionar el puerto y cargar.

El servo se maneja con LEDC, así que **no requiere ninguna librería externa**.

Para usarlo con la app web: en `backend/main.py` pon `USE_SIMULATOR = False` y
`SERIAL_PORT` al puerto del ESP32.

## Probar la lógica sin hardware (software-in-the-loop)

```bash
g++ -O2 -std=c++17 -o test_host test_host.cpp
./test_host
```

Debe mostrar que PID, LQR y SMC alcanzan el setpoint (LQR y SMC con error
estacionario muy bajo) y que el difuso PD deja un pequeño offset — el mismo
comportamiento que el simulador del backend con el modelo del balancín.

## Archivos

```
balancin.ino     firmware principal (servo, sensor Sharp, protocolo, lazo)
control_laws.h   leyes de control PID / difusa / LQR / SMC (compartidas)
test_host.cpp    prueba de escritorio en lazo cerrado
```
