# Firmware — Carrito seguidor de línea (ESP32-S3)

Firmware del prototipo **carrito** para la plataforma modular de control.
La variable controlada es la **posición de la línea** (offset lateral) medida con
el arreglo IR; el control corrige el giro de los dos motores para llevar esa
posición al setpoint (0 = línea centrada).

## Hardware

| Componente | Detalle |
|---|---|
| Placa | ESP32-S3-N16R8 |
| Driver de motores | TB6612FNG (dual) |
| Motores | 2x DC 6V 100RPM con caja reductora (diferencial izq/der) |
| Sensor | TCRT-5000, arreglo IR de 5 canales (OUT1..OUT5) |
| Alimentación | Batería Li-Ion 2S 7.4V → buck XL4015 |

## Pines (VERIFICAR contra tu diagrama)

Estos son los pines asignados en `carrito.ino`. **Confírmalos contra el
cableado real en Cirkit** y ajusta el bloque `// PINES` si algo no coincide:

```
Sensor IR TCRT-5000 (OUT1..OUT5):  GPIO 4, 5, 6, 7, 15
TB6612FNG  Motor A (izq): AIN1=9,  AIN2=46, PWMA=3
TB6612FNG  Motor B (der): BIN1=11, BIN2=12, PWMB=13
TB6612FNG  STBY: 10
```

Dos cosas a revisar en tu módulo TCRT-5000:
- **Digital vs analógico:** por defecto el firmware lee las salidas como
  digitales. Si tu módulo entrega salida analógica, pon `#define USE_ANALOG 1`.
- **Polaridad:** `LINE_DETECTED_LEVEL` indica el nivel del pin cuando el sensor
  ve la línea. Si el carrito "se va al revés", cambia `HIGH` por `LOW`.

## Protocolo serial (115200 baud)

Idéntico al backend, para que la app web lo maneje sin cambios:

```
PC -> ESP32 : START,<controlador>,<p1>,<p2>,<p3>,<p4>\n
PC -> ESP32 : STOP\n
ESP32 -> PC : DATA,<t>,<ref>,<vc>,<error>,<u>\n
ESP32 -> PC : DONE\n
```

Parámetros por controlador:

| Controlador | p1 | p2 | p3 | p4 |
|---|---|---|---|---|
| `pid` | setpoint | kp | ki | kd |
| `difusa` | setpoint | ke | kde | ku |
| `lqr` | setpoint | q_pos | q_vel | r |
| `smc` | setpoint | lambda (λ) | eta (η) | phi (φ) |

## Cargar al ESP32

1. Arduino IDE con el core **esp32 by Espressif (3.x)**.
2. Placa: *ESP32S3 Dev Module*.
3. Abrir `carrito.ino` (deja `control_laws.h` en la misma carpeta).
4. Seleccionar el puerto y cargar.

Para usarlo con la app web: en `backend/main.py` pon `USE_SIMULATOR = False` y
`SERIAL_PORT` al puerto del ESP32 (mac/Linux `/dev/tty...`, Windows `COM...`).

## Probar la lógica sin hardware (software-in-the-loop)

`control_laws.h` es C++ puro, así que las mismas leyes que corren en el ESP32 se
pueden compilar y validar en la PC contra el modelo de planta:

```bash
g++ -O2 -std=c++17 -o test_host test_host.cpp
./test_host
```

Debe mostrar que PID, LQR y SMC alcanzan el setpoint (LQR y SMC con error
estacionario muy bajo) y que el difuso PD deja un pequeño offset — el mismo
comportamiento que el simulador del backend.

## Archivos

```
carrito.ino      firmware principal (sensor, motores, protocolo, lazo de control)
control_laws.h   leyes de control PID / difusa / LQR / SMC (compartidas)
test_host.cpp    prueba de escritorio en lazo cerrado
```
