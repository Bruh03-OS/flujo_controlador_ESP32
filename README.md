# Flujo Controlador ESP32

Aplicación web para configurar, ejecutar y analizar controladores sobre un prototipo
físico (Balancín o Carrito) manejado por un ESP32. Consta de tres páginas:

1. **Selección** — eliges prototipo (Balancín / Carrito) y controlador (PID, Lógica On-Off, Difusa, Predictiva).
2. **Control** — formulario dinámico de parámetros (inician en 0); al llenarse aparece **Iniciar**. Durante la corrida ves telemetría en vivo (Referencia, Variable Controlada, Error, Señal de Control, Tiempo) en un osciloscopio. Termina al alcanzar el setpoint o al pulsar **Finalizar**; muestra un aviso y ofrece pasar a Datos.
3. **Datos** — tabla de las últimas 50 corridas con Tiempo de ejecución, Error RMS, Sobreimpulso y Error estacionario, más gráficas de respuesta temporal y comparación entre corridas.

## Arquitectura

```
Navegador (React)  ⇄  WebSocket / HTTP  ⇄  Backend (FastAPI)  ⇄  Serial USB  ⇄  ESP32
                                              │
                                              ├─ simulador de planta (modo sin hardware)
                                              ├─ cálculo de métricas
                                              └─ SQLite (runs.db)
```

El backend incluye un **simulador** de planta + controlador para que todo funcione
sin hardware. Para usar el ESP32 real solo se cambia `USE_SIMULATOR = False` en
`backend/main.py` y se ajusta `SERIAL_PORT`.

## Requisitos

- Python 3.10+
- Node.js 18+

## Cómo correrlo

### Backend (puerto 8000)

```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --reload --port 8000
```

### Frontend

**Opción A — desarrollo (recarga en caliente, puerto 5173):**

```bash
cd frontend
npm install
npm run dev
```

Abre http://localhost:5173  (Vite reenvía `/api` y `/ws` al backend en :8000).

**Opción B — producción (un solo servidor):**

```bash
cd frontend
npm install
npm run build      # genera frontend/dist
```

Con el backend corriendo, abre http://localhost:8000 — FastAPI sirve el build.

## Conectar el ESP32 real

En `backend/main.py`:

```python
USE_SIMULATOR = False
SERIAL_PORT = "COM3"        # Windows;  Linux/Mac: "/dev/ttyUSB0"
```

### Protocolo serial

```
PC   -> ESP32 :  START,<controlador>,<p1>,<p2>,...\n
PC   -> ESP32 :  STOP\n
ESP32 -> PC   :  DATA,<t>,<ref>,<vc>,<error>,<u>\n
ESP32 -> PC   :  DONE\n
```

El firmware del ESP32 mantiene un único código cargado; los parámetros llegan por
`START` y se aplican en caliente. No se re-flashea al cambiar parámetros.

## Estructura

```
backend/
  main.py          API FastAPI + WebSocket + servido del build
  controllers.py   definiciones de prototipos y controladores (parámetros)
  control_laws.py  leyes de control del simulador (PID, on-off, difusa, MPC)
  source.py        simulador de planta y lectura serial del ESP32
  metrics.py       Error RMS, Sobreimpulso, Error estacionario, Tiempo
  database.py      persistencia SQLite (últimos 50 registros)
frontend/
  src/pages/       Seleccion, Control, Datos
  src/components/  StepBar, Scope (osciloscopio en canvas)
  src/store.jsx    estado global del experimento
```
