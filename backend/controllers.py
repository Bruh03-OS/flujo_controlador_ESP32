"""
Definiciones de prototipos y controladores.

Cada controlador declara sus parámetros de forma que el frontend construya el
formulario dinámicamente. Todos los parámetros se inicializan en 0 en la UI.

Controladores del proyecto: PID, Lógica Difusa, LQR y Modos Deslizantes (SMC).
"""

# --- Prototipos (planta a controlar) ---------------------------------------
# Cada prototipo define un modelo de planta de 2do orden aproximado:
#   K   -> ganancia
#   wn  -> frecuencia natural
#   z   -> amortiguamiento
# El "balancín" (Ball and Beam) es poco amortiguado; el "carrito" (seguidor de
# línea) es más amortiguado. Estos valores solo los usa el SIMULADOR; con el
# ESP32 real la planta es física y estos números se ignoran.
PROTOTIPOS = {
    "balancin": {
        "id": "balancin",
        "nombre": "Balancín (Ball and Beam)",
        "descripcion": "Viga basculante con esfera. Sistema poco amortiguado, propenso a oscilar.",
        "unidad": "grados",
        "planta": {"K": 1.0, "wn": 2.2, "z": 0.18},
    },
    "carrito": {
        "id": "carrito",
        "nombre": "Carrito (seguidor de línea)",
        "descripcion": "Posición respecto al centro de la línea. Respuesta más amortiguada.",
        "unidad": "cm",
        "planta": {"K": 1.0, "wn": 1.6, "z": 0.55},
    },
}

# --- Controladores ----------------------------------------------------------
# Cada 'param' tiene:
#   key      -> identificador interno
#   label    -> etiqueta visible
#   unidad   -> texto de unidad (puede ir vacío)
#   min/max  -> límites de validación (opcionales)
CONTROLADORES = {
    "pid": {
        "id": "pid",
        "nombre": "PID",
        "descripcion": "Proporcional-Integral-Derivativo clásico (estrategia de referencia).",
        "params": [
            {"key": "setpoint", "label": "Setpoint", "unidad": "ref", "min": -100, "max": 100},
            {"key": "kp", "label": "Kp", "unidad": "", "min": 0, "max": 100},
            {"key": "ki", "label": "Ki", "unidad": "", "min": 0, "max": 100},
            {"key": "kd", "label": "Kd", "unidad": "", "min": 0, "max": 100},
        ],
    },
    "difusa": {
        "id": "difusa",
        "nombre": "Lógica Difusa",
        "descripcion": "Controlador difuso PD con ganancias de escalado.",
        "params": [
            {"key": "setpoint", "label": "Setpoint", "unidad": "ref", "min": -100, "max": 100},
            {"key": "ke", "label": "Ke (error)", "unidad": "", "min": 0, "max": 100},
            {"key": "kde", "label": "Kde (d error)", "unidad": "", "min": 0, "max": 100},
            {"key": "ku", "label": "Ku (salida)", "unidad": "", "min": 0, "max": 100},
        ],
    },
    "lqr": {
        "id": "lqr",
        "nombre": "LQR",
        "descripcion": "Regulador lineal cuadrático: realimentación de estado óptima con pesos Q y R.",
        "params": [
            {"key": "setpoint", "label": "Setpoint", "unidad": "ref", "min": -100, "max": 100},
            {"key": "q_pos", "label": "Q posición", "unidad": "", "min": 0, "max": 1000},
            {"key": "q_vel", "label": "Q velocidad", "unidad": "", "min": 0, "max": 1000},
            {"key": "r", "label": "R (esfuerzo)", "unidad": "", "min": 0.001, "max": 1000},
        ],
    },
    "smc": {
        "id": "smc",
        "nombre": "Modos Deslizantes (SMC)",
        "descripcion": "Control por modos deslizantes: superficie s = λ·e + ė con capa límite.",
        "params": [
            {"key": "setpoint", "label": "Setpoint", "unidad": "ref", "min": -100, "max": 100},
            {"key": "lambda", "label": "λ (pendiente)", "unidad": "", "min": 0, "max": 50},
            {"key": "eta", "label": "η (conmutación)", "unidad": "", "min": 0, "max": 200},
            {"key": "phi", "label": "φ (capa límite)", "unidad": "", "min": 0.001, "max": 100},
        ],
    },
}


def meta():
    """Estructura que consume el frontend para armar Selección y formularios."""
    return {
        "prototipos": [
            {k: v[k] for k in ("id", "nombre", "descripcion", "unidad")}
            for v in PROTOTIPOS.values()
        ],
        "controladores": [
            {"id": c["id"], "nombre": c["nombre"], "descripcion": c["descripcion"], "params": c["params"]}
            for c in CONTROLADORES.values()
        ],
    }
