"""
Definiciones de prototipos y controladores.

Cada controlador declara sus parámetros de forma que el frontend construya el
formulario dinámicamente. Todos los parámetros se inicializan en 0 en la UI.
"""

# --- Prototipos (planta a controlar) ---------------------------------------
# Cada prototipo define un modelo de planta de 2do orden aproximado:
#   K   -> ganancia
#   wn  -> frecuencia natural
#   z   -> amortiguamiento
# El "balancín" es poco amortiguado (oscila / sobreimpulso); el "carrito" es
# más amortiguado. Estos valores solo los usa el SIMULADOR; con el ESP32 real
# la planta es física y estos números se ignoran.
PROTOTIPOS = {
    "balancin": {
        "id": "balancin",
        "nombre": "Balancín",
        "descripcion": "Viga basculante. Sistema poco amortiguado, propenso a oscilar.",
        "unidad": "grados",
        "planta": {"K": 1.0, "wn": 2.2, "z": 0.18},
    },
    "carrito": {
        "id": "carrito",
        "nombre": "Carrito",
        "descripcion": "Posición de carro sobre riel. Respuesta más amortiguada.",
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
        "descripcion": "Proporcional-Integral-Derivativo clásico.",
        "params": [
            {"key": "setpoint", "label": "Setpoint", "unidad": "ref", "min": -100, "max": 100},
            {"key": "kp", "label": "Kp", "unidad": "", "min": 0, "max": 100},
            {"key": "ki", "label": "Ki", "unidad": "", "min": 0, "max": 100},
            {"key": "kd", "label": "Kd", "unidad": "", "min": 0, "max": 100},
        ],
    },
    "logica": {
        "id": "logica",
        "nombre": "Lógica (On-Off)",
        "descripcion": "Control on-off con banda de histéresis.",
        "params": [
            {"key": "setpoint", "label": "Setpoint", "unidad": "ref", "min": -100, "max": 100},
            {"key": "histeresis", "label": "Histéresis", "unidad": "ref", "min": 0, "max": 50},
            {"key": "u_alta", "label": "Salida alta", "unidad": "u", "min": 0, "max": 100},
            {"key": "u_baja", "label": "Salida baja", "unidad": "u", "min": -100, "max": 100},
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
    "predictiva": {
        "id": "predictiva",
        "nombre": "Predictiva (MPC)",
        "descripcion": "Control predictivo por modelo con horizonte y peso de esfuerzo.",
        "params": [
            {"key": "setpoint", "label": "Setpoint", "unidad": "ref", "min": -100, "max": 100},
            {"key": "np", "label": "Horizonte pred. (Np)", "unidad": "pasos", "min": 1, "max": 40},
            {"key": "nc", "label": "Horizonte control (Nc)", "unidad": "pasos", "min": 1, "max": 20},
            {"key": "lambda", "label": "Peso esfuerzo (λ)", "unidad": "", "min": 0, "max": 100},
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
