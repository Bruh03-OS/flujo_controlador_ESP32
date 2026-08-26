"""
Leyes de control usadas por el simulador.

Cada controlador es una clase con .step(y, dt) -> u  (señal de control),
donde y es la variable controlada medida. Con el ESP32 real, estas leyes
viven en el firmware; aquí replican el comportamiento para poder probar todo
sin hardware.
"""


class PID:
    def __init__(self, setpoint, kp, ki, kd):
        self.sp = setpoint
        self.kp, self.ki, self.kd = kp, ki, kd
        self._int = 0.0
        self._prev_e = None

    def step(self, y, dt):
        e = self.sp - y
        self._int += e * dt
        de = 0.0 if self._prev_e is None else (e - self._prev_e) / dt
        self._prev_e = e
        u = self.kp * e + self.ki * self._int + self.kd * de
        return u


class OnOff:
    """Control lógico on-off con histéresis."""
    def __init__(self, setpoint, histeresis, u_alta, u_baja):
        self.sp = setpoint
        self.h = histeresis
        self.u_alta, self.u_baja = u_alta, u_baja
        self._u = u_baja

    def step(self, y, dt):
        e = self.sp - y
        if e > self.h:
            self._u = self.u_alta
        elif e < -self.h:
            self._u = self.u_baja
        # dentro de la banda: mantiene la última salida
        return self._u


class Fuzzy:
    """
    Controlador difuso PD tipo Mamdani simplificado.
    Fuzzifica error (e) y derivada (de) en {Negativo, Cero, Positivo} con
    ganancias de escalado ke/kde, aplica tabla de 3x3 reglas y defuzzifica.
    """
    RULES = [  # [de idx][e idx] -> salida normalizada [-1..1]
        [-1.0, -1.0, 0.0],
        [-1.0,  0.0, 1.0],
        [ 0.0,  1.0, 1.0],
    ]

    def __init__(self, setpoint, ke, kde, ku):
        self.sp = setpoint
        self.ke, self.kde, self.ku = ke, kde, ku
        self._prev_e = None

    @staticmethod
    def _memberships(x):
        # x normalizado (~[-1,1]) -> grados de N, Z, P
        x = max(-1.0, min(1.0, x))
        neg = max(0.0, -x)
        pos = max(0.0, x)
        zero = max(0.0, 1.0 - abs(x))
        s = neg + zero + pos
        if s == 0:
            return [0.0, 1.0, 0.0]
        return [neg / s, zero / s, pos / s]

    def step(self, y, dt):
        e = self.sp - y
        de = 0.0 if self._prev_e is None else (e - self._prev_e) / dt
        self._prev_e = e
        me = self._memberships(e * self.ke * 0.01)
        mde = self._memberships(de * self.kde * 0.01)
        num = den = 0.0
        for i in range(3):
            for j in range(3):
                w = mde[i] * me[j]
                num += w * self.RULES[i][j]
                den += w
        out = 0.0 if den == 0 else num / den
        return out * self.ku


class MPC:
    """
    Control predictivo por modelo (simplificado, SISO).
    Usa el modelo interno de la planta para predecir Np pasos y elige la señal
    de control constante que minimiza el error futuro con penalización lambda
    al esfuerzo. Optimización escalar por barrido.
    """
    def __init__(self, setpoint, np_, nc, lam, model):
        self.sp = setpoint
        self.np = int(max(1, np_))
        self.nc = int(max(1, nc))
        self.lam = lam
        self.model = model  # dict con K, wn, z
        self._x = [0.0, 0.0]  # estado interno estimado [y, y']

    def _predict(self, x, u, dt):
        K, wn, z = self.model["K"], self.model["wn"], self.model["z"]
        y, yd = x
        ydd = K * wn * wn * u - 2 * z * wn * yd - wn * wn * y
        yd2 = yd + ydd * dt
        y2 = y + yd * dt
        return [y2, yd2]

    def step(self, y, dt):
        # sincroniza estimación de posición con la medición
        self._x[0] = y
        best_u, best_J = 0.0, float("inf")
        for k in range(-40, 41):
            u = k * 2.5
            x = list(self._x)
            J = 0.0
            for _ in range(self.np):
                x = self._predict(x, u, dt)
                J += (self.sp - x[0]) ** 2
            J += self.lam * (u ** 2) * 0.001
            if J < best_J:
                best_J, best_u = J, u
        # avanza el estado interno un paso con la acción elegida
        self._x = self._predict(self._x, best_u, dt)
        return best_u


def build_controller(controlador_id, params, planta):
    p = params
    if controlador_id == "pid":
        return PID(p["setpoint"], p["kp"], p["ki"], p["kd"])
    if controlador_id == "logica":
        return OnOff(p["setpoint"], p["histeresis"], p["u_alta"], p["u_baja"])
    if controlador_id == "difusa":
        return Fuzzy(p["setpoint"], p["ke"], p["kde"], p["ku"])
    if controlador_id == "predictiva":
        return MPC(p["setpoint"], p["np"], p["nc"], p["lambda"], planta)
    raise ValueError(f"Controlador desconocido: {controlador_id}")
