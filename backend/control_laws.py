"""
Leyes de control usadas por el simulador.

Cada controlador es una clase con .step(y, dt) -> u  (señal de control),
donde y es la variable controlada medida. Con el ESP32 real, estas leyes
viven en el firmware; aquí replican el comportamiento para poder probar todo
sin hardware.

Controladores: PID, Fuzzy (difusa), LQR y SMC (modos deslizantes).
LQR y SMC son basados en modelo: usan el modelo de 2do orden de la planta.
"""

DT = 0.03  # periodo de muestreo (s), coherente con source.py


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
        return self.kp * e + self.ki * self._int + self.kd * de


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


class LQR:
    """
    Regulador lineal cuadrático (realimentación de estado óptima).

    Estado x = [posición, velocidad]. La ganancia K se obtiene resolviendo la
    ecuación de Riccati discreta (DARE) a partir del modelo de la planta y de
    los pesos Q = diag(q_pos, q_vel) y R. Se agrega un término de
    prealimentación para llevar el estado estacionario al setpoint.

        u = ff*sp + k1*(sp - y) - k2*vel
    """
    def __init__(self, setpoint, q_pos, q_vel, r, planta):
        self.sp = setpoint
        K, wn, z = planta["K"], planta["wn"], planta["z"]
        a0 = wn * wn          # rigidez
        a1 = 2 * z * wn       # amortiguamiento
        b = K * wn * wn       # ganancia de entrada
        dt = DT
        # discretización Euler del modelo continuo
        Ad = [[1.0, dt], [-a0 * dt, 1.0 - a1 * dt]]
        Bd = [0.0, b * dt]
        Q = [[max(q_pos, 0.0), 0.0], [0.0, max(q_vel, 0.0)]]
        R = max(r, 1e-3)
        self.k1, self.k2 = self._dlqr(Ad, Bd, Q, R)
        self.ff = a0 / b if b != 0 else 0.0     # prealimentación (u para sostener sp)
        self._prev_y = None

    @staticmethod
    def _matT(A):
        return [[A[0][0], A[1][0]], [A[0][1], A[1][1]]]

    @staticmethod
    def _mul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(2)) for j in range(2)] for i in range(2)]

    def _dlqr(self, Ad, Bd, Q, R):
        # Itera la DARE hasta converger (2x2, una sola entrada).
        P = [[0.0, 0.0], [0.0, 0.0]]
        AdT = self._matT(Ad)
        for _ in range(1000):
            AdTP = self._mul(AdT, P)
            AdTPA = self._mul(AdTP, Ad)
            # Ad^T P Bd  (2x1)
            AtPB = [AdTP[0][0] * Bd[0] + AdTP[0][1] * Bd[1],
                    AdTP[1][0] * Bd[0] + AdTP[1][1] * Bd[1]]
            # Bd^T P Bd  (escalar)
            BtPB = Bd[0] * (P[0][0] * Bd[0] + P[0][1] * Bd[1]) + \
                   Bd[1] * (P[1][0] * Bd[0] + P[1][1] * Bd[1])
            S = R + BtPB
            Pn = [[Q[i][j] + AdTPA[i][j] - AtPB[i] * AtPB[j] / S for j in range(2)] for i in range(2)]
            diff = max(abs(Pn[i][j] - P[i][j]) for i in range(2) for j in range(2))
            P = Pn
            if diff < 1e-9:
                break
        # K = (1/S) Bd^T P Ad
        BtP = [P[0][0] * Bd[0] + P[1][0] * Bd[1], P[0][1] * Bd[0] + P[1][1] * Bd[1]]
        BtPB = Bd[0] * (P[0][0] * Bd[0] + P[0][1] * Bd[1]) + \
               Bd[1] * (P[1][0] * Bd[0] + P[1][1] * Bd[1])
        S = R + BtPB
        k1 = (BtP[0] * Ad[0][0] + BtP[1] * Ad[1][0]) / S
        k2 = (BtP[0] * Ad[0][1] + BtP[1] * Ad[1][1]) / S
        return k1, k2

    def step(self, y, dt):
        vel = 0.0 if self._prev_y is None else (y - self._prev_y) / dt
        self._prev_y = y
        return self.ff * self.sp + self.k1 * (self.sp - y) - self.k2 * vel


class SMC:
    """
    Control por modos deslizantes (Sliding Mode Control).

    Superficie deslizante s = λ·e + ė, con e = sp - y.
    Ley de control: u = u_eq + η·sat(s/φ), donde u_eq es la prealimentación
    (control equivalente del modelo) y la saturación en la capa límite φ
    reduce el castañeo (chattering) frente a la función signo pura.
    """
    def __init__(self, setpoint, lam, eta, phi, planta):
        self.sp = setpoint
        self.lam = lam
        self.eta = eta
        self.phi = max(phi, 1e-3)
        K, wn = planta["K"], planta["wn"]
        b = K * wn * wn
        a0 = wn * wn
        self.ueq = a0 / b if b != 0 else 0.0    # control equivalente por unidad de sp
        self._prev_e = None

    def step(self, y, dt):
        e = self.sp - y
        de = 0.0 if self._prev_e is None else (e - self._prev_e) / dt
        self._prev_e = e
        s = self.lam * e + de
        sat = max(-1.0, min(1.0, s / self.phi))
        return self.ueq * self.sp + self.eta * sat


def build_controller(controlador_id, params, planta):
    p = params
    if controlador_id == "pid":
        return PID(p["setpoint"], p["kp"], p["ki"], p["kd"])
    if controlador_id == "difusa":
        return Fuzzy(p["setpoint"], p["ke"], p["kde"], p["ku"])
    if controlador_id == "lqr":
        return LQR(p["setpoint"], p["q_pos"], p["q_vel"], p["r"], planta)
    if controlador_id == "smc":
        return SMC(p["setpoint"], p["lambda"], p["eta"], p["phi"], planta)
    raise ValueError(f"Controlador desconocido: {controlador_id}")
