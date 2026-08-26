"""
Fuente de telemetría.

Dos implementaciones intercambiables:
  - SimulatorSource: simula la planta + el controlador (sin hardware).
  - Esp32SerialSource: lee líneas por USB del ESP32 (protocolo serial abajo).

Ambas producen dicts de telemetría:  {t, ref, vc, error, u}
y detectan cuándo se alcanzó el setpoint (done).

Protocolo serial ESP32 (para cuando conectes el hardware):
  PC  -> ESP32 :  START,<controlador>,<p1>,<p2>,...\n
  PC  -> ESP32 :  STOP\n
  ESP32 -> PC  :  DATA,<t>,<ref>,<vc>,<error>,<u>\n
  ESP32 -> PC  :  DONE\n
"""
import math
import random

DT = 0.03            # paso de integración / muestreo (s)  ~33 Hz
MAX_T = 25.0         # tope de seguridad de duración (s)
U_MIN, U_MAX = -100.0, 100.0
TOL_STEADY_S = 1.0   # tiempo sostenido dentro de tolerancia para dar "done"


class SimulatorSource:
    def __init__(self, prototipo, controlador_id, params, controller):
        self.planta = prototipo["planta"]
        self.controller = controller
        self.setpoint = float(params.get("setpoint", 0.0))
        self.dt = DT
        # estado de la planta 2do orden: [y, y']
        self.y = 0.0
        self.yd = 0.0
        self.t = 0.0
        self._steady_acc = 0.0
        self.T, self.REF, self.VC, self.ERR, self.U = [], [], [], [], []

    def _tolerance(self):
        return max(0.02 * abs(self.setpoint), 0.3)

    def step(self):
        K = self.planta["K"]; wn = self.planta["wn"]; z = self.planta["z"]
        # medición con ruido leve
        y_meas = self.y + random.gauss(0, 0.02)
        u = self.controller.step(y_meas, self.dt)
        u = max(U_MIN, min(U_MAX, u))
        # dinámica de la planta (2do orden, Euler)
        ydd = K * wn * wn * u - 2 * z * wn * self.yd - wn * wn * self.y
        self.yd += ydd * self.dt
        self.y += self.yd * self.dt
        self.t += self.dt

        ref = self.setpoint
        vc = self.y
        err = ref - vc

        self.T.append(round(self.t, 3)); self.REF.append(round(ref, 4))
        self.VC.append(round(vc, 4)); self.ERR.append(round(err, 4))
        self.U.append(round(u, 4))

        # ¿setpoint alcanzado de forma sostenida?
        if abs(err) <= self._tolerance():
            self._steady_acc += self.dt
        else:
            self._steady_acc = 0.0
        done = self._steady_acc >= TOL_STEADY_S or self.t >= MAX_T
        reason = None
        if done:
            reason = "setpoint" if self._steady_acc >= TOL_STEADY_S else "timeout"

        return {
            "t": round(self.t, 3),
            "ref": round(ref, 4),
            "vc": round(vc, 4),
            "error": round(err, 4),
            "u": round(u, 4),
            "done": done,
            "reason": reason,
        }

    def arrays(self):
        return self.T, self.REF, self.VC, self.ERR


class Esp32SerialSource:
    """
    Lectura real del ESP32 por puerto serial. Se activa poniendo
    USE_SIMULATOR=False en main.py y ajustando el puerto.
    Requiere: pip install pyserial
    """
    def __init__(self, prototipo, controlador_id, params, port, baud=115200):
        import serial  # import diferido para no exigir pyserial en modo simulador
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.T, self.REF, self.VC, self.ERR = [], [], [], []
        # ordena params según su definición y envía START
        vals = ",".join(str(v) for v in params.values())
        self.ser.write(f"START,{controlador_id},{vals}\n".encode())

    def step(self):
        line = self.ser.readline().decode(errors="ignore").strip()
        if not line:
            return None
        if line == "DONE":
            return {"t": self.T[-1] if self.T else 0, "done": True, "reason": "setpoint"}
        if line.startswith("DATA,"):
            _, t, ref, vc, err, u = line.split(",")
            t, ref, vc, err, u = map(float, (t, ref, vc, err, u))
            self.T.append(t); self.REF.append(ref); self.VC.append(vc); self.ERR.append(err)
            return {"t": t, "ref": ref, "vc": vc, "error": err, "u": u,
                    "done": False, "reason": None}
        return None

    def stop(self):
        try:
            self.ser.write(b"STOP\n")
            self.ser.close()
        except Exception:
            pass

    def arrays(self):
        return self.T, self.REF, self.VC, self.ERR
