/*
 * balancin.ino — Firmware del balancín (Ball and Beam)
 * Plataforma experimental modular de control · ESP32-S3-N16R8
 *
 * Prototipo        : balancin (Ball and Beam)
 * Variable control : posición de la bola sobre la viga (cm, respecto al centro)
 * Sensor           : Sharp IR analógico (GP2Y0A21 aprox.) -> posición de la bola
 * Actuador         : servomotor MG996R -> inclina la viga
 *
 * Protocolo serial (115200 baud) — idéntico al backend:
 *   PC -> ESP32 : START,<controlador>,<p1>,<p2>,<p3>,<p4>\n
 *   PC -> ESP32 : STOP\n
 *   ESP32 -> PC : DATA,<t>,<ref>,<vc>,<error>,<u>\n
 *   ESP32 -> PC : DONE\n
 *
 * Controladores : pid | difusa | lqr | smc  (ver control_laws.h)
 * Requiere Arduino-ESP32 core 3.x. El servo se maneja con LEDC (sin librerías).
 */
#include "control_laws.h"

// ===========================================================================
//  PINES  —  VERIFICAR CONTRA EL DIAGRAMA ANTES DE CARGAR
// ===========================================================================
const int SHARP_PIN = 4;    // salida analógica del Sharp (GPIO con ADC)
const int SERVO_PIN = 18;   // señal PWM del servo MG996R

// ===========================================================================
//  MODELO DE LA PLANTA (balancín) — usado por LQR y SMC
//  Coincide con PROTOTIPOS['balancin'] del backend.
// ===========================================================================
const PlantModel BALANCIN = {1.0f, 2.2f, 0.18f};

// ===========================================================================
//  CONFIGURACIÓN
// ===========================================================================
// --- Servo ---
const int   SERVO_FREQ = 50;        // Hz (estándar para servos)
const int   SERVO_RES  = 16;        // bits de resolución LEDC
const float SERVO_CENTER = 90.0f;   // ángulo con la viga nivelada (grados)
const float SERVO_SPAN   = 35.0f;   // inclinación máxima a cada lado (grados)
const float SERVO_MIN_DEG = 45.0f, SERVO_MAX_DEG = 135.0f;  // límites de seguridad
const int   CONTROL_SIGN = 1;       // si la bola "se escapa", cambia a -1

// --- Sensor Sharp ---
const float BEAM_CENTER_CM = 15.0f; // distancia al centro de la viga (recalibrar)
const int   SHARP_SAMPLES  = 8;     // promedio para reducir ruido

// --- Lazo de control ---
const float DT         = 0.03f;     // periodo de muestreo (s) ~33 Hz
const float MAX_T      = 25.0f;     // tope de seguridad (s)
const float TOL_STEADY = 1.0f;      // s dentro de tolerancia para DONE

// ===========================================================================
//  ESTADO
// ===========================================================================
Controller ctrl;
bool  running = false;
float t = 0, steadyAcc = 0, lastPos = 0;
unsigned long tPrev = 0;

// ---------------------------------------------------------------------------
//  Servo (LEDC, sin librería externa)
// ---------------------------------------------------------------------------
void servoWriteDeg(float deg) {
  if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
  if (deg > SERVO_MAX_DEG) deg = SERVO_MAX_DEG;
  float pulse_us = 500.0f + (deg / 180.0f) * 2000.0f;   // 500..2500 us
  uint32_t maxDuty = (1UL << SERVO_RES) - 1;
  uint32_t duty = (uint32_t)(pulse_us / 20000.0f * maxDuty);  // periodo 20 ms
  ledcWrite(SERVO_PIN, duty);
}

// u (señal de control, ±100) -> ángulo de la viga
void applyControl(float u) {
  float deg = SERVO_CENTER + CONTROL_SIGN * (u / 100.0f) * SERVO_SPAN;
  servoWriteDeg(deg);
}

void beamLevel() { servoWriteDeg(SERVO_CENTER); }   // viga nivelada (reposo)

// ---------------------------------------------------------------------------
//  Sensor Sharp: posición de la bola (cm respecto al centro de la viga)
// ---------------------------------------------------------------------------
float readPosition() {
  long acc = 0;
  for (int i = 0; i < SHARP_SAMPLES; i++) acc += analogRead(SHARP_PIN);
  float adc = acc / (float)SHARP_SAMPLES;
  float v = adc / 4095.0f * 3.3f;                    // ESP32-S3 ADC 12 bits
  // Curva aproximada del GP2Y0A21 (10-80 cm). RECALIBRAR con tu sensor.
  float dist = (v > 0.2f) ? 27.86f * powf(v, -1.15f) : 80.0f;
  lastPos = dist - BEAM_CENTER_CM;                   // 0 = centro de la viga
  return lastPos;
}

// ---------------------------------------------------------------------------
//  Protocolo serial
// ---------------------------------------------------------------------------
void startRun(char *args) {
  char *name = strtok(args, ",");
  float p[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    char *tok = strtok(NULL, ",");
    if (tok) p[i] = atof(tok);
  }
  CtrlType tp = ctrlTypeFromName(name);
  if (tp == CT_NONE) return;
  ctrl.begin(tp, p[0], p[1], p[2], p[3], BALANCIN);   // modelo del balancín
  running = true;
  t = 0; steadyAcc = 0;
  tPrev = millis();
}

void stopRun() {
  running = false;
  beamLevel();                 // nivela la viga al terminar
  Serial.println("DONE");
}

void handleSerial() {
  static char buf[64];
  static int idx = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0'; idx = 0;
      if (!strncmp(buf, "START,", 6)) startRun(buf + 6);
      else if (!strcmp(buf, "STOP"))  stopRun();
    } else if (idx < (int)sizeof(buf) - 1) {
      buf[idx++] = ch;
    }
  }
}

// ---------------------------------------------------------------------------
//  Setup / Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  beamLevel();
}

void loop() {
  handleSerial();
  if (!running) return;

  unsigned long now = millis();
  if (now - tPrev < (unsigned long)(DT * 1000)) return;
  tPrev = now;

  float pos = readPosition();          // variable controlada (posición de la bola)
  float ref = ctrl.sp;                 // referencia (setpoint)
  float err = ref - pos;
  float u   = ctrl.step(pos, DT);      // señal de control
  applyControl(u);                     // -> ángulo del servo
  t += DT;

  // DATA,<t>,<ref>,<vc>,<error>,<u>
  Serial.print("DATA,");
  Serial.print(t, 3);   Serial.print(',');
  Serial.print(ref, 4); Serial.print(',');
  Serial.print(pos, 4); Serial.print(',');
  Serial.print(err, 4); Serial.print(',');
  Serial.println(u, 4);

  float tol = fabsf(ref) * 0.02f; if (tol < 0.3f) tol = 0.3f;
  if (fabsf(err) <= tol) steadyAcc += DT; else steadyAcc = 0;
  if (steadyAcc >= TOL_STEADY || t >= MAX_T) stopRun();
}
