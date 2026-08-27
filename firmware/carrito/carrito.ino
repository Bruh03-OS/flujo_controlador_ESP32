/*
 * carrito.ino — Firmware del carrito seguidor de línea
 * Plataforma experimental modular de control · ESP32-S3-N16R8
 *
 * Prototipo        : carrito
 * Variable control : posición de la línea (offset lateral bajo el arreglo IR)
 * Sensor           : TCRT-5000, arreglo IR de 5 canales (OUT1..OUT5)
 * Driver           : TB6612FNG  ->  2x motor DC 6V 100RPM (tracción diferencial)
 *
 * Protocolo serial (115200 baud) — idéntico al backend:
 *   PC -> ESP32 : START,<controlador>,<p1>,<p2>,<p3>,<p4>\n
 *   PC -> ESP32 : STOP\n
 *   ESP32 -> PC : DATA,<t>,<ref>,<vc>,<error>,<u>\n
 *   ESP32 -> PC : DONE\n
 *
 * Controladores : pid | difusa | lqr | smc  (ver control_laws.h)
 *
 * Requiere Arduino-ESP32 core 3.x (API ledcAttach / ledcWrite por pin).
 */
#include "control_laws.h"

// ===========================================================================
//  PINES  —  VERIFICAR CONTRA EL DIAGRAMA DE CIRKIT ANTES DE CARGAR
//  (leídos del diagrama; si alguno no coincide, ajústalo aquí y ya)
// ===========================================================================
// Sensor IR TCRT-5000: OUT1..OUT5 (de izquierda a derecha del carrito)
const int IR_PINS[5] = {4, 5, 6, 7, 15};

// Driver TB6612FNG
const int AIN1 = 9,  AIN2 = 46, PWMA = 3;    // Motor A (izquierdo)
const int BIN1 = 11, BIN2 = 12, PWMB = 13;   // Motor B (derecho)
const int STBY = 10;                          // habilitación del driver

// ===========================================================================
//  CONFIGURACIÓN
// ===========================================================================
#define USE_ANALOG            0     // 0 = salidas digitales, 1 = salidas analógicas
#define LINE_DETECTED_LEVEL   HIGH  // nivel del pin cuando el sensor VE la línea
const float  SENSOR_WEIGHTS[5] = {-100, -50, 0, 50, 100};  // posición por canal
const float  BASE_SPEED = 45.0f;    // avance base (0..100); el control corrige el giro

const float  DT          = 0.03f;   // periodo de muestreo (s)  ~33 Hz
const float  MAX_T       = 25.0f;   // tope de seguridad (s)
const float  TOL_STEADY  = 1.0f;    // s dentro de tolerancia para declarar DONE
const int    PWM_FREQ    = 20000;   // 20 kHz (fuera del rango audible)
const int    PWM_RES     = 8;       // 8 bits -> duty 0..255

// ===========================================================================
//  ESTADO
// ===========================================================================
Controller ctrl;
bool   running = false;
float  t = 0, steadyAcc = 0, lastPos = 0;
unsigned long tPrev = 0;

// ---------------------------------------------------------------------------
//  Motores
// ---------------------------------------------------------------------------
void motorWrite(int in1, int in2, int pwmPin, float speed) {
  bool fwd = speed >= 0;
  int duty = (int)(fabsf(speed) / 100.0f * 255.0f);
  if (duty > 255) duty = 255;
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW  : HIGH);
  ledcWrite(pwmPin, duty);
}

// u = señal de control (giro). Diferencial sobre la velocidad base.
void applyControl(float u) {
  float left  = BASE_SPEED + u;
  float right = BASE_SPEED - u;
  left  = constrain(left,  -100.0f, 100.0f);
  right = constrain(right, -100.0f, 100.0f);
  motorWrite(AIN1, AIN2, PWMA, left);
  motorWrite(BIN1, BIN2, PWMB, right);
}

void motorsStop() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

// ---------------------------------------------------------------------------
//  Sensor: estima la posición de la línea (centroide ponderado, -100..100)
// ---------------------------------------------------------------------------
float readPosition() {
#if USE_ANALOG
  float num = 0, den = 0;
  for (int i = 0; i < 5; i++) {
    int v = analogRead(IR_PINS[i]);          // 0..4095
    num += SENSOR_WEIGHTS[i] * v;
    den += v;
  }
  if (den < 1) return lastPos;               // línea perdida: mantener última
  lastPos = num / den;
  return lastPos;
#else
  float num = 0; int cnt = 0;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(IR_PINS[i]) == LINE_DETECTED_LEVEL) {
      num += SENSOR_WEIGHTS[i];
      cnt++;
    }
  }
  if (cnt == 0) return lastPos;              // línea perdida: mantener última
  lastPos = num / cnt;
  return lastPos;
#endif
}

// ---------------------------------------------------------------------------
//  Protocolo serial
// ---------------------------------------------------------------------------
void startRun(char *args) {
  // args = "<controlador>,<p1>,<p2>,<p3>,<p4>"
  char *name = strtok(args, ",");
  float p[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    char *tok = strtok(NULL, ",");
    if (tok) p[i] = atof(tok);
  }
  CtrlType tp = ctrlTypeFromName(name);
  if (tp == CT_NONE) return;

  ctrl.begin(tp, p[0], p[1], p[2], p[3]);
  running = true;
  t = 0; steadyAcc = 0; lastPos = 0;
  tPrev = millis();
  digitalWrite(STBY, HIGH);                  // habilita el driver
}

void stopRun(const char *reason) {
  running = false;
  motorsStop();
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
      if (!strncmp(buf, "START,", 6))      startRun(buf + 6);
      else if (!strcmp(buf, "STOP"))       stopRun("manual");
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
  for (int i = 0; i < 5; i++) pinMode(IR_PINS[i], INPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, LOW);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  motorsStop();
}

void loop() {
  handleSerial();
  if (!running) return;

  unsigned long now = millis();
  if (now - tPrev < (unsigned long)(DT * 1000)) return;
  tPrev = now;

  float pos = readPosition();          // variable controlada
  float ref = ctrl.sp;                 // referencia (setpoint)
  float err = ref - pos;
  float u   = ctrl.step(pos, DT);      // señal de control
  applyControl(u);
  t += DT;

  // DATA,<t>,<ref>,<vc>,<error>,<u>
  Serial.print("DATA,");
  Serial.print(t, 3);     Serial.print(',');
  Serial.print(ref, 4);   Serial.print(',');
  Serial.print(pos, 4);   Serial.print(',');
  Serial.print(err, 4);   Serial.print(',');
  Serial.println(u, 4);

  // ¿setpoint alcanzado de forma sostenida?
  float tol = fabsf(ref) * 0.02f; if (tol < 0.3f) tol = 0.3f;
  if (fabsf(err) <= tol) steadyAcc += DT; else steadyAcc = 0;

  if (steadyAcc >= TOL_STEADY || t >= MAX_T) stopRun("setpoint");
}
