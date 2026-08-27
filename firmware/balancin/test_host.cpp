/*
 * test_host.cpp — Prueba de escritorio (software-in-the-loop)
 *
 * Compila el MISMO control_laws.h que usa el firmware y lo corre en lazo
 * cerrado contra el modelo de planta del carrito. Sirve para verificar, sin
 * ESP32, que las cuatro leyes de control funcionan y convergen igual que en
 * el backend de Python.
 *
 *   g++ -O2 -o test_host test_host.cpp && ./test_host
 */
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include "control_laws.h"

const float DT = 0.03f, MAX_T = 25.0f, TOL_STEADY = 1.0f;
const PlantModel BALANCIN = {1.0f, 2.2f, 0.18f};

struct Metrics { float t, rms, overshoot, ess; const char* reason; };

Metrics run(CtrlType tp, float p0, float p1, float p2, float p3) {
  Controller c;
  c.begin(tp, p0, p1, p2, p3, BALANCIN);

  float y = 0, yd = 0, t = 0, steady = 0;
  float y0 = 0, sp = p0;
  std::vector<float> ref, vc, err;
  const char* reason = "timeout";

  for (int step = 0; step < 5000; step++) {
    float u = c.step(y, DT);
    if (u > 100) u = 100; if (u < -100) u = -100;
    // planta 2do orden (idéntica al backend)
    float ydd = BALANCIN.K * BALANCIN.wn * BALANCIN.wn * u
              - 2 * BALANCIN.z * BALANCIN.wn * yd
              - BALANCIN.wn * BALANCIN.wn * y;
    yd += ydd * DT;
    y  += yd * DT;
    t  += DT;

    float e = sp - y;
    ref.push_back(sp); vc.push_back(y); err.push_back(e);

    float tol = fabsf(sp) * 0.02f; if (tol < 0.3f) tol = 0.3f;
    if (fabsf(e) <= tol) steady += DT; else steady = 0;
    if (steady >= TOL_STEADY) { reason = "setpoint"; break; }
    if (t >= MAX_T)           { reason = "timeout";  break; }
  }

  // métricas (mismas fórmulas que backend/metrics.py)
  int n = err.size();
  float sumsq = 0; for (float e : err) sumsq += e * e;
  float rms = sqrtf(sumsq / n);
  float salto = sp - y0, over = 0;
  if (fabsf(salto) > 1e-9f) {
    float peak = vc[0];
    for (float v : vc) if ((salto > 0 && v > peak) || (salto < 0 && v < peak)) peak = v;
    over = (salto > 0 ? (peak - sp) / salto : (sp - peak) / (-salto)) * 100.0f;
    if (over < 0) over = 0;
  }
  int cola = n * 0.15f; if (cola < 1) cola = 1;
  float ess = 0; for (int i = n - cola; i < n; i++) ess += err[i];
  ess = fabsf(ess / cola);

  return { t, rms, over, ess, reason };
}

int main() {
  printf("%-11s %-9s %8s %8s %9s %9s %10s\n",
         "controlador", "reason", "t(s)", "vc_fin", "rms", "sobre%", "ess");
  printf("--------------------------------------------------------------------------\n");

  struct Case { const char* name; CtrlType tp; float p[4]; };
  Case cases[] = {
    {"pid",    CT_PID,    {20,  5,   1.5f, 1.5f}},
    {"difusa", CT_DIFUSA, {20, 70,  40,  60}},
    {"lqr",    CT_LQR,    {20, 120,  8,  1}},
    {"smc",    CT_SMC,    {20,  4,  50,  6}},
  };

  for (auto& cs : cases) {
    Metrics m = run(cs.tp, cs.p[0], cs.p[1], cs.p[2], cs.p[3]);
    printf("%-11s %-9s %8.2f %8s %9.3f %9.2f %10.4f\n",
           cs.name, m.reason, m.t, "", m.rms, m.overshoot, m.ess);
  }
  return 0;
}
