/*
 * control_laws.h  —  Leyes de control (C++ puro, sin dependencias de Arduino)
 *
 * Replica exactamente las leyes del backend (backend/control_laws.py) para que
 * el ESP32 y el simulador se comporten igual. Al ser C++ puro, este header se
 * incluye tanto en el firmware (.ino) como en la prueba de escritorio
 * (test_host.cpp), de modo que probamos el MISMO código de control.
 *
 * Controladores y parámetros (p0..p3):
 *   PID    : setpoint, kp, ki, kd
 *   DIFUSA : setpoint, ke, kde, ku
 *   LQR    : setpoint, q_pos, q_vel, r
 *   SMC    : setpoint, lambda, eta, phi
 *
 * LQR y SMC son basados en modelo: usan el modelo de 2do orden de la planta.
 */
#ifndef CONTROL_LAWS_H
#define CONTROL_LAWS_H

#include <math.h>
#include <string.h>

enum CtrlType { CT_PID, CT_DIFUSA, CT_LQR, CT_SMC, CT_NONE };

// Modelo interno de la planta (por defecto, "carrito"), usado por LQR y SMC.
struct PlantModel { float K, wn, z; };

class Controller {
 public:
  CtrlType type = CT_NONE;
  float sp = 0, a = 0, b = 0, c = 0;   // sp + 3 parametros

  void begin(CtrlType t, float p0, float p1, float p2, float p3,
             PlantModel model = {1.0f, 1.6f, 0.55f}) {
    type = t; sp = p0; a = p1; b = p2; c = p3;
    _integ = 0; _prevE = 0; _hasPrev = false; _prevY = 0; _hasPrevY = false;
    _model = model;
    if (t == CT_LQR) _designLQR();
    if (t == CT_SMC) _designSMC();
  }

  // y = variable medida (posicion). Devuelve u (senal de control) en [-100,100].
  float step(float y, float dt) {
    float u = 0;
    switch (type) {
      case CT_PID:    u = _pid(y, dt);   break;
      case CT_DIFUSA: u = _fuzzy(y, dt); break;
      case CT_LQR:    u = _lqr(y, dt);   break;
      case CT_SMC:    u = _smc(y, dt);   break;
      default:        u = 0;             break;
    }
    if (u > 100) u = 100;
    if (u < -100) u = -100;
    return u;
  }

 private:
  float _integ = 0, _prevE = 0; bool _hasPrev = false;
  float _prevY = 0; bool _hasPrevY = false;
  PlantModel _model = {1.0f, 1.6f, 0.55f};
  float _k1 = 0, _k2 = 0, _ff = 0;   // LQR
  float _ueq = 0;                    // SMC

  // ---- PID ----
  float _pid(float y, float dt) {
    float e = sp - y;                        // a=kp, b=ki, c=kd
    _integ += e * dt;
    float de = _hasPrev ? (e - _prevE) / dt : 0.0f;
    _prevE = e; _hasPrev = true;
    return a * e + b * _integ + c * de;
  }

  // ---- Difuso PD (Mamdani 3x3 simplificado) ----
  static void _memberships(float x, float m[3]) {
    if (x > 1) x = 1; if (x < -1) x = -1;
    float neg = x < 0 ? -x : 0;
    float pos = x > 0 ? x : 0;
    float zero = 1 - fabsf(x); if (zero < 0) zero = 0;
    float s = neg + zero + pos;
    if (s == 0) { m[0] = 0; m[1] = 1; m[2] = 0; }
    else        { m[0] = neg / s; m[1] = zero / s; m[2] = pos / s; }
  }
  float _fuzzy(float y, float dt) {
    static const float RULES[3][3] = {{-1, -1, 0}, {-1, 0, 1}, {0, 1, 1}};
    float e = sp - y;                        // a=ke, b=kde, c=ku
    float de = _hasPrev ? (e - _prevE) / dt : 0.0f;
    _prevE = e; _hasPrev = true;
    float me[3], mde[3];
    _memberships(e * a * 0.01f, me);
    _memberships(de * b * 0.01f, mde);
    float num = 0, den = 0;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        float w = mde[i] * me[j];
        num += w * RULES[i][j];
        den += w;
      }
    float out = (den == 0) ? 0 : num / den;
    return out * c;                          // escala por ku
  }

  // ---- LQR: diseno (resuelve la DARE 2x2) ----
  void _designLQR() {
    float a0 = _model.wn * _model.wn;
    float a1 = 2 * _model.z * _model.wn;
    float bb = _model.K * _model.wn * _model.wn;
    float dt = 0.03f;
    float Ad[2][2] = {{1.0f, dt}, {-a0 * dt, 1.0f - a1 * dt}};
    float Bd[2] = {0.0f, bb * dt};
    float q_pos = a < 0 ? 0 : a, q_vel = b < 0 ? 0 : b;
    float R = c < 1e-3f ? 1e-3f : c;
    float P[2][2] = {{0, 0}, {0, 0}};
    for (int it = 0; it < 1000; it++) {
      float AtP[2][2] = {
        {Ad[0][0]*P[0][0] + Ad[1][0]*P[1][0], Ad[0][0]*P[0][1] + Ad[1][0]*P[1][1]},
        {Ad[0][1]*P[0][0] + Ad[1][1]*P[1][0], Ad[0][1]*P[0][1] + Ad[1][1]*P[1][1]}};
      float AtPA[2][2] = {
        {AtP[0][0]*Ad[0][0] + AtP[0][1]*Ad[1][0], AtP[0][0]*Ad[0][1] + AtP[0][1]*Ad[1][1]},
        {AtP[1][0]*Ad[0][0] + AtP[1][1]*Ad[1][0], AtP[1][0]*Ad[0][1] + AtP[1][1]*Ad[1][1]}};
      float AtPB[2] = {AtP[0][0]*Bd[0] + AtP[0][1]*Bd[1],
                       AtP[1][0]*Bd[0] + AtP[1][1]*Bd[1]};
      float BtPB = Bd[0]*(P[0][0]*Bd[0] + P[0][1]*Bd[1]) +
                   Bd[1]*(P[1][0]*Bd[0] + P[1][1]*Bd[1]);
      float S = R + BtPB;
      float Q[2][2] = {{q_pos, 0}, {0, q_vel}};
      float Pn[2][2]; float diff = 0;
      for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
          Pn[i][j] = Q[i][j] + AtPA[i][j] - AtPB[i] * AtPB[j] / S;
          float d = fabsf(Pn[i][j] - P[i][j]); if (d > diff) diff = d;
        }
      memcpy(P, Pn, sizeof(P));
      if (diff < 1e-9f) break;
    }
    float BtP[2] = {P[0][0]*Bd[0] + P[1][0]*Bd[1], P[0][1]*Bd[0] + P[1][1]*Bd[1]};
    float BtPB = Bd[0]*(P[0][0]*Bd[0] + P[0][1]*Bd[1]) +
                 Bd[1]*(P[1][0]*Bd[0] + P[1][1]*Bd[1]);
    float S = R + BtPB;
    _k1 = (BtP[0]*Ad[0][0] + BtP[1]*Ad[1][0]) / S;
    _k2 = (BtP[0]*Ad[0][1] + BtP[1]*Ad[1][1]) / S;
    _ff = (bb != 0) ? a0 / bb : 0.0f;
  }
  float _lqr(float y, float dt) {
    float vel = _hasPrevY ? (y - _prevY) / dt : 0.0f;
    _prevY = y; _hasPrevY = true;
    return _ff * sp + _k1 * (sp - y) - _k2 * vel;
  }

  // ---- SMC: control por modos deslizantes ----
  void _designSMC() {
    float bb = _model.K * _model.wn * _model.wn;
    float a0 = _model.wn * _model.wn;
    _ueq = (bb != 0) ? a0 / bb : 0.0f;
  }
  float _smc(float y, float dt) {
    float e = sp - y;                        // a=lambda, b=eta, c=phi
    float de = _hasPrev ? (e - _prevE) / dt : 0.0f;
    _prevE = e; _hasPrev = true;
    float phi = c < 1e-3f ? 1e-3f : c;
    float s = a * e + de;
    float sat = s / phi; if (sat > 1) sat = 1; if (sat < -1) sat = -1;
    return _ueq * sp + b * sat;              // b = eta
  }
};

// Mapea el nombre recibido por serial al tipo de controlador.
inline CtrlType ctrlTypeFromName(const char *s) {
  if (!strcmp(s, "pid"))    return CT_PID;
  if (!strcmp(s, "difusa")) return CT_DIFUSA;
  if (!strcmp(s, "lqr"))    return CT_LQR;
  if (!strcmp(s, "smc"))    return CT_SMC;
  return CT_NONE;
}

#endif  // CONTROL_LAWS_H
