"""
Backend FastAPI: puente entre el navegador (WebSocket) y la planta
(simulador o ESP32 por serial), cálculo de métricas y persistencia.

Correr:  uvicorn main:app --reload --port 8000   (desde la carpeta backend)
"""
import asyncio
import json

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
import os

import controllers as ctrl
import control_laws
import source as src
import metrics as mx
import database as db

# --- Configuración ----------------------------------------------------------
# Cambia a False y ajusta SERIAL_PORT cuando conectes el ESP32 real.
USE_SIMULATOR = True
SERIAL_PORT = "/dev/ttyUSB0"   # Windows: "COM3"

app = FastAPI(title="Flujo Controlador ESP32")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
def _startup():
    db.init_db()


@app.get("/api/meta")
def get_meta():
    return ctrl.meta()


@app.get("/api/logs")
def get_logs():
    return db.last_runs(50)


@app.get("/api/logs/{run_id}")
def get_log(run_id: int):
    run = db.get_run(run_id)
    if not run:
        return JSONResponse({"error": "no encontrado"}, status_code=404)
    return run


def _build_source(prototipo_id, controlador_id, params):
    prototipo = ctrl.PROTOTIPOS[prototipo_id]
    if USE_SIMULATOR:
        controller = control_laws.build_controller(controlador_id, params, prototipo["planta"])
        return src.SimulatorSource(prototipo, controlador_id, params, controller)
    return src.Esp32SerialSource(prototipo, controlador_id, params, SERIAL_PORT)


@app.websocket("/ws")
async def ws_control(ws: WebSocket):
    await ws.accept()
    stop_flag = {"stop": False}

    async def _listen():
        # escucha mensajes del cliente (p. ej. Finalizar) en paralelo
        try:
            while True:
                msg = await ws.receive_text()
                data = json.loads(msg)
                if data.get("action") == "stop":
                    stop_flag["stop"] = True
                    return
        except WebSocketDisconnect:
            stop_flag["stop"] = True

    try:
        # primer mensaje: configuración de la corrida
        cfg = json.loads(await ws.receive_text())
        prototipo_id = cfg["prototipo"]
        controlador_id = cfg["controlador"]
        params = cfg["params"]

        source = _build_source(prototipo_id, controlador_id, params)
        listener = asyncio.create_task(_listen())

        reason = "manual"
        while not stop_flag["stop"]:
            tel = source.step()
            if tel is None:
                await asyncio.sleep(src.DT)
                continue
            await ws.send_text(json.dumps({"type": "telemetry", **tel}))
            if tel.get("done"):
                reason = tel.get("reason") or "setpoint"
                break
            await asyncio.sleep(src.DT)

        listener.cancel()

        # cierre serial si aplica
        if hasattr(source, "stop"):
            source.stop()

        # métricas + guardado
        T, REF, VC, ERR = source.arrays()
        metrics = mx.compute_metrics(T, REF, VC, ERR)
        telemetria = {"t": T, "ref": REF, "vc": VC, "error": ERR}
        run_id = db.save_run(prototipo_id, controlador_id, params, metrics, telemetria)

        await ws.send_text(json.dumps({
            "type": "done",
            "reason": reason,
            "metrics": metrics,
            "run_id": run_id,
        }))
    except WebSocketDisconnect:
        pass
    finally:
        try:
            await ws.close()
        except Exception:
            pass


# --- Servir el build de React (producción) ----------------------------------
DIST = os.path.join(os.path.dirname(__file__), "..", "frontend", "dist")
if os.path.isdir(DIST):
    app.mount("/assets", StaticFiles(directory=os.path.join(DIST, "assets")), name="assets")

    @app.get("/{full_path:path}")
    async def spa(full_path: str):
        index = os.path.join(DIST, "index.html")
        return FileResponse(index)
