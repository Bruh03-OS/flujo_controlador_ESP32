import { useEffect, useRef, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useStore } from "../store.jsx";
import { wsUrl } from "../api.js";
import StepBar from "../components/StepBar.jsx";
import Scope from "../components/Scope.jsx";

const EMPTY_TEL = { t: 0, ref: 0, vc: 0, error: 0, u: 0 };

export default function Control() {
  const nav = useNavigate();
  const { ready, controladorDef, prototipo, controlador, params, setParams } = useStore();

  const [running, setRunning] = useState(false);
  const [data, setData] = useState([]);       // historial de telemetría
  const [last, setLast] = useState(EMPTY_TEL);
  const [finished, setFinished] = useState(null); // {reason, metrics, run_id}
  const wsRef = useRef(null);

  useEffect(() => {
    if (!ready) nav("/");
  }, [ready, nav]);

  useEffect(() => () => wsRef.current?.close(), []);

  if (!controladorDef) return null;

  // Validación: todos los campos con número válido dentro de rango
  const validate = (key, raw) => {
    const def = controladorDef.params.find((p) => p.key === key);
    if (raw === "" || raw === "-" || isNaN(Number(raw))) return false;
    const v = Number(raw);
    if (def.min !== undefined && v < def.min) return false;
    if (def.max !== undefined && v > def.max) return false;
    return true;
  };
  const allValid = controladorDef.params.every((p) => validate(p.key, params[p.key]));

  const onChange = (key, val) => setParams({ ...params, [key]: val });

  const start = () => {
    setData([]); setLast(EMPTY_TEL); setFinished(null);
    const ws = new WebSocket(wsUrl());
    wsRef.current = ws;
    ws.onopen = () => {
      const numeric = {};
      controladorDef.params.forEach((p) => (numeric[p.key] = Number(params[p.key])));
      ws.send(JSON.stringify({ prototipo, controlador, params: numeric }));
      setRunning(true);
    };
    ws.onmessage = (ev) => {
      const m = JSON.parse(ev.data);
      if (m.type === "telemetry") {
        setLast(m);
        setData((d) => [...d, m]);
      } else if (m.type === "done") {
        setRunning(false);
        setFinished({ reason: m.reason, metrics: m.metrics, run_id: m.run_id });
        ws.close();
      }
    };
    ws.onerror = () => setRunning(false);
  };

  const finalizar = () => wsRef.current?.send(JSON.stringify({ action: "stop" }));

  const reasonText = finished?.reason === "setpoint"
    ? "El sistema alcanzó el setpoint y se estabilizó."
    : finished?.reason === "timeout"
    ? "Se alcanzó el tope de duración sin estabilizar por completo."
    : "El proceso fue detenido manualmente.";

  return (
    <>
      <StepBar current="/control" />
      <div className="page">
        <div className="page-head">
          <h1>Control en tiempo real</h1>
          <p>Ingresa los parámetros del controlador {controladorDef.nombre}. Al iniciar verás la respuesta del prototipo en vivo.</p>
        </div>

        <div className="control-grid">
          {/* Formulario */}
          <div className="panel">
            <h2>Parámetros · {controladorDef.nombre}</h2>
            {controladorDef.params.map((p) => {
              const bad = params[p.key] !== "" && !validate(p.key, params[p.key]);
              return (
                <div className="field" key={p.key}>
                  <label>
                    {p.label}
                    {p.unidad ? <span className="u">{p.unidad}</span> : null}
                  </label>
                  <input
                    type="number"
                    className={bad ? "bad" : ""}
                    value={params[p.key] ?? "0"}
                    disabled={running}
                    onChange={(e) => onChange(p.key, e.target.value)}
                  />
                  {bad && (
                    <div className="hint">
                      Rango {p.min ?? "-∞"} a {p.max ?? "∞"}
                    </div>
                  )}
                </div>
              );
            })}

            <div className="start-slot">
              {!running && allValid && (
                <button className="btn primary block fade-in" onClick={start}>
                  Iniciar
                </button>
              )}
              {running && (
                <button className="btn danger block" onClick={finalizar}>
                  Finalizar
                </button>
              )}
              {!running && !allValid && (
                <p style={{ fontSize: 12.5, color: "var(--ink-faint)", fontFamily: "var(--f-mono)", margin: 0 }}>
                  Completa todos los campos para iniciar.
                </p>
              )}
            </div>
          </div>

          {/* Osciloscopio + lecturas */}
          <div>
            <div className="scope-wrap">
              <div className="scope-head">
                <span className="title">Respuesta del sistema</span>
                <span className={`status ${running ? "live" : ""}`}>
                  <span className="led" />
                  {running ? "ADQUIRIENDO" : finished ? "DETENIDO" : "EN ESPERA"}
                </span>
              </div>
              <div className="legend">
                <span><i style={{ background: "#e8a33d" }} />Referencia</span>
                <span><i style={{ background: "#12b5ac" }} />Variable controlada</span>
                <span><i style={{ background: "#e5484d" }} />Error</span>
                <span><i style={{ background: "#5b8cf5" }} />Señal de control</span>
              </div>
              <Scope data={data} />
              <div className="readouts">
                <Readout k="Referencia" v={last.ref} />
                <Readout k="Var. controlada" v={last.vc} />
                <Readout k="Error" v={last.error} color="var(--sig-err)" />
                <Readout k="Señal control" v={last.u} color="var(--sig-u)" />
                <Readout k="Tiempo" v={last.t} unit="s" />
              </div>
            </div>
          </div>
        </div>
      </div>

      {finished && (
        <div className="overlay">
          <div className="modal">
            <span className="warn-badge">⚠ PROCESO FINALIZADO</span>
            <h3>Corrida completada</h3>
            <p>{reasonText}</p>
            <div className="mini-metrics">
              <div><div className="k">Tiempo de ejecución</div><div className="v">{finished.metrics.tiempo_ejecucion} s</div></div>
              <div><div className="k">Error RMS</div><div className="v">{finished.metrics.error_rms}</div></div>
              <div><div className="k">Sobreimpulso</div><div className="v">{finished.metrics.sobreimpulso} %</div></div>
              <div><div className="k">Error estacionario</div><div className="v">{finished.metrics.error_estacionario}</div></div>
            </div>
            <div className="actions-row">
              <button className="btn ghost" onClick={() => setFinished(null)}>Cerrar</button>
              <button className="btn primary" onClick={() => nav("/datos")}>Continuar a datos</button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}

function Readout({ k, v, unit = "", color }) {
  const num = typeof v === "number" ? v.toFixed(2) : v;
  return (
    <div className="readout">
      <div className="k">{k}</div>
      <div className="v" style={{ color: color || "var(--panel-ink)" }}>
        {num}<span style={{ fontSize: 12, color: "#6d8480" }}>{unit ? " " + unit : ""}</span>
      </div>
    </div>
  );
}
