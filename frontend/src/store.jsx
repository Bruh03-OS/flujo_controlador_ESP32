import { createContext, useContext, useEffect, useState } from "react";
import { fetchMeta } from "./api.js";

const StoreCtx = createContext(null);

export function StoreProvider({ children }) {
  const [meta, setMeta] = useState(null);
  const [error, setError] = useState(null);

  const [prototipo, setPrototipo] = useState(null);   // id
  const [controlador, setControlador] = useState(null); // id
  const [params, setParams] = useState({});           // { key: valor }

  useEffect(() => {
    fetchMeta().then(setMeta).catch((e) => setError(e.message));
  }, []);

  const controladorDef = meta?.controladores.find((c) => c.id === controlador) || null;
  const prototipoDef = meta?.prototipos.find((p) => p.id === prototipo) || null;

  // Inicializa los parámetros en 0 al elegir un controlador
  function chooseControlador(id) {
    setControlador(id);
    const def = meta?.controladores.find((c) => c.id === id);
    const init = {};
    def?.params.forEach((p) => (init[p.key] = "0"));
    setParams(init);
  }

  const value = {
    meta, error,
    prototipo, setPrototipo,
    controlador, chooseControlador,
    controladorDef, prototipoDef,
    params, setParams,
    ready: Boolean(prototipo && controlador),
  };
  return <StoreCtx.Provider value={value}>{children}</StoreCtx.Provider>;
}

export const useStore = () => useContext(StoreCtx);
