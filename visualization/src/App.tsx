import { Suspense, useEffect, useState } from "react";
import { Leva } from "leva";
import ExperimentSidebar from "./components/ExperimentSidebar";
import SceneShell from "./components/SceneShell";
import { experiments } from "./experiments/registry";
import { useKernel } from "./kernel/useKernel";

function getHashId(): string | null {
  const h = window.location.hash.replace(/^#\/?/, "");
  return h || null;
}

export default function App() {
  const [activeId, setActiveId] = useState<string | null>(
    () => getHashId() ?? experiments[0]?.id ?? null,
  );
  const kernel = useKernel();

  useEffect(() => {
    const onHash = () => setActiveId(getHashId());
    window.addEventListener("hashchange", onHash);
    return () => window.removeEventListener("hashchange", onHash);
  }, []);

  useEffect(() => {
    if (activeId) window.location.hash = `/${activeId}`;
  }, [activeId]);

  const active = experiments.find((e) => e.id === activeId) ?? null;
  const Active = active?.component ?? null;

  return (
    <div className="app">
      <ExperimentSidebar
        experiments={experiments}
        activeId={activeId}
        onSelect={setActiveId}
      />
      <div className="viewport">
        {!kernel && <div className="loading">loading kernel…</div>}
        <Leva collapsed={false} />
        <SceneShell>
          {Active && kernel ? (
            <Suspense fallback={null}>
              <Active />
            </Suspense>
          ) : null}
        </SceneShell>
        {!Active && (
          <div className="empty-state">
            <span>select an experiment</span>
          </div>
        )}
      </div>
    </div>
  );
}
