import type { ExperimentMeta } from "../experiments/registry";

type Props = {
  experiments: ExperimentMeta[];
  activeId: string | null;
  onSelect: (id: string) => void;
};

export default function ExperimentSidebar({ experiments, activeId, onSelect }: Props) {
  const groups = new Map<string, ExperimentMeta[]>();
  for (const exp of experiments) {
    const key = exp.group ?? "Other";
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key)!.push(exp);
  }

  return (
    <aside className="sidebar">
      <h1>ngk playground</h1>
      {Array.from(groups.entries()).map(([group, items]) => (
        <div key={group}>
          <div className="group">{group}</div>
          {items.map((exp) => (
            <button
              key={exp.id}
              className={activeId === exp.id ? "active" : ""}
              onClick={() => onSelect(exp.id)}
            >
              {exp.title}
            </button>
          ))}
        </div>
      ))}
    </aside>
  );
}
