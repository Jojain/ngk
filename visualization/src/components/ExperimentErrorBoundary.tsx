import { Component, type ErrorInfo, type ReactNode } from "react";

type Props = {
  resetKey: string | null;
  children: ReactNode;
};

type State = {
  error: unknown;
  resetKey: string | null;
};

export default class ExperimentErrorBoundary extends Component<Props, State> {
  state: State = {
    error: null,
    resetKey: this.props.resetKey,
  };

  static getDerivedStateFromError(error: unknown): Partial<State> {
    return { error };
  }

  static getDerivedStateFromProps(props: Props, state: State): Partial<State> | null {
    if (props.resetKey !== state.resetKey) {
      return {
        error: null,
        resetKey: props.resetKey,
      };
    }
    return null;
  }

  componentDidCatch(error: unknown, info: ErrorInfo) {
    console.error("Experiment render failed", error, info);
  }

  render() {
    if (this.state.error) {
      return (
        <div className="experiment-error" role="alert">
          <strong>Experiment error</strong>
          <span>{formatError(this.state.error)}</span>
        </div>
      );
    }

    return this.props.children;
  }
}

function formatError(error: unknown): string {
  if (error instanceof Error) return error.message;
  if (typeof error === "string") return error;
  try {
    return JSON.stringify(error);
  } catch {
    return String(error);
  }
}
