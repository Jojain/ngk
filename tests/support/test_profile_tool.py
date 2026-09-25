"""Command-line behavior of the NGK sampling profiler."""

import importlib
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from typer.testing import CliRunner

sys.path.insert(0, str(Path(__file__).parents[2] / "profiling"))
PROFILE_TOOL = importlib.import_module("ngk_profile.profile")
RUNNER = CliRunner()


class RustExampleProfilingTests(unittest.TestCase):
    def test_rust_example_file_path_resolves_from_repo_root(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            example_dir = repo / "examples"
            example_dir.mkdir()
            example = example_dir / "sample.rs"
            example.write_text("fn main() {}\n", encoding="utf-8")
            with (
                patch.object(PROFILE_TOOL, "REPO_ROOT", repo),
                patch.object(PROFILE_TOOL, "RUST_EXAMPLES_DIR", example_dir),
            ):
                self.assertEqual(PROFILE_TOOL.resolve_script("examples/sample.rs"), example)

    def test_rust_example_builds_with_symbols_and_records_native_executable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            example_dir = repo / "examples"
            example_dir.mkdir()
            (example_dir / "sample.rs").write_text("fn main() {}\n", encoding="utf-8")

            with (
                patch.object(PROFILE_TOOL, "REPO_ROOT", repo),
                patch.object(PROFILE_TOOL, "RUST_EXAMPLES_DIR", example_dir),
                patch.object(PROFILE_TOOL, "find_tool", side_effect=lambda name, env: name),
                patch.object(PROFILE_TOOL, "run") as run,
                patch.object(PROFILE_TOOL, "absolutize_file", return_value=1),
            ):
                result = RUNNER.invoke(PROFILE_TOOL.app, ["sample", "--no-open"])

        self.assertEqual(result.exit_code, 0, result.output)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(commands[0], ["cargo", "build", "--profile", "profiling", "--example", "sample"])
        self.assertEqual(commands[1][0:2], ["samply", "record"])
        self.assertEqual(commands[1][-2], "--")
        expected_executable = repo / "target" / "profiling" / "examples" / (
            "sample.exe" if PROFILE_TOOL.IS_WINDOWS else "sample"
        )
        self.assertEqual(commands[1][-1], expected_executable)

    def test_python_recorder_is_rejected_for_rust_example(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            example_dir = repo / "examples"
            example_dir.mkdir()
            (example_dir / "sample.rs").write_text("fn main() {}\n", encoding="utf-8")
            with (
                patch.object(PROFILE_TOOL, "REPO_ROOT", repo),
                patch.object(PROFILE_TOOL, "RUST_EXAMPLES_DIR", example_dir),
                patch.object(PROFILE_TOOL, "run") as run,
            ):
                result = RUNNER.invoke(PROFILE_TOOL.app, ["sample", "--tool", "py-spy"])

        self.assertNotEqual(result.exit_code, 0)
        self.assertIn("py-spy", str(result.exception))
        run.assert_not_called()

    def test_rust_repeat_records_multiple_executions(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            example_dir = repo / "examples"
            example_dir.mkdir()
            (example_dir / "sample.rs").write_text("fn main() {}\n", encoding="utf-8")
            with (
                patch.object(PROFILE_TOOL, "REPO_ROOT", repo),
                patch.object(PROFILE_TOOL, "RUST_EXAMPLES_DIR", example_dir),
                patch.object(PROFILE_TOOL, "find_tool", side_effect=lambda name, env: name),
                patch.object(PROFILE_TOOL, "run") as run,
                patch.object(PROFILE_TOOL, "absolutize_file", return_value=1),
            ):
                result = RUNNER.invoke(PROFILE_TOOL.app, ["sample", "20", "--no-open"])

        self.assertEqual(result.exit_code, 0, result.output)
        record = run.call_args_list[1].args[0]
        self.assertIn(["--iteration-count", "20"], [record[i : i + 2] for i in range(len(record) - 1)])


class PythonScriptProfilingTests(unittest.TestCase):
    def test_python_script_still_builds_extension_and_runs_in_venv(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            example_dir = repo / "bindings" / "python" / "examples"
            example_dir.mkdir(parents=True)
            (example_dir / "sample.py").write_text("def main(): pass\n", encoding="utf-8")
            python = repo / ".venv" / ("Scripts" if PROFILE_TOOL.IS_WINDOWS else "bin") / (
                "python.exe" if PROFILE_TOOL.IS_WINDOWS else "python"
            )
            python.parent.mkdir(parents=True)
            python.touch()
            with (
                patch.object(PROFILE_TOOL, "REPO_ROOT", repo),
                patch.object(PROFILE_TOOL, "EXAMPLES_DIR", example_dir),
                patch.object(PROFILE_TOOL, "find_tool", side_effect=lambda name, env: name),
                patch.object(PROFILE_TOOL, "run") as run,
                patch.object(PROFILE_TOOL, "absolutize_file", return_value=1),
            ):
                result = RUNNER.invoke(PROFILE_TOOL.app, ["sample", "--no-open"])

        self.assertEqual(result.exit_code, 0, result.output)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(commands[0], ["maturin", "develop", "--profile", "profiling"])
        self.assertEqual(commands[1][-3:], ["--", python, example_dir / "sample.py"])

    def test_python_repeat_stays_in_one_process(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            example_dir = repo / "bindings" / "python" / "examples"
            example_dir.mkdir(parents=True)
            script = example_dir / "sample.py"
            script.write_text("def main(): pass\n", encoding="utf-8")
            python = repo / ".venv" / ("Scripts" if PROFILE_TOOL.IS_WINDOWS else "bin") / (
                "python.exe" if PROFILE_TOOL.IS_WINDOWS else "python"
            )
            python.parent.mkdir(parents=True)
            python.touch()
            with (
                patch.object(PROFILE_TOOL, "REPO_ROOT", repo),
                patch.object(PROFILE_TOOL, "EXAMPLES_DIR", example_dir),
                patch.object(PROFILE_TOOL, "find_tool", side_effect=lambda name, env: name),
                patch.object(PROFILE_TOOL, "run") as run,
                patch.object(PROFILE_TOOL, "absolutize_file", return_value=1),
            ):
                result = RUNNER.invoke(PROFILE_TOOL.app, ["sample", "3", "--no-open"])

        self.assertEqual(result.exit_code, 0, result.output)
        record = run.call_args_list[1].args[0]
        self.assertNotIn("--iteration-count", record)
        self.assertEqual(record[-4:], [python, Path(PROFILE_TOOL.__file__).parent / "runner.py", script, "3"])


if __name__ == "__main__":
    unittest.main()
