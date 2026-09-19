"""Release-version policy for the repository release helper."""

import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

from typer.testing import CliRunner

SCRIPT = Path(__file__).parents[2] / "release" / "ngk_release" / "__init__.py"
SPEC = importlib.util.spec_from_file_location("release_tool", SCRIPT)
assert SPEC is not None
assert SPEC.loader is not None
RELEASE_TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RELEASE_TOOL)

sys.path.insert(0, str(Path(__file__).parents[2] / "profiling"))


RUNNER = CliRunner()


class VersionBumpTests(unittest.TestCase):
    def test_bump_help_exposes_the_typer_release_option(self) -> None:
        result = RUNNER.invoke(RELEASE_TOOL.app, ["--help"])

        self.assertEqual(result.exit_code, 0, result.output)
        self.assertIn("--release", result.output)

    def test_bump_defaults_to_a_patch_increment(self) -> None:
        with patch.object(
            RELEASE_TOOL, "apply_bump", return_value="0.0.2"
        ) as apply_bump:
            result = RUNNER.invoke(RELEASE_TOOL.app, [])

        self.assertEqual(result.exit_code, 0, result.output)
        apply_bump.assert_called_once_with(
            Path(RELEASE_TOOL.__file__).resolve().parents[2], "patch", False
        )

    def test_bump_accepts_a_component_and_release_flag(self) -> None:
        with patch.object(
            RELEASE_TOOL, "apply_bump", return_value="0.1.0"
        ) as apply_bump:
            result = RUNNER.invoke(RELEASE_TOOL.app, ["minor", "--release"])

        self.assertEqual(result.exit_code, 0, result.output)
        apply_bump.assert_called_once_with(
            Path(RELEASE_TOOL.__file__).resolve().parents[2], "minor", True
        )

    def test_bump_version_advances_exactly_one_semantic_component(self) -> None:
        self.assertEqual(RELEASE_TOOL.bump_version("0.0.1", "patch"), "0.0.2")
        self.assertEqual(RELEASE_TOOL.bump_version("0.0.1", "minor"), "0.1.0")
        self.assertEqual(RELEASE_TOOL.bump_version("0.0.1", "major"), "1.0.0")

    def test_version_field_replacement_is_scoped_to_its_toml_section(self) -> None:
        text = '[package]\nversion = "0.0.1"\n\n[project]\nversion = "0.0.1"\n'

        self.assertEqual(RELEASE_TOOL.read_version_file(text, "package"), "0.0.1")
        self.assertEqual(
            RELEASE_TOOL.replace_version(text, "project", "0.0.2"),
            '[package]\nversion = "0.0.1"\n\n[project]\nversion = "0.0.2"\n',
        )


if __name__ == "__main__":
    unittest.main()
