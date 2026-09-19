"""Bump NGK's release version and optionally commit and tag the result."""

from __future__ import annotations

import re
import subprocess
from enum import Enum
from pathlib import Path
from typing import Annotated

import typer


STABLE_VERSION = r"(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)"
app = typer.Typer(help=__doc__, no_args_is_help=False)


class BumpKind(str, Enum):
    """The release components that can be incremented."""

    major = "major"
    minor = "minor"
    patch = "patch"


def bump_version(version: str, kind: str) -> str:
    """Return the next major, minor, or patch version for a stable SemVer string."""
    match = re.fullmatch(r"(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)", version)
    if match is None:
        raise ValueError(f"Expected a stable semantic version, got {version!r}.")

    major, minor, patch = (int(part) for part in match.groups())
    if kind == "major":
        return f"{major + 1}.0.0"
    if kind == "minor":
        return f"{major}.{minor + 1}.0"
    if kind == "patch":
        return f"{major}.{minor}.{patch + 1}"
    raise ValueError(f"Unsupported version bump {kind!r}.")


def version_pattern(section: str) -> re.Pattern[str]:
    """Locate the one stable version field belonging to a TOML section."""
    return re.compile(
        rf"(?ms)(?P<prefix>^\[{re.escape(section)}\]\s*.*?^version\s*=\s*\")"
        rf"(?P<version>{STABLE_VERSION})(?P<suffix>\")"
    )


def read_version_file(text: str, section: str) -> str:
    """Read the stable version assigned in one TOML section."""
    match = version_pattern(section).search(text)
    if match is None:
        raise ValueError(f"Could not find [{section}] version.")
    return match.group("version")


def replace_version(text: str, section: str, version: str) -> str:
    """Replace exactly one TOML version field, preserving all other formatting."""
    pattern = version_pattern(section)

    def replacement(match: re.Match[str]) -> str:
        return f"{match.group('prefix')}{version}{match.group('suffix')}"

    result, replacements = pattern.subn(replacement, text)
    if replacements != 1:
        raise ValueError(f"Expected one [{section}] version field, found {replacements}.")
    return result


def replace_lock_package_version(text: str, package: str, version: str) -> str:
    """Replace one package version in a Cargo lockfile."""
    pattern = re.compile(
        rf'(?ms)(^\[\[package\]\]\s*^name\s*=\s*"{re.escape(package)}"\s*^version\s*=\s*")'
        rf'(?P<version>{STABLE_VERSION})(")'
    )

    def replacement(match: re.Match[str]) -> str:
        return f"{match.group(1)}{version}{match.group(3)}"

    result, replacements = pattern.subn(replacement, text)
    if replacements != 1:
        raise ValueError(f'Expected one Cargo.lock package entry for "{package}", found {replacements}.')
    return result


def run(command: list[str], repo: Path) -> str:
    """Run a release command in the repository and return its standard output."""
    print("+", " ".join(command))
    result = subprocess.run(command, cwd=repo, check=True, text=True, capture_output=True)
    if result.stdout.strip():
        print(result.stdout.strip())
    return result.stdout.strip()


def require_release_branch(repo: Path) -> None:
    """Refuse to release from anything other than the protected master branch."""
    branch = run(["git", "branch", "--show-current"], repo)
    if branch != "master":
        raise RuntimeError(f"Releases must start from master, not {branch or 'a detached HEAD'}.")

    changes = run(["git", "status", "--porcelain"], repo)
    if changes:
        raise RuntimeError("Release checkout is not clean; commit, stash, or discard its changes first.")


def create_release(repo: Path, version: str) -> None:
    """Commit the synchronized version files and create the matching annotated tag."""
    tag = f"v{version}"
    if run(["git", "tag", "--list", tag], repo):
        raise RuntimeError(f"Tag {tag} already exists.")

    run(["git", "add", "Cargo.toml", "Cargo.lock", "pyproject.toml", "uv.lock"], repo)
    run(["git", "commit", "-m", f"chore(release): {tag}"], repo)
    run(["git", "tag", "-a", tag, "-m", f"NGK {version}"], repo)
    print(f"Created commit and tag {tag}. Push them with: git push origin master --follow-tags")


def apply_bump(repo: Path, kind: str, release: bool) -> str:
    """Synchronize package versions and refresh both lockfiles."""
    cargo = repo / "Cargo.toml"
    cargo_lock = repo / "Cargo.lock"
    pyproject = repo / "pyproject.toml"
    lockfile = repo / "uv.lock"

    cargo_text = cargo.read_text(encoding="utf-8")
    pyproject_text = pyproject.read_text(encoding="utf-8")
    cargo_version = read_version_file(cargo_text, "package")
    python_version = read_version_file(pyproject_text, "project")
    if cargo_version != python_version:
        raise RuntimeError(
            f"Version sources disagree: Cargo.toml is {cargo_version}, pyproject.toml is {python_version}."
        )

    if release:
        require_release_branch(repo)

    next_version = bump_version(cargo_version, kind)
    originals = {
        cargo: cargo_text,
        cargo_lock: cargo_lock.read_text(encoding="utf-8"),
        pyproject: pyproject_text,
        lockfile: lockfile.read_text(encoding="utf-8"),
    }
    try:
        cargo.write_text(replace_version(cargo_text, "package", next_version), encoding="utf-8")
        cargo_lock.write_text(
            replace_lock_package_version(cargo_lock.read_text(encoding="utf-8"), "ngk", next_version),
            encoding="utf-8",
        )
        pyproject.write_text(replace_version(pyproject_text, "project", next_version), encoding="utf-8")
        run(["uv", "lock"], repo)
    except BaseException:
        for path, content in originals.items():
            path.write_text(content, encoding="utf-8")
        raise

    print(f"Bumped {cargo_version} -> {next_version}.")
    if release:
        create_release(repo, next_version)
    else:
        print("Review and commit the version files manually, or undo them and rerun with --release.")
    return next_version


@app.command()
def bump(
    kind: Annotated[BumpKind, typer.Argument(help="release component to increment")] = BumpKind.patch,
    release: Annotated[
        bool,
        typer.Option(help="commit the version update and create its annotated vX.Y.Z tag"),
    ] = False,
) -> None:
    """Increment the synchronized package version; patch is the default."""
    repo = Path(__file__).resolve().parents[2]
    apply_bump(repo, kind.value, release)


def main() -> None:
    """Run the release command-line application."""
    app()


__all__ = [
    "app",
    "apply_bump",
    "bump",
    "bump_version",
    "main",
    "read_version_file",
    "replace_lock_package_version",
    "replace_version",
]
