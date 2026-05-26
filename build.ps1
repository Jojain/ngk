$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path $PSScriptRoot
Push-Location $RepoRoot

try {
    cargo build
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    if (-not $env:UV_CACHE_DIR) {
        $env:UV_CACHE_DIR = Join-Path $RepoRoot ".uv-cache"
    }
    if (-not $env:UV_PYTHON_INSTALL_DIR) {
        $env:UV_PYTHON_INSTALL_DIR = Join-Path $RepoRoot ".uv-python"
    }

    if (-not (Test-Path ".venv")) {
        uv venv --managed-python --python 3.12 .venv
        if ($LASTEXITCODE -ne 0) {
            exit $LASTEXITCODE
        }
    }

    $env:VIRTUAL_ENV = Join-Path $RepoRoot ".venv"
    $env:PATH = (Join-Path $env:VIRTUAL_ENV "Scripts") + ";" + $env:PATH

    uv run --no-project --managed-python --python 3.12 --with maturin maturin develop
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
}
