param(
    [string]$Script = "python/examples/explore_block.py"
)

$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Push-Location $RepoRoot

try {
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

    $Maturin = Get-ChildItem -Path $env:UV_CACHE_DIR -Recurse -Filter "maturin.exe" -ErrorAction SilentlyContinue |
        Select-Object -First 1 -ExpandProperty FullName

    if ($Maturin) {
        & $Maturin develop
    }
    else {
        uv run --no-project --managed-python --python 3.12 --with maturin maturin develop
    }
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    & (Join-Path $env:VIRTUAL_ENV "Scripts\python.exe") $Script
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
}
