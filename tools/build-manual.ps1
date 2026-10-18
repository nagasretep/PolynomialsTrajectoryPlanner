# Builds the user manual into build/UserManual.pdf (criterion A4).
# Requires pandoc and a LaTeX distribution with xelatex (MiKTeX or TeX Live) on the PATH.
# Run from anywhere:  powershell -ExecutionPolicy Bypass -File tools\build-manual.ps1
param(
    [string]$MainFont = "Cambria",
    [string]$MathFont = "Cambria Math"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root "build"
New-Item -ItemType Directory -Force -Path $build | Out-Null

$out = Join-Path $build "UserManual.pdf"
pandoc (Join-Path $root "manual\UserManual.md") -o $out --toc --toc-depth=2 --pdf-engine=xelatex `
    -V mainfont="$MainFont" -V mathfont="$MathFont" -V geometry:margin=2.5cm
if ($LASTEXITCODE -ne 0) { throw "pandoc failed (exit code $LASTEXITCODE). If lmodern.sty is missing, see roadmap/06-theory-library-roadmap.md." }
Write-Host "Written $out"
