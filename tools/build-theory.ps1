# Builds the theory library into build/PolynomialsTrajectoryPlanner.pdf (criterion D2).
# Requires pandoc and a LaTeX distribution with xelatex (MiKTeX or TeX Live) on the PATH.
# Run from anywhere:  powershell -ExecutionPolicy Bypass -File tools\build-theory.ps1
param(
    [string]$MainFont = "Cambria",
    [string]$MathFont = "Cambria Math"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$theory = Join-Path $root "theory"
$build = Join-Path $root "build"
New-Item -ItemType Directory -Force -Path $build | Out-Null

$docs = @(
    "BookHeader", "SymbolsAndNomenclature", "NinthDegreePolynomialTheory", "BoundaryConditions",
    "PolynomialCoefficientDetermination", "TrajectoryConstraints", "ConstantVelocitySegment",
    "BlendingSegments", "TrajectoryPassingThroughConstraintPoints"
) | ForEach-Object { Join-Path $theory "$_.md" }

$out = Join-Path $build "PolynomialsTrajectoryPlanner.pdf"
pandoc @docs -o $out --toc --toc-depth=2 --pdf-engine=xelatex `
    -V mainfont="$MainFont" -V mathfont="$MathFont" -V geometry:margin=2.5cm
if ($LASTEXITCODE -ne 0) { throw "pandoc failed (exit code $LASTEXITCODE). If lmodern.sty is missing, install it or see roadmap/06-theory-library-roadmap.md." }
Write-Host "Written $out"
