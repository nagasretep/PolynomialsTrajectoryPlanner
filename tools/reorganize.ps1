# reorganize.ps1 - one-shot repository reorganization (see roadmap/decisions/ADR-0001).
# Run ONCE from anywhere:  powershell -ExecutionPolicy Bypass -File tools\reorganize.ps1
# Prerequisites: Visual Studio closed; baseline commit done (git status clean except the new files).
# It uses "git mv" (history preserved), untracks bin/obj/.vs, and does NOT commit.

$ErrorActionPreference = 'Stop'
Set-Location (Split-Path -Parent $PSScriptRoot)

if (-not (Test-Path '.git'))        { throw 'Not at the repository root.' }
if (-not (Test-Path 'project\src')) { throw 'project\src not found: already reorganized?' }
if (Get-Process devenv -ErrorAction SilentlyContinue) { throw 'Close Visual Studio first.' }

function Test-Tracked([string]$path) {
    $out = git ls-files -- $path
    return [bool]$out
}

function Move-Smart([string]$from, [string]$to) {
    if (-not (Test-Path $from)) { Write-Warning "missing, skipped: $from"; return }
    $dir = Split-Path $to -Parent
    if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir | Out-Null }
    if (Test-Tracked $from) { git mv -- $from $to } else { Move-Item -- $from $to }
    if ($LASTEXITCODE -ne 0) { throw "move failed: $from -> $to" }
    Write-Host "moved: $from -> $to"
}

# 1. Stop tracking build output and IDE state, then delete the (regenerable) folders.
$junk = @(
    'project\.vs',
    'project\src\TrajectoryPlanner.App\bin',  'project\src\TrajectoryPlanner.App\obj',
    'project\src\TrajectoryPlanner.Core\bin', 'project\src\TrajectoryPlanner.Core\obj',
    'project\src\TrajectoryPlanner.Core.Tests\bin', 'project\src\TrajectoryPlanner.Core.Tests\obj'
)
foreach ($j in $junk) {
    git rm -r --cached -q --ignore-unmatch -- $j
    if (Test-Path $j) {
        try { Remove-Item $j -Recurse -Force } catch { Write-Warning "could not delete $j : $_" }
    }
}

# 2. Specification
foreach ($f in '00-vision','01-requirements','02-data-model','03-assumptions','04-future-topics','05-success-criteria') {
    Move-Smart "project\docs\$f.md" "spec\$f.md"
}

# 3. Theory library
foreach ($f in 'SymbolsAndNomenclature','NinthDegreePolynomialTheory','BoundaryConditions',
               'PolynomialCoefficientDetermination','TrajectoryConstraints','ConstantVelocitySegment',
               'BlendingSegments','TrajectoryPassingThroughConstraintPoints','BookHeader') {
    Move-Smart "project\docs\$f.md" "theory\$f.md"
}

# 4. Design and roadmap
Move-Smart 'project\docs\07-implementation-plan.md'     'design\07-implementation-plan.md'
Move-Smart 'project\docs\08-ui-first-prototype.md'      'design\08-ui-first-prototype.md'
Move-Smart 'project\docs\06-theory-library-roadmap.md'  'roadmap\06-theory-library-roadmap.md'

# 5. Archive (original brief, generated books)
Move-Smart 'PolynomialTrajectoryPlannerApp.docx'        'archive\PolynomialTrajectoryPlannerApp.docx'
Move-Smart 'project\docs\specs_and_docs_book.pdf'       'archive\pdf\specs_and_docs_book.pdf'
Move-Smart 'project\docs\specs_and_docs_book_2.pdf'     'archive\pdf\specs_and_docs_book_2.pdf'

# 6. Code, solution, assets
Move-Smart 'project\src'                                'src'
Move-Smart 'project\PolynomialsTrajectoryPlanner.slnx'  'PolynomialsTrajectoryPlanner.slnx'
Move-Smart 'IconsAndGraphics'                           'assets'

# 7. Old README is replaced by the new root README.md
if (Test-Path 'project\README.md') { git rm -f -q -- 'project\README.md' }

# 8. Fix the icon path in the App project (src\TrajectoryPlanner.App -> ..\..\assets)
$csproj = 'src\TrajectoryPlanner.App\TrajectoryPlanner.App.csproj'
$text = [System.IO.File]::ReadAllText((Resolve-Path $csproj))
$new  = $text.Replace('..\..\..\IconsAndGraphics\calculus.png', '..\..\assets\calculus.png')
if ($new -eq $text) { Write-Warning 'icon path not found in csproj: check manually' }
else { [System.IO.File]::WriteAllText((Resolve-Path $csproj), $new); Write-Host 'csproj icon path updated' }

# 9. Remove the empty "project" folder (only if nothing but leftovers remain)
if (Test-Path 'project') {
    $left = Get-ChildItem 'project' -Force -Recurse -File | Where-Object { $_.FullName -notlike '*\.vs\*' }
    if (-not $left) { try { Remove-Item 'project' -Recurse -Force } catch { Write-Warning "could not delete project : $_" } }
    else { Write-Warning 'project\ still contains files:'; $left | ForEach-Object { Write-Warning $_.FullName } }
}

Write-Host ''
Write-Host 'Done. Review with: git status'
Write-Host 'Then verify:       dotnet build PolynomialsTrajectoryPlanner.slnx ; dotnet test src\TrajectoryPlanner.Core.Tests'
Write-Host 'No commit was made.'
