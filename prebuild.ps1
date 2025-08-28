# prebuild.ps1
# Windows PowerShell equivalent of your mac script

$ErrorActionPreference = 'Stop'

# Ensure we're inside a Git repo and cd to the repo root
$gitRoot = git rev-parse --show-toplevel 2>$null
if (-not $gitRoot) {
    Write-Error "Not a Git repository (run from within your project's Git repo)."
    exit 1
}
Set-Location $gitRoot

# Base Git hash
$VERSION = (git rev-parse --short=8 HEAD).Trim()

# Check for dirty state
$IS_DIRTY = $false

# Uncommitted changes
git diff --quiet | Out-Null
if ($LASTEXITCODE -ne 0) {
    Write-Warning "⚠️  WARNING: You have uncommitted changes!"
    $IS_DIRTY = $true
}

# Staged but uncommitted changes
git diff --cached --quiet | Out-Null
if ($LASTEXITCODE -ne 0) {
    Write-Warning "⚠️  WARNING: You have staged (but uncommitted) changes!"
    $IS_DIRTY = $true
}

# Untracked files
$UNTRACKED = git ls-files --others --exclude-standard
if ($UNTRACKED) {
    Write-Warning "⚠️  WARNING: You have untracked files:"
    $UNTRACKED | ForEach-Object { Write-Warning "  $_" }
    $IS_DIRTY = $true
}

# Check for unpushed commits
$UPSTREAM = git rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>$null
if (-not $UPSTREAM) {
    Write-Warning "⚠️  WARNING: No upstream branch set. Cannot check for unpushed commits."
    $IS_DIRTY = $true
} else {
    $ahead = [int](git rev-list --count "$UPSTREAM..HEAD")
    if ($ahead -gt 0) {
        Write-Warning "⚠️  WARNING: You have $ahead commit(s) not pushed to '$UPSTREAM'."
        $IS_DIRTY = $true
    }
}

# Add ** suffix if dirty
if ($IS_DIRTY) { $VERSION = "$VERSION**" }

# Output version info file (same path as your mac script)
$headerPath = Join-Path $gitRoot 'Core/Inc/gitversion.h'
"#define GIT_HASH `"$VERSION`"" | Set-Content -Path $headerPath -Encoding ascii

# Touch main.c to force rebuild (create if missing)
$mainPath = Join-Path $gitRoot 'Core/Src/main.c'
if (Test-Path $mainPath) {
    (Get-Item $mainPath).LastWriteTime = Get-Date
} else {
    New-Item -ItemType File -Path $mainPath -Force | Out-Null
}

Write-Host "GIT HASH: $VERSION"
