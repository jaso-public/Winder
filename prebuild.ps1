# Fail fast on script errors (optional)
$ErrorActionPreference = 'Stop'

# Go to the Git root
$gitRoot = git rev-parse --show-toplevel 2>$null
if (-not $gitRoot) {
    Write-Error "Not a git repository"
    exit 1
}
Set-Location $gitRoot

# Base Git hash
$VERSION = git rev-parse --short=8 HEAD

$IS_DIRTY = $false

# ---- Uncommitted/staged changes (use exit code) ----
git diff --quiet         >$null 2>&1
$wc1 = $LASTEXITCODE     # 0 = clean, 1 = changes
git diff --cached --quiet >$null 2>&1
$wc2 = $LASTEXITCODE

if (($wc1 -ne 0) -or ($wc2 -ne 0)) {
    Write-Warning "You have uncommitted or staged changes."
    $IS_DIRTY = $true
}

# ---- Untracked files ----
$UNTRACKED = git ls-files --others --exclude-standard
if ($UNTRACKED) {
    Write-Warning "You have untracked files:"
    $UNTRACKED | ForEach-Object { Write-Warning "  $_" }
    $IS_DIRTY = $true
}

# ---- Unpushed commits (use exit code) ----
$UPSTREAM = (git rev-parse --abbrev-ref --symbolic-full-name "@{u}" 2>$null)
if ($LASTEXITCODE -eq 0 -and $UPSTREAM) {
    $UPSTREAM = $UPSTREAM.Trim()
    git diff --quiet "$UPSTREAM"..HEAD >$null 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "You have commits not pushed to '$UPSTREAM'."
        $IS_DIRTY = $true
    }
} else {
    Write-Warning "No upstream branch set. Cannot check for unpushed commits."
    $IS_DIRTY = $true
}

# ---- Add ** suffix if dirty ----
if ($IS_DIRTY) {
    $VERSION = "${VERSION}**"
}

# ---- Write header (ensure folder exists) ----
$headerPath = Join-Path $gitRoot "Core\Inc\gitversion.h"
New-Item -ItemType Directory -Path (Split-Path $headerPath) -Force | Out-Null
"#define GIT_HASH `"$VERSION`"" | Out-File -Encoding ASCII -NoNewline $headerPath

# ---- Touch Core/Src/main.c ----
$mainFile = Join-Path $gitRoot "Core\Src\main.c"
if (Test-Path $mainFile) {
    (Get-Item $mainFile).LastWriteTime = Get-Date
}

Write-Output "GIT HASH: $VERSION"
