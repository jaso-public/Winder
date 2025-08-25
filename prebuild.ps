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

# Uncommitted/staged changes
if (-not (git diff --quiet) -or -not (git diff --cached --quiet)) {
    Write-Warning "⚠️  You have uncommitted changes!"
    $IS_DIRTY = $true
}

# Untracked files
$UNTRACKED = git ls-files --others --exclude-standard
if ($UNTRACKED) {
    Write-Warning "⚠️  You have untracked files:"
    $UNTRACKED | ForEach-Object { Write-Warning "  $_" }
    $IS_DIRTY = $true
}

# Check for unpushed commits
$UPSTREAM = git rev-parse --abbrev-ref --symbolic-full-name "@{u}" 2>$null
if ($LASTEXITCODE -eq 0) {
    if (-not (git diff --quiet "$UPSTREAM"..HEAD)) {
        Write-Warning "⚠️  You have commits not pushed to '$UPSTREAM'!"
        $IS_DIRTY = $true
    }
} else {
    Write-Warning "⚠️  No upstream branch set. Cannot check for unpushed commits."
    $IS_DIRTY = $true
}

# Add -dirty suffix if needed
if ($IS_DIRTY) {
    $VERSION = "${VERSION}**"
}

# Output version info
Write-Output "GIT HASH: $VERSION"
"#define GIT_HASH `"$VERSION`"" | Out-File -Encoding ASCII "Core\Inc\gitversion.h"

# Touch Core/Src/main.c (update timestamp)
$mainFile = "Core\Src\main.c"
if (Test-Path $mainFile) {
    (Get-Item $mainFile).LastWriteTime = Get-Date
}

