#!/bin/bash

# Go to the Git root
cd "$(git rev-parse --show-toplevel)" || exit 1

# Get current Git hash
VERSION=$(git rev-parse --short=8 HEAD)
echo "GIT HASH: $VERSION"

# Write to header
echo "#define GIT_HASH \"$VERSION\"" > Core/Inc/gitversion.h

# Force rebuild
touch Core/Src/main.c

# === GIT STATE CHECKS ===

# Check for uncommitted (modified or staged) changes
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "⚠️  WARNING: You have uncommitted changes!" >&2
fi

# Check for untracked files
UNTRACKED=$(git ls-files --others --exclude-standard)
if [ -n "$UNTRACKED" ]; then
    echo "⚠️  WARNING: You have untracked files:" >&2
    echo "$UNTRACKED" >&2
fi

# Check for unpushed commits
UPSTREAM=$(git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>/dev/null)
if [ $? -eq 0 ]; then
    if ! git diff --quiet "$UPSTREAM"..HEAD; then
        echo "⚠️  WARNING: You have commits that are not pushed to '$UPSTREAM'!" >&2
    fi
else
    echo "⚠️  WARNING: No upstream branch set. Cannot check for unpushed commits." >&2
fi

