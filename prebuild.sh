#!/bin/bash

# Go to the Git root
cd "$(git rev-parse --show-toplevel)" || exit 1

# Base Git hash
VERSION=$(git rev-parse --short=8 HEAD)

# Check for dirty state
IS_DIRTY=false

# Uncommitted/staged changes
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "⚠️  WARNING: You have uncommitted changes!" >&2
    IS_DIRTY=true
fi

# Untracked files
UNTRACKED=$(git ls-files --others --exclude-standard)
if [ -n "$UNTRACKED" ]; then
    echo "⚠️  WARNING: You have untracked files:" >&2
    echo "$UNTRACKED" >&2
    IS_DIRTY=true
fi

# Check for unpushed commits
UPSTREAM=$(git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>/dev/null)
if [ $? -eq 0 ]; then
    if ! git diff --quiet "$UPSTREAM"..HEAD; then
        echo "⚠️  WARNING: You have commits that are not pushed to '$UPSTREAM'!" >&2
        IS_DIRTY=true
    fi
else
    echo "⚠️  WARNING: No upstream branch set. Cannot check for unpushed commits." >&2
    IS_DIRTY=true
fi

# Add -dirty suffix if needed
if [ "$IS_DIRTY" = true ]; then
    VERSION="${VERSION}**"
fi

# Output version info
echo "GIT HASH: $VERSION"
echo "#define GIT_HASH \"$VERSION\"" > Core/Inc/gitversion.h

# Touch to force rebuild
touch Core/Src/main.c

