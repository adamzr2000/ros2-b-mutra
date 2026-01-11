#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <commit message...>"
  exit 1
fi

COMMIT_MESSAGE="$*"

# Ensure we're in a git repo
git rev-parse --is-inside-work-tree >/dev/null

# Show what will be committed (safety)
echo "---- git status ----"
git status --short
echo "--------------------"

# Stage everything (same behavior as your script)
git add -A

# Only commit if there is something staged
if git diff --cached --quiet; then
  echo "Nothing staged to commit. Aborting."
  exit 0
fi

git commit -m "$COMMIT_MESSAGE"

# Push current branch to origin (avoids forced rename to main)
current_branch="$(git branch --show-current)"
git push -u origin "$current_branch"

echo "Pushed to origin/$current_branch"
