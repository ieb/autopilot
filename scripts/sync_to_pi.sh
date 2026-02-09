#!/bin/bash
#
# Sync changed files to Raspberry Pi
# ==================================
#
# Copies files that have changed since the local main branch
# to a Raspberry Pi via rsync (single connection, one password prompt).
#
# Usage:
#   ./scripts/sync_to_pi.sh user@host:path
#
# Examples:
#   ./scripts/sync_to_pi.sh pi@pilot1.local:autopilot
#   ./scripts/sync_to_pi.sh ieb@192.168.1.100:/media/ieb/disk1/autopilot
#   ./scripts/sync_to_pi.sh pi@pilot1.local:~/projects/autopilot
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Parse destination argument
DEST="$1"

if [ -z "$DEST" ]; then
    echo -e "${RED}ERROR: No destination specified${NC}"
    echo ""
    echo "Usage: $0 user@host:path"
    echo ""
    echo "Examples:"
    echo "  $0 pi@pilot1.local:autopilot"
    echo "  $0 ieb@192.168.1.100:/media/ieb/disk1/autopilot"
    echo "  $0 pi@pilot1.local:~/projects/autopilot"
    exit 1
fi

# Parse user@host:path format
if [[ ! "$DEST" =~ ^[^@]+@[^:]+:.+$ ]]; then
    echo -e "${RED}ERROR: Invalid destination format${NC}"
    echo ""
    echo "Expected format: user@host:path"
    echo "Example: pi@pilot1.local:autopilot"
    exit 1
fi

cd "$PROJECT_ROOT"

echo -e "${GREEN}Syncing to ${DEST}${NC}"
echo ""

# Find changed files compared to local main branch
# Include: modified, added, renamed files
# Exclude: deleted files (they don't exist to copy)
echo "Finding changed files (vs local main)..."

# 1. Committed changes on current branch vs main
COMMITTED_FILES=$(git diff --name-only --diff-filter=ACMR main...HEAD 2>/dev/null || \
                  git diff --name-only --diff-filter=ACMR main 2>/dev/null || \
                  echo "")

# 2. Uncommitted changes (staged + unstaged) to tracked files
UNCOMMITTED_FILES=$(git diff --name-only --diff-filter=ACMR HEAD 2>/dev/null || echo "")

# 3. Staged changes
STAGED_FILES=$(git diff --name-only --cached --diff-filter=ACMR 2>/dev/null || echo "")

# 4. Untracked files (new files not yet committed)
UNTRACKED_FILES=$(git ls-files --others --exclude-standard)

# Combine all sources
CHANGED_FILES=$(echo -e "${COMMITTED_FILES}\n${UNCOMMITTED_FILES}\n${STAGED_FILES}" | sort -u)

# Combine and deduplicate, filter to only existing files
ALL_FILES=$(echo -e "${CHANGED_FILES}\n${UNTRACKED_FILES}" | sort -u | grep -v '^$' || true)

# Filter to only files that exist
EXISTING_FILES=""
while IFS= read -r file; do
    if [ -n "$file" ] && [ -f "$file" ]; then
        EXISTING_FILES="${EXISTING_FILES}${file}"$'\n'
    fi
done <<< "$ALL_FILES"
EXISTING_FILES=$(echo "$EXISTING_FILES" | grep -v '^$' || true)

if [ -z "$EXISTING_FILES" ]; then
    echo -e "${YELLOW}No changed files to sync${NC}"
    exit 0
fi

# Count files
FILE_COUNT=$(echo "$EXISTING_FILES" | wc -l | tr -d ' ')
echo -e "Found ${GREEN}${FILE_COUNT}${NC} changed file(s):"
echo ""

# Show files to be synced
echo "$EXISTING_FILES" | while read -r file; do
    SIZE=$(ls -lh "$file" 2>/dev/null | awk '{print $5}')
    echo "  $file ($SIZE)"
done
echo ""

# Confirm before syncing
read -p "Sync these files to ${DEST}? [Y/n] " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Nn]$ ]]; then
    echo "Cancelled."
    exit 0
fi

echo ""
echo "Syncing files..."

# Create temporary file list for rsync
TMPFILE=$(mktemp)
echo "$EXISTING_FILES" > "$TMPFILE"

# Use rsync with --files-from for single connection
# -a: archive mode (preserves permissions, etc.)
# -v: verbose
# -R: use relative paths (preserves directory structure)
# --files-from: read file list from temp file
rsync -avR --files-from="$TMPFILE" . "${DEST}/"

rm -f "$TMPFILE"

echo ""
echo -e "${GREEN}Done!${NC} Synced ${FILE_COUNT} file(s)"
