#!/bin/bash
# Verification script to check JAR contents

set -e

JAR_FILE=$(find build/libs -name "AdambotsLib-*.jar" -type f | head -1)

if [ -z "$JAR_FILE" ]; then
    echo "❌ ERROR: No JAR file found in build/libs/"
    echo "   Run './gradlew jar' first"
    exit 1
fi

echo "Checking JAR file: $JAR_FILE"
echo ""

# Check for docs/ and tests/
if jar tf "$JAR_FILE" | grep -E "^(docs|tests)" > /dev/null; then
    echo "❌ ERROR: Found docs/ or tests/ in JAR!"
    echo ""
    echo "Entries found:"
    jar tf "$JAR_FILE" | grep -E "^(docs|tests)"
    exit 1
else
    echo "✓ Confirmed: docs/ and tests/ are NOT in JAR"
fi

# Count entries
ENTRY_COUNT=$(jar tf "$JAR_FILE" | wc -l | tr -d ' ')
echo "✓ Total entries in JAR: $ENTRY_COUNT"

# Check for main compiled classes
if jar tf "$JAR_FILE" | grep "com/adambots/lib" > /dev/null; then
    echo "✓ Library classes found in JAR"
else
    echo "⚠️  WARNING: No library classes found in JAR!"
fi

# Check for backup folder
if jar tf "$JAR_FILE" | grep "backup/" > /dev/null; then
    echo "✓ Backup folder included in JAR"
else
    echo "⚠️  WARNING: No backup folder in JAR"
fi

echo ""
echo "JAR verification complete!"
