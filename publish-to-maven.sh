#!/bin/bash
# Script to publish AdambotsLib to the maven directory for GitHub Pages

echo "Publishing AdambotsLib to maven directory..."

# Build the library
./gradlew clean build publish

# Create maven directory if it doesn't exist
mkdir -p maven

# Copy the published artifacts to the maven directory
echo "Copying artifacts to maven directory..."
cp -r build/maven/* maven/

echo "Done! The maven directory now contains:"
ls -R maven/

echo ""
echo "Next steps:"
echo "1. Commit the changes: git add maven/ AdambotsLib.json"
echo "2. Push to GitHub: git push"
echo "3. Make sure GitHub Pages is enabled for your repository"
