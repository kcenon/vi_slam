#!/bin/bash
# Build script for Android application

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ANDROID_DIR="$PROJECT_ROOT/android"

cd "$ANDROID_DIR"

echo "Building Android application..."
./gradlew clean build

echo ""
echo "Build completed successfully!"
echo "APK location: $ANDROID_DIR/app/build/outputs/apk/debug/app-debug.apk"
