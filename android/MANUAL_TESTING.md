# Manual Testing Guide for History Screen

This document provides manual testing procedures for the Session History Screen (Issue #114).

## Prerequisites

- Android device or emulator running Android API 24+
- At least one recorded session (create via Main Screen)

## Test Cases

### 1. Session List Display

**Steps:**
1. Launch the app
2. Tap the "History" button on the Main Screen
3. Verify session list displays correctly

**Expected Results:**
- Each session card shows:
  - Session ID
  - Timestamp (formatted as "MMM dd, yyyy HH:mm")
  - Duration (formatted as HH:MM:SS or MM:SS)
  - File size (formatted with appropriate unit: B, KB, MB, GB)
  - Frame count
  - IMU sample count
- List scrolls smoothly
- Material Design 3 styling applied

### 2. Empty State

**Steps:**
1. Delete all sessions (or use fresh install)
2. Navigate to History Screen

**Expected Results:**
- "No Sessions" title displayed
- "Start recording to create your first session" subtitle shown
- No session list visible

### 3. Delete Confirmation

**Steps:**
1. Navigate to History Screen with at least one session
2. Tap the delete icon (red trash icon) on a session
3. Verify confirmation dialog appears

**Expected Results:**
- Dialog shows "Delete Session" title
- Warning message: "Are you sure you want to delete this session? This action cannot be undone."
- Two buttons: "Cancel" and "Delete" (red)

### 4. Delete Session

**Steps:**
1. Follow steps 1-3 from "Delete Confirmation"
2. Tap "Delete" button
3. Verify session is removed

**Expected Results:**
- Session disappears from list
- Remaining sessions remain intact
- If no sessions remain, empty state shown

### 5. Search Functionality

**Steps:**
1. Navigate to History Screen
2. Tap the search icon in top bar
3. Type session ID or partial ID
4. Verify filtered results

**Expected Results:**
- Search bar appears with "Search sessions..." placeholder
- List filters in real-time as you type
- Case-insensitive search
- Back arrow closes search and clears filter

### 6. Sort Functionality

**Steps:**
1. Navigate to History Screen
2. Tap "Sort" button in top bar
3. Select different sort criteria

**Expected Results:**
- Sort dialog shows three options:
  - "Date (newest first)" - default
  - "Duration (longest first)"
  - "Size (largest first)"
- Radio button indicates current selection
- Selecting option sorts list immediately and closes dialog

### 7. Navigation

**Steps:**
1. From Main Screen, tap "History" button
2. From History Screen, tap back arrow

**Expected Results:**
- History Screen opens from Main Screen
- Back button returns to Main Screen
- No crash or navigation errors

## Known Limitations

- **Session detail view**: Currently shows TODO message (to be implemented in future)
- **Export functionality**: Currently shows TODO message (to be implemented in future)
- **Thumbnail images**: Shows placeholder gray box (actual thumbnails not yet generated)

## Build and Installation

Due to Java version compatibility issues with Java 25.0.1, manual testing is recommended:

```bash
# Switch to Java 21 (if available)
export JAVA_HOME=/path/to/java21

# Build and install
./gradlew :app:assembleDebug
./gradlew :app:installDebug
```

Or use Android Studio:
1. Open project in Android Studio
2. Select "Run" > "Run 'app'"
3. Choose device/emulator

## Automated Testing

Unit tests for HistoryViewModel:
```bash
./gradlew :app:testDebugUnitTest --tests "com.vi.slam.android.ui.HistoryViewModelTest"
```

**Note**: Requires Java 21 or earlier due to Kotlin compiler compatibility.
