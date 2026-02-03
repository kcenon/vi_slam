# Vocabulary Files

This directory contains vocabulary files required by certain SLAM frameworks. These files are not included in the repository due to their large size.

## Required Files

### ORB-SLAM3: ORBvoc.txt

The ORB vocabulary file is required for ORB-SLAM3 and VINS-Mono loop closure.

**Download Instructions:**

1. Download from the official ORB-SLAM3 repository:
   ```bash
   wget https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz
   tar -xzf ORBvoc.txt.tar.gz
   mv ORBvoc.txt vocab/
   rm ORBvoc.txt.tar.gz
   ```

2. Or use the provided script:
   ```bash
   ./scripts/download_vocab.sh
   ```

**File Details:**
- Size: ~120 MB (compressed ~40 MB)
- Format: Text file containing DBoW2 vocabulary
- Description: Pre-trained vocabulary with 10 levels and 6 branching factor (k10L6)

### VINS-Mono: brief_k10L6.bin

The BRIEF vocabulary file is required for VINS-Mono loop closure.

**Download Instructions:**

1. Download from the official VINS-Mono repository:
   ```bash
   wget https://github.com/HKUST-Aerial-Robotics/VINS-Mono/raw/master/support_files/brief_k10L6.bin
   mv brief_k10L6.bin vocab/
   ```

**File Details:**
- Size: ~40 MB
- Format: Binary DBoW2 vocabulary
- Description: Pre-trained BRIEF descriptor vocabulary

## Verification

After downloading, verify the files exist:

```bash
ls -la vocab/
# Expected output:
# ORBvoc.txt     (~120 MB)
# brief_k10L6.bin (~40 MB)
```

## Notes

- These vocabulary files are pre-trained and should not be modified.
- Different datasets may require re-training vocabularies for optimal performance.
- The vocabulary files are essential for loop closure functionality; without them, loop closure will be disabled.
- Store these files locally as they are too large for version control.
