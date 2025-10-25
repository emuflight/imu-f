#!/bin/sh

# Ensure the script runs from the repository root
if [ ! -f "src/version.h" ]; then
    echo "Error: This script must be run from the repository root."
    exit 1
fi

# --- Configuration ---
# Expected directory name of the extracted toolchain
TOOLCHAIN_DIR="gcc-arm-none-eabi-6-2017-q1-update"
VERSION_FILE="src/version.h"
BUILD_OUTPUT_FILE="output/F3.bin"

# --- 1. Conditional Download and Extraction ---

if [ ! -d "$TOOLCHAIN_DIR" ] || [ ! -f "$TOOLCHAIN_DIR/bin/arm-none-eabi-gcc" ]; then
    echo "Toolchain directory '$TOOLCHAIN_DIR' or required binary not found."

    # 1a. Detect Operating System
    OS=""
    case $(uname) in
        "Linux" )
            OS="linux"
            ;;
        "Darwin" )
            OS="mac"
            ;;
        * )
            echo "Error: Unsupported operating system ($(uname)). Exiting."
            exit 1
            ;;
    esac

    echo "Detected OS: $OS. Starting download..."

    # 1b. Construct Download URL
    DOWNLOAD_URL="https://developer.arm.com/-/media/Files/downloads/gnu-rm/6_1-2017q1/gcc-arm-none-eabi-6-2017-q1-update-${OS}.tar.bz2"

    # 1c. Download (silent and follow redirects) and pipe directly to tar for extraction
    # 'xjf' is used: 'x' extract, 'j' for bzip2, 'f' for file/stream
    if ! curl -L -# "$DOWNLOAD_URL" | tar -xjf -; then
        echo "========================================================================="
        echo "ERROR: Failed to download or extract the ARM toolchain."
        echo "Please check if 'curl' and 'tar' are installed and the URL is still valid."
        echo "========================================================================="
        exit 1
    fi

    echo "Download and extraction of $TOOLCHAIN_DIR completed successfully."
else
    echo "Toolchain directory '$TOOLCHAIN_DIR' and required binary found. Skipping download and extraction."
fi


# --- 2. Build Steps ---

echo "Setting up PATH..."

# Export the new PATH variable, using 'pwd' to ensure we get the absolute path
# to the toolchain's bin directory regardless of where the script is called from.
TOOLCHAIN_BIN_PATH="$(pwd)/$TOOLCHAIN_DIR/bin"
export PATH="$PATH:$TOOLCHAIN_BIN_PATH"

echo "Executing build script with: python make.py -C -T F3"

# Execute the python build script and check the result immediately
if ! python make.py -C -T F3; then
    echo "=========================================================="
    echo "❌ Build failed. Check the output above for errors."
    echo "=========================================================="
    exit 1
fi

echo "=========================================================="
echo "✅ Build completed successfully."
echo "=========================================================="
echo ""

# --- 3. Post-Build: Version Extraction and Binary Copy ---

echo "--- Post-Build Processing ---"

# 3a. Check for version file existence
if [ ! -f "$VERSION_FILE" ]; then
    echo "Error: Version file '$VERSION_FILE' not found. Cannot determine firmware version."
    exit 1
fi

# 3b. Extract FIRMWARE_VERSION using grep and awk (highly portable)
# Looks for the line, and prints the third field (the number).
FIRMWARE_VERSION=$(grep '^[[:space:]]*#define[[:space:]]*FIRMWARE_VERSION' "$VERSION_FILE" | awk '{print $3}')

# Basic validation: ensure extracted version is a non-empty decimal number
if [ -z "$FIRMWARE_VERSION" ] || ! expr "$FIRMWARE_VERSION" : '^[0-9]\+$' >/dev/null; then
    echo "Error: Could not extract a valid numerical FIRMWARE_VERSION from '$VERSION_FILE'."
    exit 1
fi

echo "Extracted Firmware Version: $FIRMWARE_VERSION"

# 3c. Define the new destination filename, ensuring it goes into the output directory
DEST_FILE="output/IMUF_${FIRMWARE_VERSION}.bin"

# 3d. Check if the built binary exists
if [ ! -f "$BUILD_OUTPUT_FILE" ]; then
    echo "Error: Build output file '$BUILD_OUTPUT_FILE' not found. Cannot proceed with copying."
    exit 1
fi

# 3e. Ensure output directory exists for the versioned binary
if [ ! -d "output" ]; then
    mkdir -p output
fi

# 3f. Copy the binary, preserving modification time, access time, and modes (-p)
echo "Copying '$BUILD_OUTPUT_FILE' to '$DEST_FILE'..."
if cp -p "$BUILD_OUTPUT_FILE" "$DEST_FILE"; then
    echo "=========================================================="
    echo "✅ Success: Binary copied to '$DEST_FILE'"
    echo "=========================================================="
else
    echo "=========================================================="
    echo "❌ Error: Failed to copy the binary."
    echo "=========================================================="
    exit 1
fi
