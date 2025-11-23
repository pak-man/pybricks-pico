#!/bin/bash
# Build script for Pybricks Motor Control
# Builds both Pico W and Pico 2 W firmware from unified source
export PICO_SDK_PATH=~/pico-sdk
export PICOTOOL_FETCH_FROM_GIT_PATH=~/picotool

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_BASE="${SCRIPT_DIR}/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_header() {
    echo -e "\n${YELLOW}============================================${NC}"
    echo -e "${YELLOW}$1${NC}"
    echo -e "${YELLOW}============================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# Check for PICO_SDK_PATH
if [ -z "$PICO_SDK_PATH" ]; then
    echo -e "${RED}Error: PICO_SDK_PATH not set${NC}"
    echo "Please set PICO_SDK_PATH to your Pico SDK installation"
    exit 1
fi

# Parse arguments
BUILD_PICOW=false
BUILD_PICO2W=false
CLEAN=false

if [ $# -eq 0 ]; then
    BUILD_PICOW=true
    BUILD_PICO2W=true
fi

for arg in "$@"; do
    case $arg in
        picow)
            BUILD_PICOW=true
            ;;
        pico2w)
            BUILD_PICO2W=true
            ;;
        all)
            BUILD_PICOW=true
            BUILD_PICO2W=true
            ;;
        clean)
            CLEAN=true
            ;;
        *)
            echo "Usage: $0 [picow|pico2w|all|clean]"
            echo "  picow   - Build for Pico W (RP2040)"
            echo "  pico2w  - Build for Pico 2 W (RP2350B)"
            echo "  all     - Build both (default)"
            echo "  clean   - Remove build directories"
            exit 1
            ;;
    esac
done

# Clean build directories if requested
if [ "$CLEAN" = true ]; then
    print_header "Cleaning build directories"
    rm -rf "${BUILD_BASE}_picow" "${BUILD_BASE}_pico2w"
    print_success "Clean complete"
    exit 0
fi

# Build Pico W firmware
if [ "$BUILD_PICOW" = true ]; then
    print_header "Building for Pico W (RP2040, 4 motors)"
    
    mkdir -p "${BUILD_BASE}_picow"
    cd "${BUILD_BASE}_picow"
    
    cmake -DTARGET_BOARD=picow "${SCRIPT_DIR}"
    make -j$(nproc)
    
    if [ -f "pybricks_picow.uf2" ]; then
        print_success "Pico W build complete: ${BUILD_BASE}_picow/pybricks_picow.uf2"
    else
        print_error "Pico W build failed"
        exit 1
    fi
fi

# Build Pico 2 W firmware
if [ "$BUILD_PICO2W" = true ]; then
    print_header "Building for Pico 2 W (RP2350B, 12 motors)"
    
    mkdir -p "${BUILD_BASE}_pico2w"
    cd "${BUILD_BASE}_pico2w"
    
    cmake -DTARGET_BOARD=pico2w "${SCRIPT_DIR}"
    make -j$(nproc)
    
    if [ -f "pybricks_pico2w.uf2" ]; then
        print_success "Pico 2 W build complete: ${BUILD_BASE}_pico2w/pybricks_pico2w.uf2"
    else
        print_error "Pico 2 W build failed"
        exit 1
    fi
fi

# Summary
print_header "Build Summary"

if [ "$BUILD_PICOW" = true ] && [ -f "${BUILD_BASE}_picow/pybricks_picow.uf2" ]; then
    SIZE=$(ls -lh "${BUILD_BASE}_picow/pybricks_picow.uf2" | awk '{print $5}')
    echo "  Pico W:   ${BUILD_BASE}_picow/pybricks_picow.uf2 ($SIZE)"
fi

if [ "$BUILD_PICO2W" = true ] && [ -f "${BUILD_BASE}_pico2w/pybricks_pico2w.uf2" ]; then
    SIZE=$(ls -lh "${BUILD_BASE}_pico2w/pybricks_pico2w.uf2" | awk '{print $5}')
    echo "  Pico 2 W: ${BUILD_BASE}_pico2w/pybricks_pico2w.uf2 ($SIZE)"
fi

echo ""
print_success "Done!"