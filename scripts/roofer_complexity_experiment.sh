#!/bin/bash
# Script to run roofer with multiple complexity factor values
# Usage: ./roofer_complexity_experiment.sh [options] <pointcloud_path> <polygon_source> <output_dir> [complexity_values...]
#
# Options:
#   --no-rerun    Disable rerun visualization (enabled by default)
#   --rerun       Enable rerun visualization (default)
#
# Example:
#   ./roofer_complexity_experiment.sh ./data/pointcloud.laz ./data/footprints.gpkg ./output
#   ./roofer_complexity_experiment.sh --no-rerun ./data/pointcloud.laz ./data/footprints.gpkg ./output 0.1 0.3 0.5

set -e

# Get script directory to find the built roofer
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOFER_ROOT="$(dirname "$SCRIPT_DIR")"
ROOFER_BIN="${ROOFER_ROOT}/build/apps/roofer-app/roofer"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default options
RERUN_ENABLED=true
JOBS=""

# Print usage
usage() {
    echo "Usage: $0 [options] <pointcloud_path> <polygon_source> <output_dir> [complexity_values...]"
    echo ""
    echo "Options:"
    echo "  --rerun              Enable rerun visualization (default)"
    echo "  --no-rerun           Disable rerun visualization"
    echo "  --filter <clause>    OGR SQL WHERE clause to select specific features"
    echo "  -j, --jobs <N>       Number of threads (default: 1 when rerun enabled)"
    echo ""
    echo "Arguments:"
    echo "  pointcloud_path   Path to pointcloud file (.LAS or .LAZ) or folder"
    echo "  polygon_source    Path to roofprint polygon source (e.g., GPKG)"
    echo "  output_dir        Base output directory for results"
    echo "  complexity_values Optional list of complexity factors (0.0-1.0)"
    echo "                    Default: 0.0 0.2 0.4 0.6 0.8 1.0"
    echo ""
    echo "Example:"
    echo "  $0 ./pointcloud.laz ./footprints.gpkg ./output"
    echo "  $0 --filter \"id = '123'\" ./pointcloud.laz ./footprints.gpkg ./output"
    echo "  $0 --no-rerun ./pointcloud.laz ./footprints.gpkg ./output 0.1 0.3 0.5"
    exit 1
}

# Parse options
FILTER_CLAUSE=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --rerun)
            RERUN_ENABLED=true
            shift
            ;;
        --no-rerun)
            RERUN_ENABLED=false
            shift
            ;;
        --filter)
            if [ -z "$2" ] || [[ "$2" == -* ]]; then
                echo -e "${RED}Error: --filter requires a value${NC}"
                usage
            fi
            FILTER_CLAUSE="$2"
            shift 2
            ;;
        -j|--jobs)
            if [ -z "$2" ] || [[ "$2" == -* ]]; then
                echo -e "${RED}Error: -j/--jobs requires a value${NC}"
                usage
            fi
            JOBS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        -*)
            echo -e "${RED}Unknown option: $1${NC}"
            usage
            ;;
        *)
            break
            ;;
    esac
done

# Check minimum arguments
if [ "$#" -lt 3 ]; then
    usage
fi

# Parse arguments
POINTCLOUD_PATH="$1"
POLYGON_SOURCE="$2"
OUTPUT_BASE_DIR="$3"
shift 3

# Default complexity factor values if not provided
if [ "$#" -eq 0 ]; then
    COMPLEXITY_VALUES=(0.0 0.2 0.4 0.6 0.8 1.0)
else
    COMPLEXITY_VALUES=("$@")
fi

# Validate input files exist
if [ ! -e "$POINTCLOUD_PATH" ]; then
    echo -e "${RED}Error: Pointcloud path does not exist: $POINTCLOUD_PATH${NC}"
    exit 1
fi

if [ ! -e "$POLYGON_SOURCE" ]; then
    echo -e "${RED}Error: Polygon source does not exist: $POLYGON_SOURCE${NC}"
    exit 1
fi

# Check if built roofer binary exists
if [ ! -x "$ROOFER_BIN" ]; then
    echo -e "${RED}Error: Built roofer binary not found at: $ROOFER_BIN${NC}"
    echo -e "${RED}Please build roofer first with 'cmake --build build'${NC}"
    exit 1
fi

# Build rerun flag
if [ "$RERUN_ENABLED" = true ]; then
    RERUN_FLAG="--rerun"
    # Default to 1 job when rerun is enabled to ensure proper logging
    if [ -z "$JOBS" ]; then
        JOBS=1
    fi
else
    RERUN_FLAG="--no-rerun"
fi

# Create base output directory
mkdir -p "$OUTPUT_BASE_DIR"

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}Roofer Complexity Factor Experiment${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo "Roofer binary:  $ROOFER_BIN"
echo "Pointcloud:     $POINTCLOUD_PATH"
echo "Polygon source: $POLYGON_SOURCE"
echo "Output dir:     $OUTPUT_BASE_DIR"
echo "Rerun:          $RERUN_ENABLED"
if [ -n "$JOBS" ]; then
    echo "Jobs:           $JOBS"
fi
if [ -n "$FILTER_CLAUSE" ]; then
    echo "Filter:         $FILTER_CLAUSE"
fi
echo "Complexity values: ${COMPLEXITY_VALUES[*]}"
echo ""

# Track successful and failed runs
SUCCESSFUL=()
FAILED=()

# Run roofer for each complexity factor
for cf in "${COMPLEXITY_VALUES[@]}"; do
    # Format complexity factor for filename (replace . with _)
    CF_FILENAME=$(echo "$cf" | sed 's/\./_/g')

    # Create output subdirectory with complexity factor in name
    OUTPUT_DIR="${OUTPUT_BASE_DIR}/complexity_${CF_FILENAME}"
    mkdir -p "$OUTPUT_DIR"

    echo -e "${YELLOW}----------------------------------------${NC}"
    echo -e "${YELLOW}Running with complexity factor: $cf${NC}"
    echo "Output directory: $OUTPUT_DIR"
    echo -e "${YELLOW}----------------------------------------${NC}"

    # Build command arguments array to handle quoting properly
    CMD_ARGS=(
        "$POINTCLOUD_PATH"
        "$POLYGON_SOURCE"
        "$OUTPUT_DIR"
        --complexity-factor "$cf"
        "$RERUN_FLAG"
    )

    # Add filter argument if set (must be properly quoted)
    if [ -n "$FILTER_CLAUSE" ]; then
        CMD_ARGS+=(--filter "$FILTER_CLAUSE")
    fi

    # Add jobs argument if set
    if [ -n "$JOBS" ]; then
        CMD_ARGS+=(-j "$JOBS")
    fi

    # Run roofer with the specified complexity factor
    if "$ROOFER_BIN" "${CMD_ARGS[@]}"; then
        echo -e "${GREEN}✓ Completed: complexity factor $cf${NC}"
        SUCCESSFUL+=("$cf")
    else
        echo -e "${RED}✗ Failed: complexity factor $cf${NC}"
        FAILED+=("$cf")
    fi
    echo ""
done

# Print summary
echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}Experiment Complete${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo "Results saved in: $OUTPUT_BASE_DIR"
echo ""
echo -e "${GREEN}Successful runs (${#SUCCESSFUL[@]}):${NC}"
for cf in "${SUCCESSFUL[@]}"; do
    CF_FILENAME=$(echo "$cf" | sed 's/\./_/g')
    echo "  - complexity_${CF_FILENAME}/ (--complexity-factor $cf)"
done

if [ ${#FAILED[@]} -gt 0 ]; then
    echo ""
    echo -e "${RED}Failed runs (${#FAILED[@]}):${NC}"
    for cf in "${FAILED[@]}"; do
        echo "  - complexity factor $cf"
    done
fi

echo ""
echo "Output directories:"
ls -la "$OUTPUT_BASE_DIR"
