#!/bin/bash

# run_roofer_configs.sh
# Runs Roofer with multiple configurations found in a specified directory.

# Usage:
#   With inputs on command line:
#     ./run_roofer_configs.sh <input_pc> <input_footprint> <output_dir> <config_dir>
#   With inputs in config files:
#     ./run_roofer_configs.sh <output_dir> <config_dir>
#   Or with named arguments:
#     ./run_roofer_configs.sh --input-pc <pc> --input-footprint <fp> <output_dir> <config_dir>

POSITIONAL_ARGS=()
USE_RERUN=false
FILTER_VAL=""
INPUT_PC=""
INPUT_FOOTPRINT=""

while [[ $# -gt 0 ]]; do
  case $1 in
    --filter)
      FILTER_VAL="$2"
      shift # past argument
      shift # past value
      ;;
    --rerun)
      USE_RERUN=true
      shift # past argument
      ;;
    --input-pc)
      INPUT_PC="$2"
      shift # past argument
      shift # past value
      ;;
    --input-footprint)
      INPUT_FOOTPRINT="$2"
      shift # past argument
      shift # past value
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1")
      shift # past argument
      ;;
  esac
done

set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters

# Determine argument mode:
# - 4 positional args: <input_pc> <input_footprint> <output_dir> <config_dir>
# - 2 positional args: <output_dir> <config_dir> (inputs from config or named args)
if [ "$#" -eq 4 ]; then
    # Legacy mode: all 4 positional args provided
    INPUT_PC="$1"
    INPUT_FOOTPRINT="$2"
    OUTPUT_DIR="$3"
    CONFIG_DIR="$4"
    LEGACY_RERUN=""
elif [ "$#" -ge 5 ]; then
    # Legacy mode with optional rerun flag
    INPUT_PC="$1"
    INPUT_FOOTPRINT="$2"
    OUTPUT_DIR="$3"
    CONFIG_DIR="$4"
    LEGACY_RERUN="$5"
elif [ "$#" -eq 2 ]; then
    # Minimal mode: only output_dir and config_dir
    # INPUT_PC and INPUT_FOOTPRINT may come from --input-pc/--input-footprint or config files
    OUTPUT_DIR="$1"
    CONFIG_DIR="$2"
    LEGACY_RERUN=""
elif [ "$#" -eq 3 ]; then
    # Could be: <output_dir> <config_dir> <legacy_rerun> OR missing one input
    # We'll treat it as output_dir, config_dir, legacy_rerun
    OUTPUT_DIR="$1"
    CONFIG_DIR="$2"
    LEGACY_RERUN="$3"
else
    echo "Usage: $0 [options] <output_dir> <config_dir>"
    echo "       $0 [options] <input_pc> <input_footprint> <output_dir> <config_dir>"
    echo ""
    echo "Options:"
    echo "  --input-pc <path>        Path to input point cloud (optional if in config)"
    echo "  --input-footprint <path> Path to input footprint (optional if in config)"
    echo "  --filter <condition>     Filter condition"
    echo "  --rerun                  Enable rerun logging"
    echo ""
    echo "Input files can be provided either:"
    echo "  1. As positional arguments (4-arg mode)"
    echo "  2. As named arguments (--input-pc, --input-footprint)"
    echo "  3. In the config TOML files (pointcloud-path, polygon-source)"
    exit 1
fi

if [[ -n "$LEGACY_RERUN" && ( "$LEGACY_RERUN" == "true" || "$LEGACY_RERUN" == "yes" || "$LEGACY_RERUN" == "1" ) ]]; then
    USE_RERUN=true
fi

# Path to the Roofer executable
# Assuming the script is run from the project root and build is in ./build
ROOFER_EXEC="./build/apps/roofer-app/roofer"

if [ ! -f "$ROOFER_EXEC" ]; then
    echo "Error: Roofer executable not found at $ROOFER_EXEC"
    echo "Please build the project first or check the path."
    exit 1
fi

# Ensure output directory exists
if [ ! -d "$OUTPUT_DIR" ]; then
    echo "Creating output directory: $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"
fi

# Check if config directory exists
if [ ! -d "$CONFIG_DIR" ]; then
    echo "Error: Config directory not found at $CONFIG_DIR"
    exit 1
fi

# Determine Flags
ROOFER_ARGS=()

if [ "$USE_RERUN" = true ]; then
    ROOFER_ARGS+=(--rerun)
    echo "Rerun logging ENABLED"
else
    echo "Rerun logging DISABLED"
fi

if [ -n "$FILTER_VAL" ]; then
    ROOFER_ARGS+=(--filter "$FILTER_VAL")
    echo "Filtering ENABLED: $FILTER_VAL"
fi

echo "Starting Roofer batch processing..."
if [ -n "$INPUT_PC" ]; then
    echo "Input PC: $INPUT_PC"
else
    echo "Input PC: (from config files)"
fi
if [ -n "$INPUT_FOOTPRINT" ]; then
    echo "Input Footprint: $INPUT_FOOTPRINT"
else
    echo "Input Footprint: (from config files)"
fi
echo "Configs: $CONFIG_DIR"
echo "Output: $OUTPUT_DIR"

# Iterate through all .toml files in the config directory
shopt -s nullglob
for config_file in "$CONFIG_DIR"/*.toml; do
    filename=$(basename -- "$config_file")
    config_name="${filename%.*}"

    # Create a specific output directory for this configuration
    current_output_dir="$OUTPUT_DIR/$config_name"
    mkdir -p "$current_output_dir"

    echo "---------------------------------------------------"
    echo "Processing config: $filename"
    echo "Output directory: $current_output_dir"

    # Build the command arguments
    # Only include input files if they were provided (via CLI or named args)
    CMD_ARGS=(-c "$config_file" "${ROOFER_ARGS[@]}")

    if [ -n "$INPUT_PC" ] && [ -n "$INPUT_FOOTPRINT" ]; then
        # Both inputs provided - pass them to override config
        CMD_ARGS+=("$INPUT_PC" "$INPUT_FOOTPRINT" "$current_output_dir")
    else
        # Inputs should come from config file, only pass output dir as positional arg
        CMD_ARGS+=("$current_output_dir")
    fi

    # Execute Roofer
    "$ROOFER_EXEC" "${CMD_ARGS[@]}"

    status=$?
    if [ $status -eq 0 ]; then
        echo "Successfully finished: $config_name"
    else
        echo "Failed: $config_name with exit code $status"
    fi
done

echo "---------------------------------------------------"
echo "All configurations processed."
