#!/bin/bash

# run_roofer_configs.sh
# Runs Roofer with multiple configurations found in a specified directory.

# Usage: ./run_roofer_configs.sh <input_pc> <input_footprint> <output_dir> <config_dir> [use_rerun]

POSITIONAL_ARGS=()
USE_RERUN=false
FILTER_VAL=""

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

if [ "$#" -lt 4 ]; then
    echo "Usage: $0 [--filter \"condition\"] [--rerun] <input_pc> <input_footprint> <output_dir> <config_dir> [use_rerun_legacy]"
    exit 1
fi

INPUT_PC="$1"
INPUT_FOOTPRINT="$2"
OUTPUT_DIR="$3"
CONFIG_DIR="$4"
# Allow legacy 5th arg as fallback for rerun
LEGACY_RERUN="$5"

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
echo "Input PC: $INPUT_PC"
echo "Input Footprint: $INPUT_FOOTPRINT"
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

    # Execute Roofer
    # We pass the input args to override whatever is in the config file
    "$ROOFER_EXEC" -c "$config_file" "${ROOFER_ARGS[@]}" "$INPUT_PC" "$INPUT_FOOTPRINT" "$current_output_dir"

    status=$?
    if [ $status -eq 0 ]; then
        echo "Successfully finished: $config_name"
    else
        echo "Failed: $config_name with exit code $status"
    fi
done

echo "---------------------------------------------------"
echo "All configurations processed."
