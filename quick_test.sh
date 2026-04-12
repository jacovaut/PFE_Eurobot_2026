#!/bin/bash
# Quick Test Script for Table Marker Detection Investigation
# This script runs both scenarios and compares the diagnostics

set -e

WORKSPACE="/home/isaac/PFE_Eurobot_2026/server_ws"
LOG_DIR="/tmp/marker_detection_test_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$LOG_DIR"

echo "=================================="
echo "Table Marker Detection Test Suite"
echo "=================================="
echo "Results saved to: $LOG_DIR"
echo ""

# Source ROS setup
cd "$WORKSPACE"
source install/setup.bash

echo "Building camera_localization package..."
colcon build --packages-select camera_localization > /dev/null 2>&1 || {
    echo "ERROR: Build failed"
    exit 1
}

echo "✓ Build successful"
echo ""

# Test parameters
DURATION=90  # seconds per test
WARMUP=5     # seconds to warmup before collecting metrics

run_scenario() {
    local name=$1
    local cmd=$2
    local log_file="$LOG_DIR/${name}.log"
    
    echo "=================================="
    echo "Scenario: $name"
    echo "Duration: $DURATION seconds (+ $WARMUP sec warmup)"
    echo "=================================="
    
    echo "Starting: $cmd"
    eval "$cmd > $log_file 2>&1" &
    local pid=$!
    
    echo "Process ID: $pid"
    
    # Let warmup complete
    sleep $WARMUP
    echo "Warmup complete, collecting metrics..."
    
    # Wait for test duration
    sleep $((DURATION - WARMUP))
    
    # Kill the process
    echo "Stopping process..."
    kill $pid 2>/dev/null || true
    sleep 2
    pkill -f "global_localization_node" 2>/dev/null || true
    pkill -f "camera_map_visualizer" 2>/dev/null || true
    
    echo "✓ Scenario complete"
    echo ""
}

# Scenario 1: Node Alone
run_scenario \
    "Node_Alone" \
    "ros2 run camera_localization global_localization_node"

# Wait between scenarios
echo "Waiting 5 seconds between scenarios..."
sleep 5

# Scenario 2: Both Nodes
run_scenario \
    "Node_with_Visualizer" \
    "ros2 run camera_localization global_localization_node & sleep 2 && ros2 run bringup camera_map_visualizer"

# Kill any remaining processes
pkill -f "global_localization_node" 2>/dev/null || true
pkill -f "camera_map_visualizer" 2>/dev/null || true

echo "=================================="
echo "Results Analysis"
echo "=================================="
echo ""

analyze_diagnostics() {
    local log_file=$1
    local scenario=$2
    
    if [ ! -f "$log_file" ]; then
        echo "No log file for $scenario"
        return
    fi
    
    echo "[$scenario]"
    
    # Count diagnostic lines
    local diag_count=$(grep -c "\[DIAGNOSTIC\]" "$log_file" || echo 0)
    echo "  Diagnostic measurements: $diag_count"
    
    # Show last diagnostic line
    local last_diag=$(grep "\[DIAGNOSTIC\]" "$log_file" | tail -1)
    if [ -n "$last_diag" ]; then
        echo "  Last measurement:"
        echo "    $last_diag" | sed 's/.*\[DIAGNOSTIC\] /      /'
    fi
    
    # Show detection statistics
    local last_detect=$(grep "\[DETECTION\]" "$log_file" | tail -1)
    if [ -n "$last_detect" ]; then
        echo "  Last detection stats:"
        echo "    $last_detect" | sed 's/.*\[DETECTION\] /      /'
    fi
    
    # Count successes/failures
    local total_detections=$(grep -c "\[DETECTION\]" "$log_file" || echo 0)
    
    if [ "$total_detections" -gt 0 ]; then
        # Extract solvePnP ok counts
        local total_ok=$(grep "\[DETECTION\]" "$log_file" | \
            grep -oE "solvePnP ok: [0-9]+" | \
            sed 's/solvePnP ok: //' | \
            awk '{s+=$1} END {print s}')
        
        local total_fail=$(grep "\[DETECTION\]" "$log_file" | \
            grep -oE "fail: [0-9]+" | \
            sed 's/fail: //' | \
            awk '{s+=$1} END {print s}')
        
        if [ -n "$total_ok" ] && [ -n "$total_fail" ]; then
            local total=$((total_ok + total_fail))
            local percentage=$((total_ok * 100 / (total_ok + total_fail)))
            echo "  Overall solvePnP success rate: $percentage% ($total_ok/$total)"
        fi
    fi
    
    echo ""
}

analyze_diagnostics "$LOG_DIR/Node_Alone.log" "Node Running Alone"
analyze_diagnostics "$LOG_DIR/Node_with_Visualizer.log" "Node with Visualizer"

echo "=================================="
echo "Key Metrics to Compare"
echo "=================================="
echo ""
echo "1. solvePnP success rate:"
echo "   - Should be SIMILAR in both scenarios (~80-100%)"
echo "   - If Node_Alone < Node_with_Visualizer, QoS fix may help"
echo ""
echo "2. Empty frames:"
echo "   - Should be 0 in both scenarios"
echo "   - If > 0, indicates camera buffer or timing issues"
echo ""
echo "3. Rescue pass effectiveness:"
echo "   - 'Rescue helped' counter should be low"
echo "   - If high, markers are being missed initially"
echo ""
echo "4. Processing time:"
echo "   - Should be consistent between scenarios"
echo "   - Variance indicates CPU scheduling differences"
echo ""

echo "=================================="
echo "Interpreting Results"
echo "=================================="
echo ""
echo "IF PROBLEM IS FIXED (QoS was the issue):"
echo "  ✓ Both scenarios have similar solvePnP success rates"
echo "  ✓ No rescue pass help needed (low counter)"
echo "  ✓ No empty frames"
echo ""
echo "IF PROBLEM PERSISTS (other issue):"
echo "  ✗ Node-alone scenario still shows lower success rate"
echo "  ✗ Diagnostics will help identify the bottleneck"
echo ""

echo ""
echo "Full logs available in: $LOG_DIR"
echo "Complete diagnostic output with all measurements in the log files above"
echo ""
